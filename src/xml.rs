use roxmltree::{Document, Node};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};
use thiserror::Error;

pub type PdoEntry = (u16, u8, u16);
pub type PdoMap = BTreeMap<u16, Vec<PdoEntry>>;
pub type SyncManagerMap = BTreeMap<u16, u8>;

const STD_DATATYPE_BITS: &[(&str, u16)] = &[
    ("BOOL", 1),
    ("BIT", 1),
    ("SINT", 8),
    ("USINT", 8),
    ("BYTE", 8),
    ("INT", 16),
    ("UINT", 16),
    ("WORD", 16),
    ("DINT", 32),
    ("UDINT", 32),
    ("DWORD", 32),
    ("LINT", 64),
    ("ULINT", 64),
    ("LWORD", 64),
    ("REAL", 32),
    ("LREAL", 64),
];

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct DecodeEsiOptions {
    pub vendor_id: Option<u32>,
    pub product_code: Option<u32>,
    pub revision_no: Option<u32>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedEsi {
    pub vendor_id: Option<u32>,
    pub product_code: Option<u32>,
    pub revision_no: Option<u32>,
    pub device_name: Option<String>,
    pub rx_pdos: Vec<u16>,
    pub tx_pdos: Vec<u16>,
    pub pdo_map_rx: PdoMap,
    pub pdo_map_tx: PdoMap,
    pub pdo_sm_rx: SyncManagerMap,
    pub pdo_sm_tx: SyncManagerMap,
    pub supports: BTreeMap<String, bool>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParsedEsiFeatures {
    pub rx_pdos: Vec<u16>,
    pub tx_pdos: Vec<u16>,
    pub pdo_entries_rx: Vec<PdoEntry>,
    pub pdo_entries_tx: Vec<PdoEntry>,
    pub pdo_map_rx: PdoMap,
    pub pdo_map_tx: PdoMap,
    pub pdo_sm_rx: SyncManagerMap,
    pub pdo_sm_tx: SyncManagerMap,
    pub supports: BTreeMap<String, bool>,
}

#[derive(Debug, Error)]
pub enum XmlDecodeError {
    #[error("failed to read ESI file `{path}`: {source}")]
    Read {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to parse ESI XML: {0}")]
    Parse(#[from] roxmltree::Error),
}

pub fn decode_esi_file(
    path: impl AsRef<Path>,
    options: DecodeEsiOptions,
) -> Result<DecodedEsi, XmlDecodeError> {
    let path_ref = path.as_ref();
    let xml = fs::read_to_string(path_ref).map_err(|source| XmlDecodeError::Read {
        path: path_ref.to_path_buf(),
        source,
    })?;
    decode_esi_str(&xml, options)
}

pub fn decode_esi_str(xml: &str, options: DecodeEsiOptions) -> Result<DecodedEsi, XmlDecodeError> {
    let doc = Document::parse(xml)?;
    let root = doc.root_element();

    let doc_vendor_id = descendants_by_localname(root, "Vendor")
        .into_iter()
        .find_map(|vendor| findtext_first_child(vendor, &["Id"]).and_then(parse_u32));

    let Some(chosen) = select_device(root, options.product_code, options.revision_no) else {
        return Ok(DecodedEsi {
            vendor_id: options.vendor_id.or(doc_vendor_id),
            product_code: options.product_code,
            revision_no: options.revision_no,
            device_name: None,
            rx_pdos: Vec::new(),
            tx_pdos: Vec::new(),
            pdo_map_rx: BTreeMap::new(),
            pdo_map_tx: BTreeMap::new(),
            pdo_sm_rx: BTreeMap::new(),
            pdo_sm_tx: BTreeMap::new(),
            supports: BTreeMap::new(),
        });
    };

    let (_, chosen_product, chosen_revision, device_name, _) = device_identity(chosen);
    let effective_vendor = options.vendor_id.or(doc_vendor_id);
    let effective_product = options.product_code.or(chosen_product);
    let effective_revision = options.revision_no.or(chosen_revision);
    let datatype_bits = build_datatype_bits_table(chosen);

    let mut rx_pdos = Vec::new();
    let mut tx_pdos = Vec::new();
    let mut pdo_map_rx = BTreeMap::new();
    let mut pdo_map_tx = BTreeMap::new();
    let mut pdo_sm_rx = BTreeMap::new();
    let mut pdo_sm_tx = BTreeMap::new();

    let rx_elems = descendants_by_localname(chosen, "RxPdo");
    let tx_elems = descendants_by_localname(chosen, "TxPdo");

    let mut parse_rxtx_pdo = |pdo_elem: Node<'_, '_>, direction: Direction| {
        let Some(pdo_index) = findtext_first_child(pdo_elem, &["Index"]).and_then(parse_u16) else {
            return;
        };

        let sm = pdo_elem
            .attribute("Sm")
            .and_then(parse_u8)
            .unwrap_or_else(|| match direction {
                Direction::Rx => 2,
                Direction::Tx => 3,
            });

        let entries = descendants_by_localname(pdo_elem, "Entry")
            .into_iter()
            .filter_map(|entry| parse_entry_triplet(entry, &datatype_bits))
            .collect::<Vec<_>>();

        match direction {
            Direction::Rx => {
                rx_pdos.push(pdo_index);
                pdo_map_rx
                    .entry(pdo_index)
                    .or_insert_with(Vec::new)
                    .extend(entries);
                pdo_sm_rx.insert(pdo_index, sm);
            }
            Direction::Tx => {
                tx_pdos.push(pdo_index);
                pdo_map_tx
                    .entry(pdo_index)
                    .or_insert_with(Vec::new)
                    .extend(entries);
                pdo_sm_tx.insert(pdo_index, sm);
            }
        }
    };

    if !rx_elems.is_empty() || !tx_elems.is_empty() {
        for rx in rx_elems {
            parse_rxtx_pdo(rx, Direction::Rx);
        }
        for tx in tx_elems {
            parse_rxtx_pdo(tx, Direction::Tx);
        }
    } else {
        for sm_elem in descendants_by_localname(chosen, "Sm") {
            let sm_idx = sm_elem.attribute("Index").and_then(parse_u8).or_else(|| {
                findtext_first_child(sm_elem, &["Index"])
                    .as_deref()
                    .and_then(parse_u8)
            });

            let dir_text = findtext_first_child(sm_elem, &["Dir"])
                .or_else(|| sm_elem.attribute("Dir").map(str::to_owned))
                .unwrap_or_default()
                .trim()
                .to_ascii_lowercase();

            let sm_name = sm_elem
                .text()
                .unwrap_or_default()
                .trim()
                .to_ascii_lowercase();

            let is_rx_sm = matches!(dir_text.as_str(), "out" | "output")
                || matches!(sm_name.as_str(), "outputs" | "output")
                || sm_idx == Some(2);

            let is_tx_sm = matches!(dir_text.as_str(), "in" | "input")
                || matches!(sm_name.as_str(), "inputs" | "input")
                || sm_idx == Some(3);

            for pdo in descendants_by_localname(sm_elem, "Pdo") {
                let Some(pdo_index) = findtext_first_child(pdo, &["Index"]).and_then(parse_u16)
                else {
                    continue;
                };

                let entries = descendants_by_localname(pdo, "Entry")
                    .into_iter()
                    .filter_map(|entry| parse_entry_triplet(entry, &datatype_bits))
                    .collect::<Vec<_>>();

                if is_rx_sm {
                    rx_pdos.push(pdo_index);
                    pdo_map_rx
                        .entry(pdo_index)
                        .or_insert_with(Vec::new)
                        .extend(entries.clone());
                    if let Some(sm_idx) = sm_idx {
                        pdo_sm_rx.insert(pdo_index, sm_idx);
                    }
                }

                if is_tx_sm {
                    tx_pdos.push(pdo_index);
                    pdo_map_tx
                        .entry(pdo_index)
                        .or_insert_with(Vec::new)
                        .extend(entries);
                    if let Some(sm_idx) = sm_idx {
                        pdo_sm_tx.insert(pdo_index, sm_idx);
                    }
                }
            }
        }
    }

    dedup_preserve_order(&mut rx_pdos);
    dedup_preserve_order(&mut tx_pdos);

    let flat_rx = pdo_map_rx.values().flatten().copied().collect::<Vec<_>>();
    let flat_tx = pdo_map_tx.values().flatten().copied().collect::<Vec<_>>();
    let combined = flat_rx
        .iter()
        .chain(flat_tx.iter())
        .copied()
        .collect::<Vec<_>>();

    let mut supports = BTreeMap::new();
    supports.insert(
        "modes_pp".to_string(),
        flat_rx.iter().any(|entry| entry.0 == 0x607A),
    );
    supports.insert(
        "modes_pv".to_string(),
        flat_rx.iter().any(|entry| entry.0 == 0x60FF),
    );
    supports.insert(
        "modes_csp".to_string(),
        flat_rx.iter().any(|entry| entry.0 == 0x607A),
    );
    supports.insert(
        "touch_probe".to_string(),
        combined
            .iter()
            .any(|entry| matches!(entry.0, 0x60B8 | 0x60B9 | 0x60BA | 0x60BB | 0x60BC)),
    );
    supports.insert(
        "statusword".to_string(),
        flat_tx.iter().any(|entry| entry.0 == 0x6041),
    );
    supports.insert(
        "controlword".to_string(),
        flat_rx.iter().any(|entry| entry.0 == 0x6040),
    );
    supports.insert(
        "mode_display".to_string(),
        flat_tx.iter().any(|entry| entry.0 == 0x6061),
    );
    supports.insert(
        "mode_command".to_string(),
        flat_rx.iter().any(|entry| entry.0 == 0x6060),
    );

    Ok(DecodedEsi {
        vendor_id: effective_vendor,
        product_code: effective_product,
        revision_no: effective_revision,
        device_name,
        rx_pdos,
        tx_pdos,
        pdo_map_rx,
        pdo_map_tx,
        pdo_sm_rx,
        pdo_sm_tx,
        supports,
    })
}

pub fn parse_esi_features_file(
    path: impl AsRef<Path>,
) -> Result<ParsedEsiFeatures, XmlDecodeError> {
    let decoded = decode_esi_file(path, DecodeEsiOptions::default())?;
    Ok(parsed_features_from_decoded(decoded))
}

pub fn parse_esi_features_str(xml: &str) -> Result<ParsedEsiFeatures, XmlDecodeError> {
    let decoded = decode_esi_str(xml, DecodeEsiOptions::default())?;
    Ok(parsed_features_from_decoded(decoded))
}

fn parsed_features_from_decoded(decoded: DecodedEsi) -> ParsedEsiFeatures {
    let pdo_entries_rx = decoded
        .pdo_map_rx
        .values()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
    let pdo_entries_tx = decoded
        .pdo_map_tx
        .values()
        .flatten()
        .copied()
        .collect::<Vec<_>>();

    ParsedEsiFeatures {
        rx_pdos: decoded.rx_pdos,
        tx_pdos: decoded.tx_pdos,
        pdo_entries_rx,
        pdo_entries_tx,
        pdo_map_rx: decoded.pdo_map_rx,
        pdo_map_tx: decoded.pdo_map_tx,
        pdo_sm_rx: decoded.pdo_sm_rx,
        pdo_sm_tx: decoded.pdo_sm_tx,
        supports: decoded.supports,
    }
}

fn descendants_by_localname<'a, 'input>(
    node: Node<'a, 'input>,
    local: &str,
) -> Vec<Node<'a, 'input>> {
    node.descendants()
        .filter(|candidate| candidate.is_element() && candidate.tag_name().name() == local)
        .collect()
}

fn find_first_child_by_localname<'a, 'input>(
    node: Node<'a, 'input>,
    local: &str,
) -> Option<Node<'a, 'input>> {
    node.children()
        .find(|child| child.is_element() && child.tag_name().name() == local)
}

fn findtext_first_child(node: Node<'_, '_>, candidates: &[&str]) -> Option<String> {
    for candidate in candidates {
        if let Some(child) = find_first_child_by_localname(node, candidate) {
            let text = child.text().unwrap_or_default().trim();
            if !text.is_empty() {
                return Some(text.to_string());
            }
        }
    }
    None
}

fn parse_u32(text: String) -> Option<u32> {
    parse_u32_str(&text)
}

fn parse_u32_str(text: &str) -> Option<u32> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return None;
    }

    let mut normalized = trimmed.to_ascii_lowercase();
    if let Some(hex) = normalized.strip_prefix("#x") {
        normalized = format!("0x{hex}");
    }

    if normalized.ends_with('h') {
        let body = &normalized[..normalized.len() - 1];
        if !body.is_empty() && body.chars().all(|ch| ch.is_ascii_hexdigit()) {
            normalized = format!("0x{body}");
        }
    }

    if let Some(hex) = normalized.strip_prefix("0x") {
        u32::from_str_radix(hex, 16).ok()
    } else {
        normalized.parse::<u32>().ok()
    }
}

fn parse_u16(text: String) -> Option<u16> {
    parse_u32(text).and_then(|value| u16::try_from(value).ok())
}

fn parse_u8(text: &str) -> Option<u8> {
    parse_u32_str(text).and_then(|value| u8::try_from(value).ok())
}

fn build_datatype_bits_table(device_elem: Node<'_, '_>) -> HashMap<String, u16> {
    let mut table = STD_DATATYPE_BITS
        .iter()
        .map(|(name, bits)| (String::from(*name), *bits))
        .collect::<HashMap<_, _>>();

    for datatype in descendants_by_localname(device_elem, "DataType") {
        let Some(name) = findtext_first_child(datatype, &["Name"]) else {
            continue;
        };
        let Some(bits) = findtext_first_child(datatype, &["BitSize", "BitLen", "BitLength"])
            .and_then(parse_u32)
            .and_then(|value| u16::try_from(value).ok())
        else {
            continue;
        };

        if bits > 0 {
            table.insert(name.trim().to_string(), bits);
        }
    }

    table
}

fn parse_entry_triplet(
    entry: Node<'_, '_>,
    datatype_bits: &HashMap<String, u16>,
) -> Option<PdoEntry> {
    let idx = findtext_first_child(entry, &["Index"]).and_then(parse_u16)?;
    if idx == 0 {
        return None;
    }

    let sub = findtext_first_child(entry, &["SubIndex", "Subindex", "SubIdx"])
        .and_then(parse_u16)
        .and_then(|value| u8::try_from(value).ok())
        .unwrap_or(0);

    let bits = findtext_first_child(entry, &["BitLen", "BitLength", "BitSize"])
        .and_then(parse_u32)
        .and_then(|value| u16::try_from(value).ok())
        .filter(|bits| *bits > 0)
        .or_else(|| {
            let datatype_name = findtext_first_child(entry, &["DataType", "Datatype", "Type"])?;
            datatype_bits
                .get(datatype_name.trim())
                .copied()
                .filter(|bits| *bits > 0)
        })?;

    Some((idx, sub, bits))
}

fn device_identity(
    device_elem: Node<'_, '_>,
) -> (Option<u32>, Option<u32>, Option<u32>, Option<String>, bool) {
    let type_elem = descendants_by_localname(device_elem, "Type")
        .into_iter()
        .next();

    let mut product_code = type_elem
        .and_then(|node| node.attribute("ProductCode"))
        .and_then(parse_u32_str);
    let mut revision_no = type_elem
        .and_then(|node| node.attribute("RevisionNo"))
        .and_then(parse_u32_str);

    if product_code.is_none() {
        product_code = device_elem.attribute("ProductCode").and_then(parse_u32_str);
    }
    if revision_no.is_none() {
        revision_no = device_elem.attribute("RevisionNo").and_then(parse_u32_str);
    }

    let device_name = find_first_child_by_localname(device_elem, "Name")
        .and_then(|name| name.text())
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .map(str::to_string);

    let invisible_attr = device_elem
        .attribute("Invisible")
        .unwrap_or_default()
        .trim()
        .to_ascii_lowercase();
    let visible = !matches!(invisible_attr.as_str(), "1" | "true" | "yes");

    (None, product_code, revision_no, device_name, visible)
}

fn select_device<'a, 'input>(
    root: Node<'a, 'input>,
    product_code: Option<u32>,
    revision_no: Option<u32>,
) -> Option<Node<'a, 'input>> {
    let devices = descendants_by_localname(root, "Device");
    if devices.is_empty() {
        return None;
    }
    if devices.len() == 1 {
        return devices.into_iter().next();
    }

    let mut best = None;
    let mut best_score = i32::MIN;

    for device in devices {
        let (_, device_product, device_revision, _, visible) = device_identity(device);
        let mut score = 0;

        if product_code.is_some() && device_product == product_code {
            score += 100;
        }
        if revision_no.is_some() && device_revision == revision_no {
            score += 50;
        }
        if visible {
            score += 10;
        }

        let physics = device
            .attribute("Physics")
            .unwrap_or_default()
            .trim()
            .to_ascii_uppercase();
        if physics == "YY" {
            score += 5;
        }
        if product_code.is_none() && device_product.is_some_and(|value| value != 0) {
            score += 3;
        }

        if score > best_score {
            best_score = score;
            best = Some(device);
        }
    }

    best
}

fn dedup_preserve_order(values: &mut Vec<u16>) {
    let mut seen = HashSet::new();
    values.retain(|value| seen.insert(*value));
}

#[derive(Clone, Copy)]
enum Direction {
    Rx,
    Tx,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_common_integer_encodings() {
        assert_eq!(parse_u32_str("123"), Some(123));
        assert_eq!(parse_u32_str("0x1A00"), Some(0x1A00));
        assert_eq!(parse_u32_str("#x1A00"), Some(0x1A00));
        assert_eq!(parse_u32_str("1A00h"), Some(0x1A00));
    }

    #[test]
    fn decodes_rx_tx_layout_and_support_flags() {
        let xml = r##"
            <EtherCATInfo xmlns="urn:example">
              <Vendor><Id>#x0000009a</Id></Vendor>
              <Descriptions>
                <Devices>
                  <Device Physics="YY">
                    <Type ProductCode="#x1234" RevisionNo="#x0002" />
                    <Name>Visible Device</Name>
                    <RxPdo Sm="2">
                      <Index>#x1600</Index>
                      <Entry>
                        <Index>#x6040</Index>
                        <SubIndex>0</SubIndex>
                        <BitLen>16</BitLen>
                      </Entry>
                      <Entry>
                        <Index>#x607A</Index>
                        <SubIndex>0</SubIndex>
                        <DataType>DINT</DataType>
                      </Entry>
                    </RxPdo>
                    <TxPdo Sm="3">
                      <Index>#x1A00</Index>
                      <Entry>
                        <Index>#x6041</Index>
                        <SubIndex>0</SubIndex>
                        <BitLen>16</BitLen>
                      </Entry>
                    </TxPdo>
                  </Device>
                </Devices>
              </Descriptions>
            </EtherCATInfo>
        "##;

        let decoded = decode_esi_str(
            xml,
            DecodeEsiOptions {
                product_code: Some(0x1234),
                revision_no: Some(0x0002),
                ..DecodeEsiOptions::default()
            },
        )
        .unwrap();

        assert_eq!(decoded.vendor_id, Some(0x9a));
        assert_eq!(decoded.product_code, Some(0x1234));
        assert_eq!(decoded.revision_no, Some(0x0002));
        assert_eq!(decoded.rx_pdos, vec![0x1600]);
        assert_eq!(decoded.tx_pdos, vec![0x1A00]);
        assert_eq!(decoded.pdo_sm_rx.get(&0x1600), Some(&2));
        assert_eq!(decoded.pdo_sm_tx.get(&0x1A00), Some(&3));
        assert_eq!(decoded.supports.get("controlword"), Some(&true));
        assert_eq!(decoded.supports.get("statusword"), Some(&true));
        assert_eq!(decoded.supports.get("modes_csp"), Some(&true));
    }

    #[test]
    fn decodes_sm_fallback_layout() {
        let xml = r#"
            <EtherCATInfo>
              <Descriptions>
                <Devices>
                  <Device ProductCode="0x1234" RevisionNo="0x1">
                    <Name>Fallback Device</Name>
                    <Sm Index="2" Dir="Output">
                      <Pdo>
                        <Index>0x1600</Index>
                        <Entry>
                          <Index>0x6040</Index>
                          <SubIndex>0</SubIndex>
                          <BitLength>16</BitLength>
                        </Entry>
                      </Pdo>
                    </Sm>
                    <Sm Index="3" Dir="Input">
                      <Pdo>
                        <Index>0x1A00</Index>
                        <Entry>
                          <Index>0x6041</Index>
                          <SubIndex>0</SubIndex>
                          <BitLength>16</BitLength>
                        </Entry>
                      </Pdo>
                    </Sm>
                  </Device>
                </Devices>
              </Descriptions>
            </EtherCATInfo>
        "#;

        let decoded = decode_esi_str(xml, DecodeEsiOptions::default()).unwrap();
        assert_eq!(decoded.rx_pdos, vec![0x1600]);
        assert_eq!(decoded.tx_pdos, vec![0x1A00]);
        assert_eq!(decoded.pdo_sm_rx.get(&0x1600), Some(&2));
        assert_eq!(decoded.pdo_sm_tx.get(&0x1A00), Some(&3));
    }
}
