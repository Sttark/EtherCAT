"""
ESI (EtherCAT Slave Information) XML decoder.

This module intentionally has **no side effects** beyond creating a logger.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple
import xml.etree.ElementTree as ET


logger = logging.getLogger(__name__)


def _localname(tag: str) -> str:
    """Strip '{namespace}' from ElementTree tags."""
    if not tag:
        return ""
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def _iter_descendants_by_localname(elem: ET.Element, local: str) -> Iterable[ET.Element]:
    """Namespace-agnostic descendant search by localname."""
    for e in elem.iter():
        if _localname(e.tag) == local:
            yield e


def _find_first_child_by_localname(elem: ET.Element, local: str) -> Optional[ET.Element]:
    for c in list(elem):
        if _localname(c.tag) == local:
            return c
    return None


def _findtext_first_child(elem: ET.Element, *candidate_locals: str) -> Optional[str]:
    for local in candidate_locals:
        c = _find_first_child_by_localname(elem, local)
        if c is not None and c.text is not None:
            t = c.text.strip()
            if t:
                return t
    return None


def _parse_int(text: Optional[str]) -> Optional[int]:
    """
    Parse common integer encodings found in ESI files.

    Supports:
    - decimal: "123"
    - hex: "0x1A00", "#x1A00", "1A00h"
    """
    if text is None:
        return None
    t = str(text).strip()
    if not t:
        return None

    tl = t.lower()
    if tl.startswith("#x"):
        tl = "0x" + tl[2:]
    if tl.endswith("h") and tl[:-1] and all(ch in "0123456789abcdef" for ch in tl[:-1]):
        tl = "0x" + tl[:-1]

    try:
        return int(tl, 0)
    except ValueError:
        return None


_STD_DATATYPE_BITS: Dict[str, int] = {
    "BOOL": 1,
    "BIT": 1,
    "SINT": 8,
    "USINT": 8,
    "BYTE": 8,
    "INT": 16,
    "UINT": 16,
    "WORD": 16,
    "DINT": 32,
    "UDINT": 32,
    "DWORD": 32,
    "LINT": 64,
    "ULINT": 64,
    "LWORD": 64,
    "REAL": 32,
    "LREAL": 64,
}


@dataclass(frozen=True)
class DecodedEsi:
    """
    Canonical, driver-consumable representation of ESI PDO mapping.
    """

    vendor_id: Optional[int]
    product_code: Optional[int]
    revision_no: Optional[int]
    device_name: Optional[str]
    rx_pdos: List[int]
    tx_pdos: List[int]
    pdo_map_rx: Dict[int, List[Tuple[int, int, int]]]
    pdo_map_tx: Dict[int, List[Tuple[int, int, int]]]
    pdo_sm_rx: Dict[int, int]
    pdo_sm_tx: Dict[int, int]
    supports: Dict[str, bool]


def _dedup_preserve_order(values: List[int]) -> List[int]:
    seen: set[int] = set()
    out: List[int] = []
    for v in values:
        if v in seen:
            continue
        seen.add(v)
        out.append(v)
    return out


def _build_datatype_bits_table(device_elem: ET.Element) -> Dict[str, int]:
    table = dict(_STD_DATATYPE_BITS)
    for dt in _iter_descendants_by_localname(device_elem, "DataType"):
        name = _findtext_first_child(dt, "Name")
        bits_text = _findtext_first_child(dt, "BitSize", "BitLen", "BitLength")
        if not name or not bits_text:
            continue
        bits = _parse_int(bits_text)
        if bits is None or bits <= 0:
            continue
        table[name.strip()] = int(bits)
    return table


def _parse_entry_triplet(entry: ET.Element, datatype_bits: Dict[str, int]) -> Optional[Tuple[int, int, int]]:
    idx = _parse_int(_findtext_first_child(entry, "Index"))
    if idx is None or idx == 0:
        return None  # padding or invalid

    sub = _parse_int(_findtext_first_child(entry, "SubIndex", "Subindex", "SubIdx"))
    if sub is None:
        sub = 0

    bits = _parse_int(_findtext_first_child(entry, "BitLen", "BitLength", "BitSize"))
    if bits is None or bits <= 0:
        dt_name = _findtext_first_child(entry, "DataType", "Datatype", "Type")
        if dt_name:
            bits = datatype_bits.get(dt_name.strip())

    if bits is None or bits <= 0:
        return None

    return (int(idx), int(sub), int(bits))


def _device_identity(device_elem: ET.Element) -> Tuple[Optional[int], Optional[int], Optional[int], Optional[str], bool]:
    type_elem = next(_iter_descendants_by_localname(device_elem, "Type"), None)
    product_code = None
    revision_no = None
    if type_elem is not None:
        product_code = _parse_int(type_elem.get("ProductCode"))
        revision_no = _parse_int(type_elem.get("RevisionNo"))
    if product_code is None:
        product_code = _parse_int(device_elem.get("ProductCode"))
    if revision_no is None:
        revision_no = _parse_int(device_elem.get("RevisionNo"))

    name_elem = _find_first_child_by_localname(device_elem, "Name")
    dev_name = name_elem.text.strip() if name_elem is not None and name_elem.text else None

    invisible_attr = (device_elem.get("Invisible") or "").strip().lower()
    visible = not (invisible_attr in {"1", "true", "yes"})
    return (None, product_code, revision_no, dev_name, visible)


def _select_device(root: ET.Element, product_code: Optional[int], revision_no: Optional[int]) -> Optional[ET.Element]:
    devices = list(_iter_descendants_by_localname(root, "Device"))
    if not devices:
        return None
    if len(devices) == 1:
        return devices[0]

    best: Optional[ET.Element] = None
    best_score = -1
    for dev in devices:
        _v, pc, rev, _name, visible = _device_identity(dev)
        score = 0
        if product_code is not None and pc == product_code:
            score += 100
        if revision_no is not None and rev == revision_no:
            score += 50
        if visible:
            score += 10
        physics = (dev.get("Physics") or "").strip().upper()
        if physics == "YY":
            score += 5
        if product_code is None and pc not in (None, 0):
            score += 3

        if score > best_score:
            best_score = score
            best = dev

    return best or devices[0]


def decode_esi(
    xml_file: str,
    *,
    vendor_id: Optional[int] = None,
    product_code: Optional[int] = None,
    revision_no: Optional[int] = None,
) -> DecodedEsi:
    """
    Robust ESI decoder that supports common EtherCAT ESI layouts:
    - <Device> ... <RxPdo>/<TxPdo> ... (ETG-style, common)
    - <Sm> ... <Pdo> ... (alternate vendor layouts)

    Namespace-agnostic (default namespaces supported) and supports #x / 0x hex.
    If multiple <Device> blocks exist, selection uses product_code/revision_no when provided,
    otherwise prefers visible, non-zero product-code devices.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    doc_vendor_id: Optional[int] = None
    for vend in _iter_descendants_by_localname(root, "Vendor"):
        vid_text = _findtext_first_child(vend, "Id")
        doc_vendor_id = _parse_int(vid_text)
        if doc_vendor_id is not None:
            break

    chosen = _select_device(root, product_code, revision_no)
    if chosen is None:
        logger.warning("No <Device> found in ESI XML; returning empty PDO config")
        return DecodedEsi(
            vendor_id=vendor_id if vendor_id is not None else doc_vendor_id,
            product_code=product_code,
            revision_no=revision_no,
            device_name=None,
            rx_pdos=[],
            tx_pdos=[],
            pdo_map_rx={},
            pdo_map_tx={},
            pdo_sm_rx={},
            pdo_sm_tx={},
            supports={},
        )

    _v, chosen_product, chosen_revision, device_name, _visible = _device_identity(chosen)
    effective_vendor = vendor_id if vendor_id is not None else doc_vendor_id
    effective_product = product_code if product_code is not None else chosen_product
    effective_revision = revision_no if revision_no is not None else chosen_revision

    datatype_bits = _build_datatype_bits_table(chosen)

    rx_pdos: List[int] = []
    tx_pdos: List[int] = []
    pdo_map_rx: Dict[int, List[Tuple[int, int, int]]] = {}
    pdo_map_tx: Dict[int, List[Tuple[int, int, int]]] = {}
    pdo_sm_rx: Dict[int, int] = {}
    pdo_sm_tx: Dict[int, int] = {}

    rx_elems = list(_iter_descendants_by_localname(chosen, "RxPdo"))
    tx_elems = list(_iter_descendants_by_localname(chosen, "TxPdo"))

    def parse_rxtx_pdo(pdo_elem: ET.Element, direction: str) -> None:
        pdo_index = _parse_int(_findtext_first_child(pdo_elem, "Index"))
        if pdo_index is None:
            return

        sm = _parse_int(pdo_elem.get("Sm"))
        if sm is None:
            sm = 2 if direction == "rx" else 3

        entries: List[Tuple[int, int, int]] = []
        for entry in _iter_descendants_by_localname(pdo_elem, "Entry"):
            triplet = _parse_entry_triplet(entry, datatype_bits)
            if triplet is None:
                continue
            entries.append(triplet)

        if direction == "rx":
            rx_pdos.append(int(pdo_index))
            pdo_map_rx.setdefault(int(pdo_index), []).extend(entries)
            pdo_sm_rx[int(pdo_index)] = int(sm)
        else:
            tx_pdos.append(int(pdo_index))
            pdo_map_tx.setdefault(int(pdo_index), []).extend(entries)
            pdo_sm_tx[int(pdo_index)] = int(sm)

    if rx_elems or tx_elems:
        for p in rx_elems:
            parse_rxtx_pdo(p, "rx")
        for p in tx_elems:
            parse_rxtx_pdo(p, "tx")
    else:
        # Fallback layout: <Sm> contains <Pdo>
        for sm_elem in _iter_descendants_by_localname(chosen, "Sm"):
            sm_idx = _parse_int(sm_elem.get("Index")) or _parse_int(_findtext_first_child(sm_elem, "Index"))
            dir_text = (_findtext_first_child(sm_elem, "Dir") or (sm_elem.get("Dir") or "")).strip().lower()
            sm_name = (sm_elem.text or "").strip().lower()

            def is_rx_sm() -> bool:
                if dir_text in {"out", "output"}:
                    return True
                if sm_name in {"outputs", "output"}:
                    return True
                return sm_idx == 2

            def is_tx_sm() -> bool:
                if dir_text in {"in", "input"}:
                    return True
                if sm_name in {"inputs", "input"}:
                    return True
                return sm_idx == 3

            for pdo in _iter_descendants_by_localname(sm_elem, "Pdo"):
                pdo_index = _parse_int(_findtext_first_child(pdo, "Index"))
                if pdo_index is None:
                    continue

                entries: List[Tuple[int, int, int]] = []
                for entry in _iter_descendants_by_localname(pdo, "Entry"):
                    triplet = _parse_entry_triplet(entry, datatype_bits)
                    if triplet is None:
                        continue
                    entries.append(triplet)

                if is_rx_sm():
                    rx_pdos.append(int(pdo_index))
                    pdo_map_rx.setdefault(int(pdo_index), []).extend(entries)
                    if sm_idx is not None:
                        pdo_sm_rx[int(pdo_index)] = int(sm_idx)
                if is_tx_sm():
                    tx_pdos.append(int(pdo_index))
                    pdo_map_tx.setdefault(int(pdo_index), []).extend(entries)
                    if sm_idx is not None:
                        pdo_sm_tx[int(pdo_index)] = int(sm_idx)

    rx_pdos = _dedup_preserve_order(rx_pdos)
    tx_pdos = _dedup_preserve_order(tx_pdos)

    flat_rx = [e for entries in pdo_map_rx.values() for e in entries]
    flat_tx = [e for entries in pdo_map_tx.values() for e in entries]
    supports = {
        "modes_pp": any(e[0] == 0x607A for e in flat_rx),
        "modes_pv": any(e[0] == 0x60FF for e in flat_rx),
        "modes_csp": any(e[0] == 0x607A for e in flat_rx),
        # Touch probe objects vary slightly by device; some use 0x60BB instead of 0x60BC for Probe-2.
        "touch_probe": any(e[0] in (0x60B8, 0x60B9, 0x60BA, 0x60BB, 0x60BC) for e in flat_rx + flat_tx),
        "statusword": any(e[0] == 0x6041 for e in flat_tx),
        "controlword": any(e[0] == 0x6040 for e in flat_rx),
        "mode_display": any(e[0] == 0x6061 for e in flat_tx),
        "mode_command": any(e[0] == 0x6060 for e in flat_rx),
    }

    if not rx_pdos and not tx_pdos:
        logger.warning(
            "Decoded ESI contains no PDOs after parsing. "
            "XML may be non-ESI or use an unsupported structure."
        )

    return DecodedEsi(
        vendor_id=effective_vendor,
        product_code=effective_product,
        revision_no=effective_revision,
        device_name=device_name,
        rx_pdos=rx_pdos,
        tx_pdos=tx_pdos,
        pdo_map_rx=pdo_map_rx,
        pdo_map_tx=pdo_map_tx,
        pdo_sm_rx=pdo_sm_rx,
        pdo_sm_tx=pdo_sm_tx,
        supports=supports,
    )


def parse_esi_features(xml_file: str) -> Dict:
    """
    Backward-compatible wrapper for v2 call sites.
    Prefer `decode_esi()` for device-scoped decoding and SM visibility.
    """
    decoded = decode_esi(xml_file)
    pdo_entries_rx = [e for entries in decoded.pdo_map_rx.values() for e in entries]
    pdo_entries_tx = [e for entries in decoded.pdo_map_tx.values() for e in entries]

    return {
        "rx_pdos": decoded.rx_pdos,
        "tx_pdos": decoded.tx_pdos,
        "pdo_entries_rx": pdo_entries_rx,
        "pdo_entries_tx": pdo_entries_tx,
        "pdo_map_rx": decoded.pdo_map_rx,
        "pdo_map_tx": decoded.pdo_map_tx,
        "pdo_sm_rx": decoded.pdo_sm_rx,
        "pdo_sm_tx": decoded.pdo_sm_tx,
        "supports": decoded.supports,
    }


