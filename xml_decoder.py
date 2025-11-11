import logging
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple


logger = logging.getLogger(__name__)


def _parse_int(text: str) -> int:
    text = text.strip()
    if text.lower().startswith("0x"):
        return int(text, 16)
    return int(text)


def parse_esi_features(xml_file: str) -> Dict:
    """
    Parse an EtherCAT ESI XML file to extract device features and PDO mappings.

    Returns a dictionary with keys:
      - rx_pdos: List[int]
      - tx_pdos: List[int]
      - pdo_entries_rx: List[Tuple[index, subindex, bits]]
      - pdo_entries_tx: List[Tuple[index, subindex, bits]]
      - supports: Dict[str, bool]
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    ns = {
        'xsi': 'http://www.w3.org/2001/XMLSchema-instance'
    }

    rx_pdos: List[int] = []
    tx_pdos: List[int] = []
    pdo_entries_rx: List[Tuple[int, int, int]] = []
    pdo_entries_tx: List[Tuple[int, int, int]] = []
    pdo_map_rx: Dict[int, List[Tuple[int, int, int]]] = {}
    pdo_map_tx: Dict[int, List[Tuple[int, int, int]]] = {}

    # Basic detection of PDOs and entries
    for sm in root.findall('.//Sm', ns) + root.findall('.//Sm'):
        sm_index_text = sm.findtext('Index')
        if not sm_index_text:
            continue
        sm_index = _parse_int(sm_index_text)
        direction = sm.findtext('Dir') or sm.get('Dir') or ''
        for pdo in sm.findall('.//Pdo'):
            pdo_index_text = pdo.findtext('Index')
            if not pdo_index_text:
                continue
            pdo_index = _parse_int(pdo_index_text)
            if direction.lower() in ('out', 'output') or sm_index in (2,):
                rx_pdos.append(pdo_index)
                pdo_map_rx.setdefault(pdo_index, [])
            if direction.lower() in ('in', 'input') or sm_index in (3,):
                tx_pdos.append(pdo_index)
                pdo_map_tx.setdefault(pdo_index, [])
            for entry in pdo.findall('.//Entry'):
                idx_text = entry.findtext('Index')
                sub_text = entry.findtext('SubIndex') or entry.findtext('Subindex')
                bits_text = entry.findtext('BitLen') or entry.findtext('BitLength')
                if not (idx_text and sub_text and bits_text):
                    continue
                idx = _parse_int(idx_text)
                sub = _parse_int(sub_text)
                bits = _parse_int(bits_text)
                if direction.lower() in ('out', 'output') or sm_index in (2,):
                    pdo_entries_rx.append((idx, sub, bits))
                    pdo_map_rx[pdo_index].append((idx, sub, bits))
                if direction.lower() in ('in', 'input') or sm_index in (3,):
                    pdo_entries_tx.append((idx, sub, bits))
                    pdo_map_tx[pdo_index].append((idx, sub, bits))

    # Feature flags (heuristics based on standard indices)
    supports = {
        'modes_pp': any(e[0] == 0x607A for e in pdo_entries_rx),
        'modes_pv': any(e[0] == 0x60FF for e in pdo_entries_rx),
        'modes_csp': any(e[0] == 0x607A for e in pdo_entries_rx),  # CSP uses target position cyclic
        'touch_probe': any(e[0] in (0x60B8, 0x60B9, 0x60BA, 0x60BC) for e in pdo_entries_rx + pdo_entries_tx),
        'statusword': any(e[0] == 0x6041 for e in pdo_entries_tx),
        'controlword': any(e[0] == 0x6040 for e in pdo_entries_rx),
        'mode_display': any(e[0] == 0x6061 for e in pdo_entries_tx),
        'mode_command': any(e[0] == 0x6060 for e in pdo_entries_rx),
    }

    return {
        'rx_pdos': sorted(set(rx_pdos)),
        'tx_pdos': sorted(set(tx_pdos)),
        'pdo_entries_rx': pdo_entries_rx,
        'pdo_entries_tx': pdo_entries_tx,
        'pdo_map_rx': pdo_map_rx,
        'pdo_map_tx': pdo_map_tx,
        'supports': supports,
    }


