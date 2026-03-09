import json
import logging
import time
from dataclasses import dataclass, fields, is_dataclass
from typing import Any, Dict, List, Optional, Tuple

from .cia402.driver import CiA402Drive
from .commands import Command, CommandType
from .config_schema import EthercatNetworkConfig, RuckigConfig
from .status_model import NetworkStatus

try:
    import _ethercat_rs as _rust_ethercat  # type: ignore[import-not-found]
except Exception:
    _rust_ethercat = None


def require_rust_backend():
    if _rust_ethercat is None:
        raise RuntimeError("Rust EtherCAT core is not available")
    return _rust_ethercat


def json_compatible(value):
    if is_dataclass(value):
        return {f.name: json_compatible(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, dict):
        return {json_compatible(k): json_compatible(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_compatible(v) for v in value]
    if isinstance(value, bytes):
        return list(value)
    return value


def network_status_from_payload(payload: Dict) -> NetworkStatus:
    status = NetworkStatus(drives={})
    for key, value in payload.items():
        if key == "drives":
            status.drives = {int(pos): dict(drive) for pos, drive in (value or {}).items()}
        elif hasattr(status, key):
            setattr(status, key, value)
    return status


@dataclass(frozen=True)
class DecodedEsi:
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


class XmlCompatibility:
    @staticmethod
    def decode_esi(
        xml_file: str,
        *,
        vendor_id: Optional[int] = None,
        product_code: Optional[int] = None,
        revision_no: Optional[int] = None,
    ) -> DecodedEsi:
        payload = require_rust_backend().decode_esi(
            xml_file,
            vendor_id=vendor_id,
            product_code=product_code,
            revision_no=revision_no,
        )
        payload = dict(payload)
        return DecodedEsi(
            vendor_id=payload.get("vendor_id"),
            product_code=payload.get("product_code"),
            revision_no=payload.get("revision_no"),
            device_name=payload.get("device_name"),
            rx_pdos=list(payload.get("rx_pdos") or []),
            tx_pdos=list(payload.get("tx_pdos") or []),
            pdo_map_rx=dict(payload.get("pdo_map_rx") or {}),
            pdo_map_tx=dict(payload.get("pdo_map_tx") or {}),
            pdo_sm_rx=dict(payload.get("pdo_sm_rx") or {}),
            pdo_sm_tx=dict(payload.get("pdo_sm_tx") or {}),
            supports=dict(payload.get("supports") or {}),
        )

    @staticmethod
    def parse_esi_features(xml_file: str) -> Dict:
        return dict(require_rust_backend().parse_esi_features(xml_file))


@dataclass
class RuckigStep:
    position: int
    velocity: float
    acceleration: float
    done: bool
    result: Optional[Any] = None


class RuckigUnavailable(RuntimeError):
    pass


class RuckigCompatibility:
    @staticmethod
    def require():
        if _rust_ethercat is None:
            raise RuckigUnavailable("Rust Ruckig planner is not available")
        return _rust_ethercat


class RuckigCspPlanner:
    """Minimal Python wrapper around the Rust-backed CSP planner."""

    def __init__(self):
        self._planner = RuckigCompatibility.require().PyRuckigCspPlanner()

    def available(self) -> bool:
        return bool(self._planner.available())

    def stop(self, slave_pos: int) -> None:
        self._planner.stop(slave_pos)

    def is_active(self, slave_pos: int) -> bool:
        return bool(self._planner.is_active(slave_pos))

    def describe(self, slave_pos: int) -> Dict[str, Any]:
        return dict(self._planner.describe(slave_pos))

    def consume_last_error(self, slave_pos: int) -> Optional[str]:
        return self._planner.consume_last_error(slave_pos)

    def is_velocity_mode(self, slave_pos: int) -> bool:
        return bool(self._planner.is_velocity_mode(slave_pos))

    def update_target_velocity(self, slave_pos: int, target_velocity: float) -> bool:
        return bool(self._planner.update_target_velocity(slave_pos, target_velocity))

    def start_position(
        self,
        slave_pos: int,
        *,
        actual_position: int,
        actual_velocity: float,
        target_position: int,
        cfg: RuckigConfig,
        dt_s_fallback: float,
        overrides: Optional[Dict[str, Optional[float]]] = None,
        actual_acceleration: float = 0.0,
    ) -> None:
        self._planner.start_position(
            slave_pos,
            actual_position,
            actual_velocity,
            target_position,
            json.dumps(cfg.__dict__),
            dt_s_fallback,
            json.dumps(overrides or {}),
            actual_acceleration,
        )

    def start_velocity(
        self,
        slave_pos: int,
        *,
        actual_position: int,
        actual_velocity: float,
        target_velocity: float,
        cfg: RuckigConfig,
        dt_s_fallback: float,
        overrides: Optional[Dict[str, Optional[float]]] = None,
        actual_acceleration: float = 0.0,
    ) -> None:
        self._planner.start_velocity(
            slave_pos,
            actual_position,
            actual_velocity,
            target_velocity,
            json.dumps(cfg.__dict__),
            dt_s_fallback,
            json.dumps(overrides or {}),
            actual_acceleration,
        )

    def step(
        self,
        slave_pos: int,
        *,
        actual_position: Optional[int],
        actual_velocity: Optional[float],
    ) -> Optional[RuckigStep]:
        payload = self._planner.step(slave_pos, actual_position, actual_velocity)
        if payload is None:
            return None
        result = dict(payload)
        return RuckigStep(
            position=int(result["position"]),
            velocity=float(result["velocity"]),
            acceleration=float(result["acceleration"]),
            done=bool(result["done"]),
            result=None,
        )


class EtherCATProcessManager:
    """Minimal Python wrapper around the Rust EtherCAT process manager."""

    def __init__(self, cfg: EthercatNetworkConfig):
        rust = require_rust_backend()
        self.cfg = cfg
        self._inner = rust.PyProcessManager(json.dumps(json_compatible(cfg)))
        self._latest_status: Optional[NetworkStatus] = None

    def start(self):
        self._inner.start()

    def is_alive(self) -> bool:
        return bool(self._inner.is_alive())

    def exitcode(self) -> Optional[int]:
        return self._inner.exitcode()

    def stop(self):
        self._inner.stop()

    def send_command(self, cmd: Command):
        payload = {
            "target_id": int(cmd.target_id),
            "type": getattr(cmd.type, "name", str(cmd.type)),
            "payload": json_compatible(cmd.params or {}),
            "correlation_id": cmd.correlation_id,
        }
        if cmd.value is not None:
            if payload["type"] in ("SET_POSITION", "SET_POSITION_CSP"):
                payload["payload"]["position"] = cmd.value
            elif payload["type"] in ("SET_VELOCITY", "SET_VELOCITY_CSV"):
                payload["payload"]["velocity"] = cmd.value
            elif payload["type"] in ("SET_TORQUE", "SET_TORQUE_CST"):
                payload["payload"]["torque"] = cmd.value
            elif payload["type"] == "START_RUCKIG_POSITION":
                payload["payload"]["target_position"] = cmd.value
            elif payload["type"] == "START_RUCKIG_VELOCITY":
                payload["payload"]["target_velocity"] = cmd.value
            elif payload["type"] in ("WRITE_RAW_PDO", "WRITE_SDO"):
                payload["payload"]["data"] = json_compatible(cmd.value)
        return bool(self._inner.send_command(json.dumps(payload)))

    def get_latest_status(self) -> Optional[NetworkStatus]:
        payload = self._inner.get_latest_status()
        if payload is None:
            return None
        self._latest_status = network_status_from_payload(dict(payload))
        return self._latest_status

    def get_status_snapshot(self) -> Optional[NetworkStatus]:
        payload = self._inner.get_status_snapshot()
        if payload is None:
            return self._latest_status
        self._latest_status = network_status_from_payload(dict(payload))
        return self._latest_status


class StatusProxy:
    def __init__(self, manager: EtherCATProcessManager):
        self._manager = manager
        self._last: Optional[NetworkStatus] = None
        self._last_at: float = 0.0

    def _refresh(self):
        now = time.time()
        refresh_period_s = float(getattr(self._manager.cfg, "status_proxy_refresh_s", 0.02))
        if refresh_period_s < 0:
            refresh_period_s = 0.0
        if now - self._last_at > refresh_period_s:
            latest = self._manager.get_status_snapshot()
            if latest is None:
                latest = self._manager.get_latest_status()
            if latest is not None:
                self._last = latest
                self._last_at = now

    def get_field(self, slave_position: int, key: str):
        self._refresh()
        if not self._last:
            return None
        drive = self._last.drives.get(slave_position) or {}
        return drive.get(key)


def attach_drive_handle(
    manager: EtherCATProcessManager,
    slave_position: int,
    status_proxy: Optional[StatusProxy] = None,
) -> CiA402Drive:
    drive = CiA402Drive(slave_position=slave_position)
    status = status_proxy or StatusProxy(manager)

    def enqueue(pos: int, cmd_type: CommandType, value, params: Dict):
        manager.send_command(Command(target_id=pos, type=cmd_type, value=value, params=params))

    def read_status(pos: int, key: str):
        return status.get_field(pos, key)

    drive._enqueue_command = enqueue  # type: ignore[attr-defined]
    drive._read_status = read_status  # type: ignore[attr-defined]

    def get_features():
        latest = manager.get_status_snapshot()
        if latest is None:
            latest = manager.get_latest_status()
        if latest and latest.drives.get(slave_position):
            return latest.drives[slave_position].get("features", {})
        return {}

    drive.get_features = get_features  # type: ignore[attr-defined]
    return drive


class EtherCATBus:
    def __init__(self, config: EthercatNetworkConfig):
        self._config = config
        self._manager: Optional[EtherCATProcessManager] = None
        self._drives: Dict[int, CiA402Drive] = {}
        self._status: Optional[StatusProxy] = None
        self._logger = logging.getLogger("EtherCATBus")

    def start(self):
        self._manager = EtherCATProcessManager(self._config)
        self._manager.start()
        self._status = StatusProxy(self._manager)
        for slave_cfg in self._config.slaves:
            if slave_cfg.cia402:
                drive = attach_drive_handle(
                    self._manager,
                    slave_cfg.position,
                    status_proxy=self._status,
                )
                self._drives[slave_cfg.position] = drive
                self._logger.info(f"Attached drive at position {slave_cfg.position}")

    def get_drive(self, position: int) -> Optional[CiA402Drive]:
        return self._drives.get(position)

    def read_field(self, slave_position: int, key: str):
        if self._status:
            return self._status.get_field(slave_position, key)
        return None

    def send_command(self, command: Command):
        if self._manager:
            self._manager.send_command(command)

    def shutdown(self):
        if self._manager:
            self._logger.info("Shutting down EtherCAT bus")
            self._manager.stop()
            self._manager = None
            self._drives.clear()
            self._status = None

    @property
    def is_running(self) -> bool:
        return self._manager is not None and self._manager.is_alive()


RustEtherCATProcessManager = EtherCATProcessManager
RustEtherCATBus = EtherCATBus
