use std::collections::{BTreeMap, HashMap};
use std::time::Duration;

use parking_lot::Mutex;
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use crate::commands::{Command, CommandKind, ProbeEdge};
use crate::config::{
    DriveConfig, EthercatNetworkConfig, PdoSelection, RuckigConfig as DriverRuckigConfig,
    SyncDirection,
};
use crate::constants::{
    ObjectAddress, CW_INDEX, DIGITAL_INPUTS_INDEX, DIP_IN_STATE_INDEX, ERROR_CODE_INDEX,
    MODES_OP_DISPLAY_INDEX, MODES_OP_INDEX, POSITION_ACTUAL_INDEX, PROBE_POS1_INDEX,
    PROBE_POS2_ALT_INDEX, PROBE_POS2_INDEX, PROBE_STATUS_INDEX, SW_INDEX, TARGET_POSITION_INDEX,
    TARGET_TORQUE_INDEX, TARGET_VELOCITY_INDEX, TORQUE_ACTUAL_INDEX, VELOCITY_ACTUAL_INDEX,
};
use crate::ffi::{
    Direction, DistributedClockConfig, MasterAccess, PdoConfig, PdoEntryAddress,
    PdoEntryConfig as FfiPdoEntryConfig, PdoEntryRegistration as FfiPdoEntryRegistration,
    SlaveIdentity, SlaveLocation, SyncManagerConfig as FfiSyncManagerConfig, WatchdogConfig,
    WatchdogMode,
};
use crate::ruckig::{
    MotionLimitsOverrides, PlannerDescription, PureRustTrajectoryBackend, RuckigConfig,
    RuckigCspPlanner, RuckigStep,
};
use crate::runtime::{
    EthercatProcessManager, RuntimeCommand, RuntimeConfig, RuntimePdoTarget, SdoConfig,
    SlaveRuntimeConfig,
};
use crate::xml::{
    decode_esi_file, parse_esi_features_file, DecodeEsiOptions, DecodedEsi, ParsedEsiFeatures,
};

type DriveFeatures = HashMap<u16, BTreeMap<String, bool>>;
type DriveAliases = HashMap<u16, u16>;

#[pyclass(unsendable)]
pub struct PyProcessManager {
    cfg: EthercatNetworkConfig,
    drive_aliases: DriveAliases,
    drive_features: DriveFeatures,
    inner: Mutex<EthercatProcessManager>,
}

#[pymethods]
impl PyProcessManager {
    #[new]
    fn new(config_json: &str) -> PyResult<Self> {
        let cfg: EthercatNetworkConfig =
            serde_json::from_str(config_json).map_err(to_py_value_error)?;
        let (runtime_cfg, drive_aliases, drive_features) = build_runtime_config(&cfg)?;
        let inner = EthercatProcessManager::new(runtime_cfg).map_err(to_py_runtime_error)?;
        Ok(Self {
            cfg,
            drive_aliases,
            drive_features,
            inner: Mutex::new(inner),
        })
    }

    fn start(&self) -> PyResult<()> {
        self.inner.lock().start().map_err(to_py_runtime_error)
    }

    fn step(&self, py: Python<'_>) -> PyResult<PyObject> {
        let results = self.inner.lock().step().map_err(to_py_runtime_error)?;
        Ok(command_results_to_py(py, &results))
    }

    fn step_with_application_time(
        &self,
        py: Python<'_>,
        application_time_ns: u64,
    ) -> PyResult<PyObject> {
        let results = self
            .inner
            .lock()
            .step_with_application_time(application_time_ns)
            .map_err(to_py_runtime_error)?;
        Ok(command_results_to_py(py, &results))
    }

    fn stop(&self) -> PyResult<()> {
        self.inner.lock().stop().map_err(to_py_runtime_error)
    }

    fn is_alive(&self) -> bool {
        self.inner.lock().is_alive()
    }

    fn exitcode(&self) -> Option<i32> {
        None
    }

    fn send_command(&self, command_json: &str) -> PyResult<bool> {
        let command: Command = serde_json::from_str(command_json).map_err(to_py_value_error)?;
        let Some(runtime_command) = translate_command(&command, &self.drive_aliases)? else {
            return Ok(false);
        };
        self.inner.lock().queue_command(runtime_command);
        Ok(true)
    }

    fn drain_completed_commands(&self, py: Python<'_>) -> PyObject {
        let results = self.inner.lock().drain_completed_commands();
        command_results_to_py(py, &results)
    }

    fn get_latest_status(&self, py: Python<'_>) -> PyResult<PyObject> {
        let manager = self.inner.lock();
        build_status_snapshot(
            py,
            &manager,
            &self.cfg,
            &self.drive_aliases,
            &self.drive_features,
        )
    }

    fn get_status_snapshot(&self, py: Python<'_>) -> PyResult<PyObject> {
        let manager = self.inner.lock();
        build_status_snapshot(
            py,
            &manager,
            &self.cfg,
            &self.drive_aliases,
            &self.drive_features,
        )
    }
}

#[pyclass(unsendable)]
pub struct PyRuckigCspPlanner {
    inner: Mutex<RuckigCspPlanner<PureRustTrajectoryBackend>>,
}

#[pymethods]
impl PyRuckigCspPlanner {
    #[new]
    fn new() -> Self {
        Self {
            inner: Mutex::new(RuckigCspPlanner::new()),
        }
    }

    fn available(&self) -> bool {
        self.inner.lock().available()
    }

    fn stop(&self, slave_pos: i32) {
        self.inner.lock().stop(slave_pos);
    }

    fn is_active(&self, slave_pos: i32) -> bool {
        self.inner.lock().is_active(slave_pos)
    }

    fn is_velocity_mode(&self, slave_pos: i32) -> bool {
        self.inner.lock().is_velocity_mode(slave_pos)
    }

    fn consume_last_error(&self, slave_pos: i32) -> Option<String> {
        self.inner.lock().consume_last_error(slave_pos)
    }

    fn update_target_velocity(&self, slave_pos: i32, target_velocity: f64) -> bool {
        self.inner
            .lock()
            .update_target_velocity(slave_pos, target_velocity)
    }

    fn describe(&self, py: Python<'_>, slave_pos: i32) -> PyObject {
        let description = self.inner.lock().describe(slave_pos);
        planner_description_to_py(py, &description)
    }

    #[pyo3(signature = (slave_pos, actual_position, actual_velocity, target_position, cfg_json, dt_s_fallback, overrides_json=None, actual_acceleration=0.0))]
    fn start_position(
        &self,
        slave_pos: i32,
        actual_position: i64,
        actual_velocity: f64,
        target_position: i64,
        cfg_json: &str,
        dt_s_fallback: f64,
        overrides_json: Option<&str>,
        actual_acceleration: f64,
    ) -> PyResult<()> {
        let cfg = parse_ruckig_config(cfg_json)?;
        let overrides = parse_motion_overrides(overrides_json)?;
        self.inner
            .lock()
            .start_position(
                slave_pos,
                actual_position,
                actual_velocity,
                target_position,
                cfg,
                dt_s_fallback,
                overrides,
                actual_acceleration,
            )
            .map_err(to_py_runtime_error)
    }

    #[pyo3(signature = (slave_pos, actual_position, actual_velocity, target_velocity, cfg_json, dt_s_fallback, overrides_json=None, actual_acceleration=0.0))]
    fn start_velocity(
        &self,
        slave_pos: i32,
        actual_position: i64,
        actual_velocity: f64,
        target_velocity: f64,
        cfg_json: &str,
        dt_s_fallback: f64,
        overrides_json: Option<&str>,
        actual_acceleration: f64,
    ) -> PyResult<()> {
        let cfg = parse_ruckig_config(cfg_json)?;
        let overrides = parse_motion_overrides(overrides_json)?;
        self.inner
            .lock()
            .start_velocity(
                slave_pos,
                actual_position,
                actual_velocity,
                target_velocity,
                cfg,
                dt_s_fallback,
                overrides,
                actual_acceleration,
            )
            .map_err(to_py_runtime_error)
    }

    #[pyo3(signature = (slave_pos, actual_position=None, actual_velocity=None))]
    fn step(
        &self,
        py: Python<'_>,
        slave_pos: i32,
        actual_position: Option<i64>,
        actual_velocity: Option<f64>,
    ) -> PyResult<Option<PyObject>> {
        let step = self
            .inner
            .lock()
            .step(slave_pos, actual_position, actual_velocity);
        Ok(step.map(|s| ruckig_step_to_py(py, &s)))
    }
}

#[pyfunction]
#[pyo3(signature = (xml_file, vendor_id=None, product_code=None, revision_no=None))]
fn decode_esi(
    py: Python<'_>,
    xml_file: &str,
    vendor_id: Option<u32>,
    product_code: Option<u32>,
    revision_no: Option<u32>,
) -> PyResult<PyObject> {
    let decoded = decode_esi_file(
        xml_file,
        DecodeEsiOptions {
            vendor_id,
            product_code,
            revision_no,
        },
    )
    .map_err(to_py_runtime_error)?;
    Ok(decoded_esi_to_py(py, &decoded))
}

#[pyfunction]
fn parse_esi_features(py: Python<'_>, xml_file: &str) -> PyResult<PyObject> {
    let parsed = parse_esi_features_file(xml_file).map_err(to_py_runtime_error)?;
    Ok(parsed_esi_features_to_py(py, &parsed))
}

pub fn register_module(_py: Python<'_>, module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyProcessManager>()?;
    module.add_class::<PyRuckigCspPlanner>()?;
    module.add_function(wrap_pyfunction!(decode_esi, module)?)?;
    module.add_function(wrap_pyfunction!(parse_esi_features, module)?)?;
    Ok(())
}

fn build_runtime_config(
    cfg: &EthercatNetworkConfig,
) -> PyResult<(RuntimeConfig, DriveAliases, DriveFeatures)> {
    let mut drive_aliases = HashMap::new();
    let mut drive_features = HashMap::new();
    let mut slaves = Vec::new();

    for drive in &cfg.slaves {
        let vendor_id = drive.vendor_id.ok_or_else(|| {
            PyValueError::new_err(format!("slave {} missing vendor_id", drive.position))
        })?;
        let product_code = drive.product_code.ok_or_else(|| {
            PyValueError::new_err(format!("slave {} missing product_code", drive.position))
        })?;

        drive_aliases.insert(drive.position, drive.alias);

        let (sync_managers, registrations, features) = build_drive_pdo_shape(drive)?;
        drive_features.insert(drive.position, features);

        let startup_sdos = drive
            .startup_sdos
            .iter()
            .map(|sdo| SdoConfig {
                index: sdo.address.index,
                subindex: sdo.address.subindex,
                data: sdo.data.clone(),
            })
            .collect::<Vec<_>>();

        let watchdog = build_watchdog_config(cfg, drive);
        let distributed_clock = if drive.enable_dc {
            Some(DistributedClockConfig {
                assign_activate: drive.dc_assign_activate.unwrap_or(0x0300) as u16,
                sync0_cycle_ns: drive
                    .dc_sync0_cycle_time_ns
                    .unwrap_or((cfg.cycle_time_ms * 1_000_000.0) as u32),
                sync0_shift_ns: drive.dc_sync0_shift_ns,
                sync1_cycle_ns: drive.dc_sync1_cycle_time_ns.max(0) as u32,
                sync1_shift_ns: drive.dc_sync1_shift_ns,
            })
        } else {
            None
        };

        slaves.push(SlaveRuntimeConfig {
            location: SlaveLocation {
                alias: drive.alias,
                position: drive.position,
            },
            identity: SlaveIdentity {
                vendor_id,
                product_code,
            },
            sync_managers,
            pdo_registrations: registrations,
            startup_sdos,
            distributed_clock,
            watchdog,
            reference_clock: cfg.dc_reference_slave == Some(drive.position),
        });
    }

    let runtime_cfg = RuntimeConfig {
        library_path: None,
        master_index: cfg.master_index,
        access: if cfg.sdo_only {
            MasterAccess::Open
        } else {
            MasterAccess::Request
        },
        cycle_time: duration_from_ms(cfg.cycle_time_ms),
        initial_application_time_ns: None,
        slaves,
    };

    Ok((runtime_cfg, drive_aliases, drive_features))
}

fn build_drive_pdo_shape(
    drive: &DriveConfig,
) -> PyResult<(
    Vec<FfiSyncManagerConfig>,
    Vec<FfiPdoEntryRegistration>,
    BTreeMap<String, bool>,
)> {
    if let Some(xml) = &drive.xml {
        let decoded = decode_esi_file(
            &xml.xml_file,
            DecodeEsiOptions {
                vendor_id: drive.vendor_id,
                product_code: drive.product_code,
                revision_no: None,
            },
        )
        .map_err(to_py_runtime_error)?;

        let rx_pdos = drive
            .pdo
            .as_ref()
            .filter(|pdo| !pdo.rx_pdos.is_empty())
            .map(|pdo| pdo.rx_pdos.clone())
            .unwrap_or_else(|| decoded.rx_pdos.clone());
        let tx_pdos = drive
            .pdo
            .as_ref()
            .filter(|pdo| !pdo.tx_pdos.is_empty())
            .map(|pdo| pdo.tx_pdos.clone())
            .unwrap_or_else(|| decoded.tx_pdos.clone());

        let rx_map = apply_custom_pdo_override(&decoded.pdo_map_rx, &drive.pdo, &rx_pdos, true);
        let tx_map = apply_custom_pdo_override(&decoded.pdo_map_tx, &drive.pdo, &tx_pdos, false);

        let mut sync_managers = Vec::new();
        if !rx_pdos.is_empty() {
            sync_managers.push(FfiSyncManagerConfig {
                index: 2,
                direction: Direction::Output,
                pdos: rx_pdos
                    .iter()
                    .map(|pdo_index| ffi_pdo_config(*pdo_index, rx_map.get(pdo_index)))
                    .collect(),
                watchdog_mode: WatchdogMode::Default,
            });
        }
        if !tx_pdos.is_empty() {
            sync_managers.push(FfiSyncManagerConfig {
                index: 3,
                direction: Direction::Input,
                pdos: tx_pdos
                    .iter()
                    .map(|pdo_index| ffi_pdo_config(*pdo_index, tx_map.get(pdo_index)))
                    .collect(),
                watchdog_mode: WatchdogMode::Default,
            });
        }

        let registrations = sync_managers
            .iter()
            .flat_map(|sync| sync.pdos.iter())
            .flat_map(|pdo| pdo.entries.iter())
            .map(|entry| FfiPdoEntryRegistration {
                slave: SlaveLocation {
                    alias: drive.alias,
                    position: drive.position,
                },
                identity: SlaveIdentity {
                    vendor_id: drive.vendor_id.unwrap_or_default(),
                    product_code: drive.product_code.unwrap_or_default(),
                },
                entry: PdoEntryAddress {
                    index: entry.index,
                    subindex: entry.subindex,
                },
                bit_length: Some(entry.bit_length),
            })
            .collect::<Vec<_>>();

        Ok((sync_managers, registrations, decoded.supports))
    } else if let Some(sync_configs) = &drive.sync_configs {
        let sync_managers = sync_configs
            .iter()
            .map(|sync| FfiSyncManagerConfig {
                index: sync.sync_manager_index,
                direction: match sync.direction {
                    SyncDirection::Output => Direction::Output,
                    SyncDirection::Input => Direction::Input,
                },
                pdos: sync
                    .pdos
                    .iter()
                    .map(|pdo| PdoConfig {
                        index: pdo.pdo_index,
                        entries: pdo
                            .entries
                            .iter()
                            .map(|entry| FfiPdoEntryConfig {
                                index: entry.address.index,
                                subindex: entry.address.subindex,
                                bit_length: entry.bit_length,
                            })
                            .collect(),
                    })
                    .collect(),
                watchdog_mode: WatchdogMode::Default,
            })
            .collect::<Vec<_>>();

        let registrations = if !drive.register_entries.is_empty() {
            drive
                .register_entries
                .iter()
                .map(|entry| FfiPdoEntryRegistration {
                    slave: SlaveLocation {
                        alias: drive.alias,
                        position: drive.position,
                    },
                    identity: SlaveIdentity {
                        vendor_id: drive.vendor_id.unwrap_or_default(),
                        product_code: drive.product_code.unwrap_or_default(),
                    },
                    entry: PdoEntryAddress {
                        index: entry.index,
                        subindex: entry.subindex,
                    },
                    bit_length: None,
                })
                .collect()
        } else {
            sync_managers
                .iter()
                .flat_map(|sync| sync.pdos.iter())
                .flat_map(|pdo| pdo.entries.iter())
                .map(|entry| FfiPdoEntryRegistration {
                    slave: SlaveLocation {
                        alias: drive.alias,
                        position: drive.position,
                    },
                    identity: SlaveIdentity {
                        vendor_id: drive.vendor_id.unwrap_or_default(),
                        product_code: drive.product_code.unwrap_or_default(),
                    },
                    entry: PdoEntryAddress {
                        index: entry.index,
                        subindex: entry.subindex,
                    },
                    bit_length: Some(entry.bit_length),
                })
                .collect()
        };

        Ok((sync_managers, registrations, BTreeMap::new()))
    } else {
        Err(PyValueError::new_err(format!(
            "slave {} requires either xml or sync_configs",
            drive.position
        )))
    }
}

fn apply_custom_pdo_override(
    original: &BTreeMap<u16, Vec<(u16, u8, u16)>>,
    selection: &Option<PdoSelection>,
    chosen_pdos: &[u16],
    rx: bool,
) -> BTreeMap<u16, Vec<(u16, u8, u16)>> {
    let mut map = original.clone();
    let Some(selection) = selection else {
        return map;
    };
    let Some(custom) = selection.custom_pdo_config.as_ref() else {
        return map;
    };

    let replacement = if rx {
        &custom.rx_entries
    } else {
        &custom.tx_entries
    };
    if replacement.is_empty() || chosen_pdos.is_empty() {
        return map;
    }

    map.clear();
    map.insert(
        chosen_pdos[0],
        replacement
            .iter()
            .map(|entry| {
                (
                    entry.address.index,
                    entry.address.subindex,
                    entry.bit_length as u16,
                )
            })
            .collect(),
    );
    map
}

fn ffi_pdo_config(pdo_index: u16, entries: Option<&Vec<(u16, u8, u16)>>) -> PdoConfig {
    PdoConfig {
        index: pdo_index,
        entries: entries
            .cloned()
            .unwrap_or_default()
            .into_iter()
            .map(|(index, subindex, bit_length)| FfiPdoEntryConfig {
                index,
                subindex,
                bit_length: bit_length as u8,
            })
            .collect(),
    }
}

fn build_watchdog_config(
    cfg: &EthercatNetworkConfig,
    drive: &DriveConfig,
) -> Option<WatchdogConfig> {
    let timeout_ms = drive
        .sm_watchdog_timeout_ms
        .or(cfg.sm_watchdog_timeout_ms)?;
    let divider = drive
        .sm_watchdog_divider
        .or(cfg.sm_watchdog_divider)
        .unwrap_or(250_000);
    let timeout_ns = timeout_ms.max(0.0) * 1_000_000.0;
    let interval_ns = (divider as f64) * 40.0;
    let intervals = if interval_ns <= 0.0 {
        0
    } else {
        (timeout_ns / interval_ns).round() as u32
    };
    Some(WatchdogConfig {
        divider: divider.clamp(1, u16::MAX as u32) as u16,
        intervals: intervals.clamp(1, u16::MAX as u32) as u16,
    })
}

fn duration_from_ms(ms: f64) -> Duration {
    Duration::from_secs_f64((ms.max(0.0)) / 1000.0)
}

fn parse_ruckig_config(config_json: &str) -> PyResult<RuckigConfig> {
    let driver_cfg: DriverRuckigConfig =
        serde_json::from_str(config_json).map_err(to_py_value_error)?;
    Ok(RuckigConfig {
        enabled: driver_cfg.enabled,
        dt_ms: driver_cfg.dt_ms,
        max_velocity: driver_cfg.max_velocity,
        max_acceleration: driver_cfg.max_acceleration,
        max_jerk: driver_cfg.max_jerk,
        velocity_lookahead_s: driver_cfg.velocity_lookahead_s,
        hold_last_commanded_position: driver_cfg.hold_last_commanded_position,
    })
}

fn parse_motion_overrides(overrides_json: Option<&str>) -> PyResult<MotionLimitsOverrides> {
    match overrides_json {
        Some(raw) => serde_json::from_str(raw).map_err(to_py_value_error),
        None => Ok(MotionLimitsOverrides::default()),
    }
}

fn translate_command(
    command: &Command,
    aliases: &DriveAliases,
) -> PyResult<Option<RuntimeCommand>> {
    let alias = *aliases.get(&command.target_id).unwrap_or(&0);
    let target = |address: ObjectAddress| RuntimePdoTarget {
        slave: SlaveLocation {
            alias,
            position: command.target_id,
        },
        entry: PdoEntryAddress {
            index: address.index,
            subindex: address.subindex,
        },
    };

    let command = match &command.kind {
        CommandKind::SetPositionMode(_) => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(MODES_OP_INDEX, 0)),
            data: [1_i8 as u8].to_vec(),
        },
        CommandKind::SetVelocityMode(_) => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(MODES_OP_INDEX, 0)),
            data: [3_i8 as u8].to_vec(),
        },
        CommandKind::SetTorqueMode(_) => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(MODES_OP_INDEX, 0)),
            data: [4_i8 as u8].to_vec(),
        },
        CommandKind::SetCspMode(_) => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(MODES_OP_INDEX, 0)),
            data: [8_i8 as u8].to_vec(),
        },
        CommandKind::SetCsvMode(_) => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(MODES_OP_INDEX, 0)),
            data: [9_i8 as u8].to_vec(),
        },
        CommandKind::SetCstMode(_) => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(MODES_OP_INDEX, 0)),
            data: [10_i8 as u8].to_vec(),
        },
        CommandKind::SetPosition(payload) | CommandKind::SetPositionCsp(payload) => {
            RuntimeCommand::WritePdo {
                target: target(ObjectAddress::new(TARGET_POSITION_INDEX, 0)),
                data: (payload.position as i32).to_le_bytes().to_vec(),
            }
        }
        CommandKind::SetVelocity(payload) | CommandKind::SetVelocityCsv(payload) => {
            RuntimeCommand::WritePdo {
                target: target(ObjectAddress::new(TARGET_VELOCITY_INDEX, 0)),
                data: (payload.velocity as i32).to_le_bytes().to_vec(),
            }
        }
        CommandKind::SetTorque(payload) | CommandKind::SetTorqueCst(payload) => {
            RuntimeCommand::WritePdo {
                target: target(ObjectAddress::new(TARGET_TORQUE_INDEX, 0)),
                data: (payload.torque as i16).to_le_bytes().to_vec(),
            }
        }
        CommandKind::WriteRawPdo(payload) => RuntimeCommand::WritePdo {
            target: target(payload.address),
            data: payload.data.clone(),
        },
        CommandKind::NoOp => RuntimeCommand::SetApplicationTime {
            app_time_ns: crate::runtime::monotonic_time_ns(),
        },
        CommandKind::ArmProbe(payload) => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(
                crate::constants::PROBE_FUNCTION_INDEX,
                0,
            )),
            data: probe_edge_to_u16(payload.edge).to_le_bytes().to_vec(),
        },
        CommandKind::DisableProbe => RuntimeCommand::WritePdo {
            target: target(ObjectAddress::new(
                crate::constants::PROBE_FUNCTION_INDEX,
                0,
            )),
            data: 0_u16.to_le_bytes().to_vec(),
        },
        CommandKind::StopMotion(_)
        | CommandKind::StartHoming(_)
        | CommandKind::StartRuckigPosition(_)
        | CommandKind::StartRuckigVelocity(_)
        | CommandKind::StopRuckig
        | CommandKind::EnableDrive
        | CommandKind::DisableDrive
        | CommandKind::ShutdownDrive
        | CommandKind::ClearFault
        | CommandKind::ReadSdo(_)
        | CommandKind::WriteSdo(_) => return Ok(None),
    };

    Ok(Some(command))
}

fn probe_edge_to_u16(edge: ProbeEdge) -> u16 {
    match edge {
        ProbeEdge::Positive => 0x0005,
        ProbeEdge::Negative => 0x0009,
    }
}

fn build_status_snapshot(
    py: Python<'_>,
    manager: &EthercatProcessManager,
    cfg: &EthercatNetworkConfig,
    aliases: &DriveAliases,
    features: &DriveFeatures,
) -> PyResult<PyObject> {
    let dict = PyDict::new_bound(py);
    let status = manager.status();
    dict.set_item(
        "timestamp_ns",
        status
            .last_cycle_completed_at_ns
            .or(status.last_cycle_started_at_ns),
    )?;
    dict.set_item("cycle_time_ms_config", Some(cfg.cycle_time_ms))?;
    dict.set_item(
        "domain_wc",
        status.last_domain_state.as_ref().map(|v| v.working_counter),
    )?;
    dict.set_item(
        "domain_wc_state",
        status.last_domain_state.as_ref().map(|v| v.wc_state as i32),
    )?;
    dict.set_item("sdo_only", cfg.sdo_only)?;
    dict.set_item("overrun_count", 0)?;
    dict.set_item("deadline_miss_count", 0)?;

    let drives = PyDict::new_bound(py);
    for drive in &cfg.slaves {
        let alias = *aliases.get(&drive.position).unwrap_or(&0);
        let drive_dict = PyDict::new_bound(py);
        drive_dict.set_item("in_op", manager.is_alive())?;
        drive_dict.set_item("enabled", false)?;

        let read_opt = |address: ObjectAddress, size: usize| -> Option<Vec<u8>> {
            manager
                .read_pdo_bytes(
                    RuntimePdoTarget {
                        slave: SlaveLocation {
                            alias,
                            position: drive.position,
                        },
                        entry: PdoEntryAddress {
                            index: address.index,
                            subindex: address.subindex,
                        },
                    },
                    size,
                )
                .ok()
        };

        if let Some(bytes) = read_opt(ObjectAddress::new(SW_INDEX, 0), 2) {
            let value = u16::from_le_bytes([bytes[0], bytes[1]]);
            drive_dict.set_item("statusword", value)?;
            drive_dict.set_item("fault", value & 0x0008 != 0)?;
            drive_dict.set_item("warning", value & 0x0080 != 0)?;
            drive_dict.set_item("target_reached", value & 0x0400 != 0)?;
            drive_dict.set_item("setpoint_ack", value & 0x1000 != 0)?;
            drive_dict.set_item("enabled", value & 0x006F == 0x0027)?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(MODES_OP_DISPLAY_INDEX, 0), 1) {
            drive_dict.set_item("mode_display", bytes[0])?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(POSITION_ACTUAL_INDEX, 0), 4) {
            drive_dict.set_item("position_actual", i32::from_le_bytes(bytes4(&bytes)))?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(VELOCITY_ACTUAL_INDEX, 0), 4) {
            drive_dict.set_item("velocity_actual", i32::from_le_bytes(bytes4(&bytes)))?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(TORQUE_ACTUAL_INDEX, 0), 2) {
            drive_dict.set_item("torque_actual", i16::from_le_bytes(bytes2(&bytes)))?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(ERROR_CODE_INDEX, 0), 2) {
            drive_dict.set_item("error_code", u16::from_le_bytes(bytes2(&bytes)))?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(DIGITAL_INPUTS_INDEX, 0), 4) {
            drive_dict.set_item("digital_inputs", u32::from_le_bytes(bytes4(&bytes)))?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(DIP_IN_STATE_INDEX, 1), 4) {
            drive_dict.set_item("dip_in_state", u32::from_le_bytes(bytes4(&bytes)))?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(PROBE_STATUS_INDEX, 0), 2) {
            let value = u16::from_le_bytes(bytes2(&bytes));
            drive_dict.set_item("probe_active", value & 0x0001 != 0)?;
            drive_dict.set_item("probe_enabled", true)?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(PROBE_POS1_INDEX, 0), 4) {
            drive_dict.set_item("probe_pos1", i32::from_le_bytes(bytes4(&bytes)))?;
        }
        if let Some(bytes) = read_opt(ObjectAddress::new(PROBE_POS2_INDEX, 0), 4) {
            drive_dict.set_item("probe_pos2", i32::from_le_bytes(bytes4(&bytes)))?;
        } else if let Some(bytes) = read_opt(ObjectAddress::new(PROBE_POS2_ALT_INDEX, 0), 4) {
            drive_dict.set_item("probe_pos2", i32::from_le_bytes(bytes4(&bytes)))?;
        }

        let feature_dict = PyDict::new_bound(py);
        if let Some(feature_map) = features.get(&drive.position) {
            for (key, value) in feature_map {
                feature_dict.set_item(key, *value)?;
            }
        }
        drive_dict.set_item("features", feature_dict)?;

        let pdo_health = PyDict::new_bound(py);
        for address in [
            ObjectAddress::new(MODES_OP_INDEX, 0),
            ObjectAddress::new(MODES_OP_DISPLAY_INDEX, 0),
            ObjectAddress::new(CW_INDEX, 0),
            ObjectAddress::new(SW_INDEX, 0),
            ObjectAddress::new(TARGET_VELOCITY_INDEX, 0),
            ObjectAddress::new(TARGET_POSITION_INDEX, 0),
        ] {
            let state = if read_opt(address, address_size(address)).is_some() {
                "pdo"
            } else {
                "missing"
            };
            pdo_health.set_item(
                format!("0x{:04X}:{}", address.index, address.subindex),
                state,
            )?;
        }
        drive_dict.set_item("pdo_health", pdo_health)?;

        drives.set_item(drive.position, drive_dict)?;
    }
    dict.set_item("drives", drives)?;
    Ok(dict.into_py(py))
}

fn address_size(address: ObjectAddress) -> usize {
    match address.index {
        MODES_OP_INDEX | MODES_OP_DISPLAY_INDEX => 1,
        TARGET_TORQUE_INDEX | TORQUE_ACTUAL_INDEX | ERROR_CODE_INDEX | PROBE_STATUS_INDEX => 2,
        _ => 4,
    }
}

fn bytes2(bytes: &[u8]) -> [u8; 2] {
    let mut out = [0_u8; 2];
    out.copy_from_slice(&bytes[..2]);
    out
}

fn bytes4(bytes: &[u8]) -> [u8; 4] {
    let mut out = [0_u8; 4];
    out.copy_from_slice(&bytes[..4]);
    out
}

fn command_results_to_py(py: Python<'_>, results: &[crate::runtime::CommandResult]) -> PyObject {
    let list = PyList::empty_bound(py);
    for item in results {
        let dict = PyDict::new_bound(py);
        let _ = dict.set_item("token", item.token);
        match &item.outcome {
            Ok(output) => {
                let _ = dict.set_item("ok", true);
                let _ = dict.set_item("outcome", format!("{output:?}"));
            }
            Err(error) => {
                let _ = dict.set_item("ok", false);
                let _ = dict.set_item("error", error);
            }
        }
        let _ = list.append(dict);
    }
    list.into_py(py)
}

fn decoded_esi_to_py(py: Python<'_>, decoded: &DecodedEsi) -> PyObject {
    let dict = PyDict::new_bound(py);
    let _ = dict.set_item("vendor_id", decoded.vendor_id);
    let _ = dict.set_item("product_code", decoded.product_code);
    let _ = dict.set_item("revision_no", decoded.revision_no);
    let _ = dict.set_item("device_name", decoded.device_name.clone());
    let _ = dict.set_item("rx_pdos", decoded.rx_pdos.clone());
    let _ = dict.set_item("tx_pdos", decoded.tx_pdos.clone());
    let _ = dict.set_item("pdo_map_rx", pdo_map_to_py(py, &decoded.pdo_map_rx));
    let _ = dict.set_item("pdo_map_tx", pdo_map_to_py(py, &decoded.pdo_map_tx));
    let _ = dict.set_item("pdo_sm_rx", mapping_to_py(py, &decoded.pdo_sm_rx));
    let _ = dict.set_item("pdo_sm_tx", mapping_to_py(py, &decoded.pdo_sm_tx));
    let _ = dict.set_item("supports", bool_map_to_py(py, &decoded.supports));
    dict.into_py(py)
}

fn parsed_esi_features_to_py(py: Python<'_>, parsed: &ParsedEsiFeatures) -> PyObject {
    let dict = PyDict::new_bound(py);
    let _ = dict.set_item("rx_pdos", parsed.rx_pdos.clone());
    let _ = dict.set_item("tx_pdos", parsed.tx_pdos.clone());
    let _ = dict.set_item(
        "pdo_entries_rx",
        entry_list_to_py(py, &parsed.pdo_entries_rx),
    );
    let _ = dict.set_item(
        "pdo_entries_tx",
        entry_list_to_py(py, &parsed.pdo_entries_tx),
    );
    let _ = dict.set_item("pdo_map_rx", pdo_map_to_py(py, &parsed.pdo_map_rx));
    let _ = dict.set_item("pdo_map_tx", pdo_map_to_py(py, &parsed.pdo_map_tx));
    let _ = dict.set_item("pdo_sm_rx", mapping_to_py(py, &parsed.pdo_sm_rx));
    let _ = dict.set_item("pdo_sm_tx", mapping_to_py(py, &parsed.pdo_sm_tx));
    let _ = dict.set_item("supports", bool_map_to_py(py, &parsed.supports));
    dict.into_py(py)
}

fn pdo_map_to_py(py: Python<'_>, map: &BTreeMap<u16, Vec<(u16, u8, u16)>>) -> PyObject {
    let dict = PyDict::new_bound(py);
    for (key, entries) in map {
        let _ = dict.set_item(*key, entry_list_to_py(py, entries));
    }
    dict.into_py(py)
}

fn mapping_to_py(py: Python<'_>, map: &BTreeMap<u16, u8>) -> PyObject {
    let dict = PyDict::new_bound(py);
    for (key, value) in map {
        let _ = dict.set_item(*key, *value);
    }
    dict.into_py(py)
}

fn bool_map_to_py(py: Python<'_>, map: &BTreeMap<String, bool>) -> PyObject {
    let dict = PyDict::new_bound(py);
    for (key, value) in map {
        let _ = dict.set_item(key, *value);
    }
    dict.into_py(py)
}

fn entry_list_to_py(py: Python<'_>, entries: &[(u16, u8, u16)]) -> PyObject {
    let list = PyList::empty_bound(py);
    for (index, subindex, bits) in entries {
        let _ = list.append((*index, *subindex, *bits));
    }
    list.into_py(py)
}

fn planner_description_to_py(py: Python<'_>, description: &PlannerDescription) -> PyObject {
    let dict = PyDict::new_bound(py);
    let _ = dict.set_item("active", description.active);
    let _ = dict.set_item("mode", description.mode);
    let _ = dict.set_item("target_position", description.target_position);
    let _ = dict.set_item("target_velocity", description.target_velocity);
    let _ = dict.set_item("error", description.error.clone());
    let _ = dict.set_item("backend", description.backend);
    dict.into_py(py)
}

fn ruckig_step_to_py(py: Python<'_>, step: &RuckigStep) -> PyObject {
    let dict = PyDict::new_bound(py);
    let _ = dict.set_item("position", step.position);
    let _ = dict.set_item("velocity", step.velocity);
    let _ = dict.set_item("acceleration", step.acceleration);
    let _ = dict.set_item("done", step.done);
    dict.into_py(py)
}

fn to_py_runtime_error<E: std::fmt::Display>(error: E) -> PyErr {
    PyRuntimeError::new_err(error.to_string())
}

fn to_py_value_error<E: std::fmt::Display>(error: E) -> PyErr {
    PyValueError::new_err(error.to_string())
}
