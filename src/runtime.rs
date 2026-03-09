use std::collections::{BTreeMap, VecDeque};
use std::time::Duration;

use thiserror::Error;

use crate::ffi::{
    DistributedClockConfig, Domain, DomainState, EthercatFfiError, EthercatLibrary, Master,
    MasterAccess, MasterInfo, PdoEntryAddress, PdoEntryOffset, PdoEntryRegistration, SlaveConfig,
    SlaveIdentity, SlaveLocation, SyncManagerConfig, WatchdogConfig,
};

pub type Result<T> = std::result::Result<T, RuntimeError>;
pub type CommandToken = u64;

#[derive(Debug, Error)]
pub enum RuntimeError {
    #[error(transparent)]
    Ffi(#[from] EthercatFfiError),
    #[error("runtime is not configured")]
    NotConfigured,
    #[error("runtime is not active")]
    NotActive,
    #[error("runtime is already configured")]
    AlreadyConfigured,
    #[error("runtime is already active")]
    AlreadyActive,
    #[error("runtime has no PDO domain configured")]
    NoDomain,
    #[error("no registered PDO entry for slave alias={alias} position={position} entry=0x{index:04x}:{subindex}")]
    UnknownPdoEntry {
        alias: u16,
        position: u16,
        index: u16,
        subindex: u8,
    },
}

#[derive(Debug, Clone)]
pub struct RuntimeConfig {
    pub library_path: Option<std::path::PathBuf>,
    pub master_index: u32,
    pub access: MasterAccess,
    pub cycle_time: Duration,
    pub initial_application_time_ns: Option<u64>,
    pub slaves: Vec<SlaveRuntimeConfig>,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            library_path: None,
            master_index: 0,
            access: MasterAccess::Request,
            cycle_time: Duration::from_millis(1),
            initial_application_time_ns: None,
            slaves: Vec::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct SlaveRuntimeConfig {
    pub location: SlaveLocation,
    pub identity: SlaveIdentity,
    pub sync_managers: Vec<SyncManagerConfig>,
    pub pdo_registrations: Vec<PdoEntryRegistration>,
    pub startup_sdos: Vec<SdoConfig>,
    pub distributed_clock: Option<DistributedClockConfig>,
    pub watchdog: Option<WatchdogConfig>,
    pub reference_clock: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SdoConfig {
    pub index: u16,
    pub subindex: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RuntimePdoTarget {
    pub slave: SlaveLocation,
    pub entry: PdoEntryAddress,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RuntimeState {
    Created,
    Configured,
    Activated,
    Running,
    Stopping,
    Stopped,
    Faulted,
}

#[derive(Debug, Clone)]
pub struct RuntimeStatus {
    pub state: RuntimeState,
    pub library_path: Option<std::path::PathBuf>,
    pub cycle_time: Duration,
    pub configured_slave_count: usize,
    pub registered_pdo_entries: usize,
    pub pending_command_count: usize,
    pub completed_command_count: usize,
    pub cycle_count: u64,
    pub last_error: Option<String>,
    pub master_info: Option<MasterInfo>,
    pub last_domain_state: Option<DomainState>,
    pub last_application_time_ns: Option<u64>,
    pub last_cycle_started_at_ns: Option<u64>,
    pub last_cycle_completed_at_ns: Option<u64>,
}

impl RuntimeStatus {
    fn new(config: &RuntimeConfig, library_path: Option<std::path::PathBuf>) -> Self {
        Self {
            state: RuntimeState::Created,
            library_path,
            cycle_time: config.cycle_time,
            configured_slave_count: 0,
            registered_pdo_entries: 0,
            pending_command_count: 0,
            completed_command_count: 0,
            cycle_count: 0,
            last_error: None,
            master_info: None,
            last_domain_state: None,
            last_application_time_ns: None,
            last_cycle_started_at_ns: None,
            last_cycle_completed_at_ns: None,
        }
    }
}

#[derive(Debug, Clone)]
pub enum RuntimeCommand {
    SetApplicationTime {
        app_time_ns: u64,
    },
    ReadBytes {
        offset: u32,
        size: usize,
    },
    WriteBytes {
        offset: u32,
        data: Vec<u8>,
    },
    ReadPdo {
        target: RuntimePdoTarget,
        size: usize,
    },
    WritePdo {
        target: RuntimePdoTarget,
        data: Vec<u8>,
    },
    Shutdown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommandOutput {
    Ack,
    Bytes(Vec<u8>),
    Shutdown,
}

#[derive(Debug, Clone)]
pub struct QueuedCommand {
    pub token: CommandToken,
    pub command: RuntimeCommand,
}

#[derive(Debug, Clone)]
pub struct CommandResult {
    pub token: CommandToken,
    pub outcome: std::result::Result<CommandOutput, String>,
}

struct ConfiguredSlave {
    config: SlaveRuntimeConfig,
    handle: SlaveConfig,
    offsets: BTreeMap<PdoEntryAddress, PdoEntryOffset>,
}

pub struct EthercatRuntime {
    config: RuntimeConfig,
    library: std::sync::Arc<EthercatLibrary>,
    master: Option<Master>,
    domain: Option<Domain>,
    slaves: Vec<ConfiguredSlave>,
    pdo_offsets: BTreeMap<RuntimePdoTarget, PdoEntryOffset>,
    pending_commands: VecDeque<QueuedCommand>,
    completed_commands: VecDeque<CommandResult>,
    next_token: CommandToken,
    status: RuntimeStatus,
}

impl EthercatRuntime {
    pub fn new(config: RuntimeConfig) -> Result<Self> {
        let library = match &config.library_path {
            Some(path) => EthercatLibrary::load_from_path(path.clone())?,
            None => EthercatLibrary::load_default()?,
        };
        let status = RuntimeStatus::new(&config, Some(library.path().to_path_buf()));

        Ok(Self {
            config,
            library,
            master: None,
            domain: None,
            slaves: Vec::new(),
            pdo_offsets: BTreeMap::new(),
            pending_commands: VecDeque::new(),
            completed_commands: VecDeque::new(),
            next_token: 1,
            status,
        })
    }

    pub fn configure(&mut self) -> Result<()> {
        if self.master.is_some() || self.domain.is_some() || !self.slaves.is_empty() {
            return Err(RuntimeError::AlreadyConfigured);
        }

        let mut master = self
            .library
            .open_master(self.config.master_index, self.config.access)?;
        let needs_domain = self
            .config
            .slaves
            .iter()
            .any(|slave| !slave.pdo_registrations.is_empty());
        let mut domain = if needs_domain {
            Some(master.create_domain()?)
        } else {
            None
        };

        let mut selected_reference_clock = false;

        for slave_cfg in self.config.slaves.clone() {
            let mut handle = master.configure_slave(slave_cfg.location, slave_cfg.identity)?;

            if !slave_cfg.sync_managers.is_empty() {
                handle.configure_pdos(&slave_cfg.sync_managers)?;
            }

            if let Some(watchdog) = slave_cfg.watchdog {
                handle.configure_watchdog(watchdog)?;
            }

            for sdo in &slave_cfg.startup_sdos {
                handle.configure_sdo(sdo.index, sdo.subindex, &sdo.data)?;
            }

            if let Some(dc) = slave_cfg.distributed_clock {
                handle.configure_dc(dc)?;
            }

            if slave_cfg.reference_clock
                || (!selected_reference_clock && slave_cfg.distributed_clock.is_some())
            {
                master.select_reference_clock(&handle)?;
                selected_reference_clock = true;
            }

            let mut offsets = BTreeMap::new();
            if let Some(domain_handle) = domain.as_mut() {
                let registered =
                    domain_handle.register_pdo_entries(&slave_cfg.pdo_registrations)?;
                for offset in registered {
                    let target = RuntimePdoTarget {
                        slave: slave_cfg.location,
                        entry: offset.entry,
                    };
                    self.pdo_offsets.insert(target, offset.clone());
                    offsets.insert(offset.entry, offset);
                }
            }

            self.slaves.push(ConfiguredSlave {
                config: slave_cfg,
                handle,
                offsets,
            });
        }

        self.status.configured_slave_count = self.slaves.len();
        self.status.registered_pdo_entries = self.pdo_offsets.len();
        self.status.master_info = Some(master.info()?);
        self.status.state = RuntimeState::Configured;

        self.master = Some(master);
        self.domain = domain;
        Ok(())
    }

    pub fn activate(&mut self) -> Result<()> {
        let master = self.master.as_mut().ok_or(RuntimeError::NotConfigured)?;
        if master.is_activated() {
            return Err(RuntimeError::AlreadyActive);
        }

        let app_time_ns = self
            .config
            .initial_application_time_ns
            .unwrap_or_else(monotonic_time_ns);
        master.set_application_time(app_time_ns)?;
        master.activate()?;

        self.status.last_application_time_ns = Some(app_time_ns);
        self.status.master_info = Some(master.info()?);
        self.status.state = RuntimeState::Activated;
        Ok(())
    }

    pub fn cycle_once(&mut self, application_time_ns: Option<u64>) -> Result<Vec<CommandResult>> {
        let started_at_ns = monotonic_time_ns();
        {
            let master = self.master.as_mut().ok_or(RuntimeError::NotConfigured)?;
            if !master.is_activated() {
                return Err(RuntimeError::NotActive);
            }
            self.status.state = RuntimeState::Running;

            if let Some(app_time_ns) = application_time_ns {
                master.set_application_time(app_time_ns)?;
                self.status.last_application_time_ns = Some(app_time_ns);
            }

            master.receive()?;
        }

        if let Some(domain) = self.domain.as_mut() {
            domain.process()?;
            self.status.last_domain_state = Some(domain.state()?);
        }

        let shutdown_requested = self.process_pending_commands();

        if let Some(domain) = self.domain.as_mut() {
            domain.queue()?;
        }

        {
            let master = self.master.as_mut().ok_or(RuntimeError::NotConfigured)?;
            if master.is_activated() {
                master.send()?;
                self.status.master_info = Some(master.info()?);
            }
        }

        self.status.cycle_count += 1;
        self.status.last_cycle_started_at_ns = Some(started_at_ns);
        self.status.last_cycle_completed_at_ns = Some(monotonic_time_ns());

        if shutdown_requested {
            self.shutdown()?;
        }

        Ok(self.drain_completed_commands())
    }

    pub fn queue_command(&mut self, command: RuntimeCommand) -> CommandToken {
        let token = self.next_token;
        self.next_token = self.next_token.saturating_add(1);
        self.pending_commands
            .push_back(QueuedCommand { token, command });
        self.status.pending_command_count = self.pending_commands.len();
        token
    }

    pub fn drain_completed_commands(&mut self) -> Vec<CommandResult> {
        let drained: Vec<_> = self.completed_commands.drain(..).collect();
        self.status.completed_command_count = self.completed_commands.len();
        drained
    }

    pub fn status(&self) -> &RuntimeStatus {
        &self.status
    }

    pub fn pdo_offset(&self, target: RuntimePdoTarget) -> Option<&PdoEntryOffset> {
        self.pdo_offsets.get(&target)
    }

    pub fn read_pdo_bytes(&self, target: RuntimePdoTarget, size: usize) -> Result<Vec<u8>> {
        let offset = self.lookup_pdo_offset(target)?;
        let domain = self.domain.as_ref().ok_or(RuntimeError::NoDomain)?;
        domain
            .read_bytes(offset.byte_offset as usize, size)
            .map_err(RuntimeError::from)
    }

    pub fn shutdown(&mut self) -> Result<()> {
        self.status.state = RuntimeState::Stopping;

        self.pending_commands.clear();
        self.status.pending_command_count = 0;

        self.domain = None;
        self.slaves.clear();
        self.pdo_offsets.clear();
        self.status.registered_pdo_entries = 0;
        self.status.configured_slave_count = 0;

        if let Some(mut master) = self.master.take() {
            if master.is_activated() {
                master.deactivate()?;
            }
            master.release();
        }

        self.status.master_info = None;
        self.status.last_domain_state = None;
        self.status.state = RuntimeState::Stopped;
        Ok(())
    }

    pub(crate) fn configured_slaves(&self) -> &[ConfiguredSlave] {
        &self.slaves
    }

    fn process_pending_commands(&mut self) -> bool {
        let mut shutdown_requested = false;

        while let Some(queued) = self.pending_commands.pop_front() {
            let outcome = self
                .execute_command(&queued.command)
                .map_err(|err| err.to_string());
            if matches!(queued.command, RuntimeCommand::Shutdown) && outcome.is_ok() {
                shutdown_requested = true;
            }
            self.completed_commands.push_back(CommandResult {
                token: queued.token,
                outcome,
            });
        }

        self.status.pending_command_count = self.pending_commands.len();
        self.status.completed_command_count = self.completed_commands.len();
        shutdown_requested
    }

    fn execute_command(&mut self, command: &RuntimeCommand) -> Result<CommandOutput> {
        match command {
            RuntimeCommand::SetApplicationTime { app_time_ns } => {
                let master = self.master.as_mut().ok_or(RuntimeError::NotConfigured)?;
                if !master.is_activated() {
                    return Err(RuntimeError::NotActive);
                }
                master.set_application_time(*app_time_ns)?;
                self.status.last_application_time_ns = Some(*app_time_ns);
                Ok(CommandOutput::Ack)
            }
            RuntimeCommand::ReadBytes { offset, size } => {
                let domain = self.domain.as_ref().ok_or(RuntimeError::NoDomain)?;
                let bytes = domain.read_bytes(*offset as usize, *size)?;
                Ok(CommandOutput::Bytes(bytes))
            }
            RuntimeCommand::WriteBytes { offset, data } => {
                let domain = self.domain.as_mut().ok_or(RuntimeError::NoDomain)?;
                domain.write_bytes(*offset as usize, data)?;
                Ok(CommandOutput::Ack)
            }
            RuntimeCommand::ReadPdo { target, size } => {
                let offset = self.lookup_pdo_offset(*target)?;
                let domain = self.domain.as_ref().ok_or(RuntimeError::NoDomain)?;
                let bytes = domain.read_bytes(offset.byte_offset as usize, *size)?;
                Ok(CommandOutput::Bytes(bytes))
            }
            RuntimeCommand::WritePdo { target, data } => {
                let byte_offset = self.lookup_pdo_offset(*target)?.byte_offset as usize;
                let domain = self.domain.as_mut().ok_or(RuntimeError::NoDomain)?;
                domain.write_bytes(byte_offset, data)?;
                Ok(CommandOutput::Ack)
            }
            RuntimeCommand::Shutdown => Ok(CommandOutput::Shutdown),
        }
    }

    fn lookup_pdo_offset(&self, target: RuntimePdoTarget) -> Result<&PdoEntryOffset> {
        self.pdo_offsets
            .get(&target)
            .ok_or(RuntimeError::UnknownPdoEntry {
                alias: target.slave.alias,
                position: target.slave.position,
                index: target.entry.index,
                subindex: target.entry.subindex,
            })
    }
}

pub struct EthercatProcessManager {
    runtime: EthercatRuntime,
}

impl EthercatProcessManager {
    pub fn new(config: RuntimeConfig) -> Result<Self> {
        Ok(Self {
            runtime: EthercatRuntime::new(config)?,
        })
    }

    pub fn start(&mut self) -> Result<()> {
        match self.runtime.status.state {
            RuntimeState::Created | RuntimeState::Stopped => {
                self.runtime.configure()?;
                self.runtime.activate()
            }
            RuntimeState::Configured => self.runtime.activate(),
            RuntimeState::Activated | RuntimeState::Running => Ok(()),
            RuntimeState::Stopping | RuntimeState::Faulted => Err(RuntimeError::NotConfigured),
        }
    }

    pub fn step(&mut self) -> Result<Vec<CommandResult>> {
        self.runtime.cycle_once(None)
    }

    pub fn step_with_application_time(
        &mut self,
        application_time_ns: u64,
    ) -> Result<Vec<CommandResult>> {
        self.runtime.cycle_once(Some(application_time_ns))
    }

    pub fn queue_command(&mut self, command: RuntimeCommand) -> CommandToken {
        self.runtime.queue_command(command)
    }

    pub fn drain_completed_commands(&mut self) -> Vec<CommandResult> {
        self.runtime.drain_completed_commands()
    }

    pub fn status(&self) -> &RuntimeStatus {
        self.runtime.status()
    }

    pub fn is_alive(&self) -> bool {
        matches!(
            self.runtime.status().state,
            RuntimeState::Configured | RuntimeState::Activated | RuntimeState::Running
        )
    }

    pub fn read_pdo_bytes(&self, target: RuntimePdoTarget, size: usize) -> Result<Vec<u8>> {
        self.runtime.read_pdo_bytes(target, size)
    }

    pub fn runtime(&self) -> &EthercatRuntime {
        &self.runtime
    }

    pub fn runtime_mut(&mut self) -> &mut EthercatRuntime {
        &mut self.runtime
    }

    pub fn stop(&mut self) -> Result<()> {
        self.runtime.shutdown()
    }
}

pub(crate) fn monotonic_time_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };

    let rc = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    if rc != 0 {
        return 0;
    }

    (ts.tv_sec as u64)
        .saturating_mul(1_000_000_000)
        .saturating_add(ts.tv_nsec as u64)
}
