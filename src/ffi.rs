use std::collections::BTreeMap;
use std::ffi::{c_int, c_uint};
use std::path::{Path, PathBuf};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use libloading::{Library, Symbol};
use thiserror::Error;

pub type Result<T> = std::result::Result<T, EthercatFfiError>;

#[derive(Debug, Error)]
pub enum EthercatFfiError {
    #[error("failed to load libethercat: {0}")]
    LibraryLoad(String),
    #[error("required symbol `{0}` missing from libethercat")]
    MissingSymbol(&'static str),
    #[error("{0} returned a null handle")]
    NullHandle(&'static str),
    #[error("{function} failed with code {code}")]
    CallFailed { function: &'static str, code: i32 },
    #[error("optional IgH API `{0}` is not available in the loaded library")]
    Unsupported(&'static str),
    #[error("master-owned handle is no longer valid")]
    MasterReleased,
    #[error("PDO access out of bounds: offset={offset}, size={size}, mapped_len={mapped_len}")]
    OutOfBounds {
        offset: usize,
        size: usize,
        mapped_len: usize,
    },
}

#[repr(C)]
pub struct ec_master_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct ec_domain_t {
    _private: [u8; 0],
}

#[repr(C)]
pub struct ec_slave_config_t {
    _private: [u8; 0],
}

#[repr(C)]
struct EcMasterInfoRaw {
    slave_count: c_uint,
    link_up_bits: c_uint,
    scan_busy: u8,
    _padding: [u8; 7],
    app_time: u64,
}

#[repr(C)]
struct EcSlaveConfigStateRaw {
    bits: c_uint,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct EcDomainStateRaw {
    working_counter: c_uint,
    wc_state: c_int,
    redundancy_active: c_uint,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct EcPdoEntryReg {
    alias: u16,
    position: u16,
    vendor_id: u32,
    product_code: u32,
    index: u16,
    subindex: u8,
    offset: *mut c_uint,
    bit_position: *mut c_uint,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct EcPdoEntryInfo {
    index: u16,
    subindex: u8,
    bit_length: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct EcPdoInfo {
    index: u16,
    n_entries: c_uint,
    entries: *const EcPdoEntryInfo,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
struct EcSyncInfo {
    index: u8,
    dir: c_int,
    n_pdos: c_uint,
    pdos: *const EcPdoInfo,
    watchdog_mode: c_int,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MasterAccess {
    Open,
    Request,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Direction {
    Invalid = 0,
    Output = 1,
    Input = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum WatchdogMode {
    Default = 0,
    Enable = 1,
    Disable = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum WorkingCounterState {
    Zero = 0,
    Incomplete = 1,
    Complete = 2,
    Unknown = -1,
}

impl WorkingCounterState {
    fn from_raw(raw: i32) -> Self {
        match raw {
            0 => Self::Zero,
            1 => Self::Incomplete,
            2 => Self::Complete,
            _ => Self::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SlaveLocation {
    pub alias: u16,
    pub position: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlaveIdentity {
    pub vendor_id: u32,
    pub product_code: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PdoEntryAddress {
    pub index: u16,
    pub subindex: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PdoEntryRegistration {
    pub slave: SlaveLocation,
    pub identity: SlaveIdentity,
    pub entry: PdoEntryAddress,
    pub bit_length: Option<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PdoEntryOffset {
    pub entry: PdoEntryAddress,
    pub byte_offset: u32,
    pub bit_position: u32,
    pub bit_length: Option<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PdoEntryConfig {
    pub index: u16,
    pub subindex: u8,
    pub bit_length: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PdoConfig {
    pub index: u16,
    pub entries: Vec<PdoEntryConfig>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SyncManagerConfig {
    pub index: u8,
    pub direction: Direction,
    pub pdos: Vec<PdoConfig>,
    pub watchdog_mode: WatchdogMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DistributedClockConfig {
    pub assign_activate: u16,
    pub sync0_cycle_ns: u32,
    pub sync0_shift_ns: i32,
    pub sync1_cycle_ns: u32,
    pub sync1_shift_ns: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WatchdogConfig {
    pub divider: u16,
    pub intervals: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MasterInfo {
    pub slave_count: u32,
    pub link_up: bool,
    pub scan_busy: bool,
    pub app_time_ns: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DomainState {
    pub working_counter: u32,
    pub wc_state: WorkingCounterState,
    pub redundancy_active: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlaveConfigState {
    pub online: bool,
    pub operational: bool,
    pub al_state: u8,
}

type EcrtOpenMaster = unsafe extern "C" fn(c_uint) -> *mut ec_master_t;
type EcrtRequestMaster = unsafe extern "C" fn(c_uint) -> *mut ec_master_t;
type EcrtReleaseMaster = unsafe extern "C" fn(*mut ec_master_t);
type EcrtMasterActivate = unsafe extern "C" fn(*mut ec_master_t) -> c_int;
type EcrtMasterDeactivate = unsafe extern "C" fn(*mut ec_master_t) -> c_int;
type EcrtMasterApplicationTime = unsafe extern "C" fn(*mut ec_master_t, u64) -> c_int;
type EcrtMasterInfo = unsafe extern "C" fn(*mut ec_master_t, *mut EcMasterInfoRaw) -> c_int;
type EcrtMasterCreateDomain = unsafe extern "C" fn(*mut ec_master_t) -> *mut ec_domain_t;
type EcrtDomainData = unsafe extern "C" fn(*const ec_domain_t) -> *mut u8;
type EcrtDomainProcess = unsafe extern "C" fn(*mut ec_domain_t) -> c_int;
type EcrtDomainQueue = unsafe extern "C" fn(*mut ec_domain_t) -> c_int;
type EcrtDomainState = unsafe extern "C" fn(*const ec_domain_t, *mut EcDomainStateRaw) -> c_int;
type EcrtDomainRegPdoEntryList =
    unsafe extern "C" fn(*mut ec_domain_t, *const EcPdoEntryReg) -> c_int;
type EcrtMasterSlaveConfig =
    unsafe extern "C" fn(*mut ec_master_t, u16, u16, u32, u32) -> *mut ec_slave_config_t;
type EcrtSlaveConfigPdos =
    unsafe extern "C" fn(*mut ec_slave_config_t, c_uint, *const EcSyncInfo) -> c_int;
type EcrtSlaveConfigDc =
    unsafe extern "C" fn(*mut ec_slave_config_t, u16, u32, i32, u32, i32) -> c_int;
type EcrtSlaveConfigSdo =
    unsafe extern "C" fn(*mut ec_slave_config_t, u16, u8, *const u8, usize) -> c_int;
type EcrtSlaveConfigState =
    unsafe extern "C" fn(*const ec_slave_config_t, *mut EcSlaveConfigStateRaw) -> c_int;
type EcrtSlaveConfigWatchdog = unsafe extern "C" fn(*mut ec_slave_config_t, u16, u16) -> c_int;
type EcrtMasterSelectReferenceClock =
    unsafe extern "C" fn(*mut ec_master_t, *mut ec_slave_config_t) -> c_int;
type EcrtMasterSyncReferenceClock = unsafe extern "C" fn(*mut ec_master_t) -> c_int;
type EcrtMasterSyncSlaveClocks = unsafe extern "C" fn(*mut ec_master_t) -> c_int;
type EcrtMasterSend = unsafe extern "C" fn(*mut ec_master_t) -> c_int;
type EcrtMasterReceive = unsafe extern "C" fn(*mut ec_master_t) -> c_int;

#[derive(Debug)]
struct DomainRegistrationKeepalive {
    regs: Vec<EcPdoEntryReg>,
    offsets: Vec<c_uint>,
    bit_positions: Vec<c_uint>,
}

#[derive(Debug)]
struct SyncConfigKeepalive {
    syncs: Vec<EcSyncInfo>,
    pdos: Vec<Vec<EcPdoInfo>>,
    entries: Vec<Vec<Vec<EcPdoEntryInfo>>>,
}

pub struct EthercatLibrary {
    _lib: Library,
    path: PathBuf,
    open_master: EcrtOpenMaster,
    request_master: EcrtRequestMaster,
    release_master: EcrtReleaseMaster,
    master_activate: EcrtMasterActivate,
    master_deactivate: EcrtMasterDeactivate,
    master_application_time: EcrtMasterApplicationTime,
    master_info: EcrtMasterInfo,
    master_create_domain: EcrtMasterCreateDomain,
    domain_data: EcrtDomainData,
    domain_process: EcrtDomainProcess,
    domain_queue: EcrtDomainQueue,
    domain_state: EcrtDomainState,
    domain_reg_pdo_entry_list: EcrtDomainRegPdoEntryList,
    master_slave_config: EcrtMasterSlaveConfig,
    slave_config_pdos: EcrtSlaveConfigPdos,
    slave_config_dc: EcrtSlaveConfigDc,
    slave_config_sdo: EcrtSlaveConfigSdo,
    slave_config_state: EcrtSlaveConfigState,
    slave_config_watchdog: Option<EcrtSlaveConfigWatchdog>,
    master_select_reference_clock: EcrtMasterSelectReferenceClock,
    master_sync_reference_clock: EcrtMasterSyncReferenceClock,
    master_sync_slave_clocks: EcrtMasterSyncSlaveClocks,
    master_send: EcrtMasterSend,
    master_receive: EcrtMasterReceive,
}

impl EthercatLibrary {
    pub fn load_default() -> Result<Arc<Self>> {
        Self::load_from_candidates(Self::default_candidates())
    }

    pub fn load_from_path(path: impl Into<PathBuf>) -> Result<Arc<Self>> {
        Self::load_from_candidates(vec![path.into()])
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn open_master(
        self: &Arc<Self>,
        master_index: u32,
        access: MasterAccess,
    ) -> Result<Master> {
        let raw = unsafe {
            match access {
                MasterAccess::Open => (self.open_master)(master_index),
                MasterAccess::Request => (self.request_master)(master_index),
            }
        };
        let handle = NonNull::new(raw).ok_or(EthercatFfiError::NullHandle(match access {
            MasterAccess::Open => "ecrt_open_master",
            MasterAccess::Request => "ecrt_request_master",
        }))?;

        Ok(Master {
            api: Arc::clone(self),
            handle,
            activated: false,
            alive: Arc::new(AtomicBool::new(true)),
        })
    }

    fn load_from_candidates(candidates: Vec<PathBuf>) -> Result<Arc<Self>> {
        let mut attempts = Vec::new();

        for candidate in candidates {
            let lib = unsafe { Library::new(&candidate) };
            match lib {
                Ok(lib) => {
                    let api = Self::from_library(lib, candidate)?;
                    return Ok(Arc::new(api));
                }
                Err(err) => attempts.push(format!("{} ({err})", candidate.display())),
            }
        }

        Err(EthercatFfiError::LibraryLoad(attempts.join("; ")))
    }

    fn from_library(lib: Library, path: PathBuf) -> Result<Self> {
        Ok(Self {
            open_master: Self::load_symbol(&lib, b"ecrt_open_master\0", "ecrt_open_master")?,
            request_master: Self::load_symbol(
                &lib,
                b"ecrt_request_master\0",
                "ecrt_request_master",
            )?,
            release_master: Self::load_symbol(
                &lib,
                b"ecrt_release_master\0",
                "ecrt_release_master",
            )?,
            master_activate: Self::load_symbol(
                &lib,
                b"ecrt_master_activate\0",
                "ecrt_master_activate",
            )?,
            master_deactivate: Self::load_symbol(
                &lib,
                b"ecrt_master_deactivate\0",
                "ecrt_master_deactivate",
            )?,
            master_application_time: Self::load_symbol(
                &lib,
                b"ecrt_master_application_time\0",
                "ecrt_master_application_time",
            )?,
            master_info: Self::load_symbol(&lib, b"ecrt_master\0", "ecrt_master")?,
            master_create_domain: Self::load_symbol(
                &lib,
                b"ecrt_master_create_domain\0",
                "ecrt_master_create_domain",
            )?,
            domain_data: Self::load_symbol(&lib, b"ecrt_domain_data\0", "ecrt_domain_data")?,
            domain_process: Self::load_symbol(
                &lib,
                b"ecrt_domain_process\0",
                "ecrt_domain_process",
            )?,
            domain_queue: Self::load_symbol(&lib, b"ecrt_domain_queue\0", "ecrt_domain_queue")?,
            domain_state: Self::load_symbol(&lib, b"ecrt_domain_state\0", "ecrt_domain_state")?,
            domain_reg_pdo_entry_list: Self::load_symbol(
                &lib,
                b"ecrt_domain_reg_pdo_entry_list\0",
                "ecrt_domain_reg_pdo_entry_list",
            )?,
            master_slave_config: Self::load_symbol(
                &lib,
                b"ecrt_master_slave_config\0",
                "ecrt_master_slave_config",
            )?,
            slave_config_pdos: Self::load_symbol(
                &lib,
                b"ecrt_slave_config_pdos\0",
                "ecrt_slave_config_pdos",
            )?,
            slave_config_dc: Self::load_symbol(
                &lib,
                b"ecrt_slave_config_dc\0",
                "ecrt_slave_config_dc",
            )?,
            slave_config_sdo: Self::load_symbol(
                &lib,
                b"ecrt_slave_config_sdo\0",
                "ecrt_slave_config_sdo",
            )?,
            slave_config_state: Self::load_symbol(
                &lib,
                b"ecrt_slave_config_state\0",
                "ecrt_slave_config_state",
            )?,
            slave_config_watchdog: Self::load_optional_symbol(
                &lib,
                b"ecrt_slave_config_watchdog\0",
            ),
            master_select_reference_clock: Self::load_symbol(
                &lib,
                b"ecrt_master_select_reference_clock\0",
                "ecrt_master_select_reference_clock",
            )?,
            master_sync_reference_clock: Self::load_symbol(
                &lib,
                b"ecrt_master_sync_reference_clock\0",
                "ecrt_master_sync_reference_clock",
            )?,
            master_sync_slave_clocks: Self::load_symbol(
                &lib,
                b"ecrt_master_sync_slave_clocks\0",
                "ecrt_master_sync_slave_clocks",
            )?,
            master_send: Self::load_symbol(&lib, b"ecrt_master_send\0", "ecrt_master_send")?,
            master_receive: Self::load_symbol(
                &lib,
                b"ecrt_master_receive\0",
                "ecrt_master_receive",
            )?,
            _lib: lib,
            path,
        })
    }

    fn load_symbol<T: Copy>(lib: &Library, raw_name: &[u8], name: &'static str) -> Result<T> {
        let symbol: Symbol<'_, T> =
            unsafe { lib.get(raw_name) }.map_err(|_| EthercatFfiError::MissingSymbol(name))?;
        Ok(*symbol)
    }

    fn load_optional_symbol<T: Copy>(lib: &Library, raw_name: &[u8]) -> Option<T> {
        unsafe { lib.get::<T>(raw_name) }.ok().map(|symbol| *symbol)
    }

    fn default_candidates() -> Vec<PathBuf> {
        let mut candidates = Vec::new();

        for env_name in ["LIBETHERCAT_SO_PATH", "LIBETHERCAT_SO"] {
            if let Some(value) = std::env::var_os(env_name) {
                if !value.is_empty() {
                    candidates.push(PathBuf::from(value));
                }
            }
        }

        candidates.extend([
            PathBuf::from("libethercat.so.1"),
            PathBuf::from("/usr/local/lib/libethercat.so.1"),
            PathBuf::from("/usr/lib/libethercat.so.1"),
            PathBuf::from("/usr/lib/x86_64-linux-gnu/libethercat.so.1"),
            PathBuf::from("/usr/lib/aarch64-linux-gnu/libethercat.so.1"),
            PathBuf::from("/lib/x86_64-linux-gnu/libethercat.so.1"),
            PathBuf::from("/lib/aarch64-linux-gnu/libethercat.so.1"),
            PathBuf::from("/opt/etherlab/lib/libethercat.so.1"),
        ]);

        candidates
    }
}

pub struct Master {
    api: Arc<EthercatLibrary>,
    handle: NonNull<ec_master_t>,
    activated: bool,
    alive: Arc<AtomicBool>,
}

impl Master {
    pub fn create_domain(&mut self) -> Result<Domain> {
        self.ensure_alive()?;
        let raw = unsafe { (self.api.master_create_domain)(self.handle.as_ptr()) };
        let handle =
            NonNull::new(raw).ok_or(EthercatFfiError::NullHandle("ecrt_master_create_domain"))?;

        Ok(Domain {
            api: Arc::clone(&self.api),
            handle,
            master_alive: Arc::clone(&self.alive),
            mapped_len: 0,
            offsets: BTreeMap::new(),
            keepalive: Vec::new(),
        })
    }

    pub fn configure_slave(
        &mut self,
        location: SlaveLocation,
        identity: SlaveIdentity,
    ) -> Result<SlaveConfig> {
        self.ensure_alive()?;
        let raw = unsafe {
            (self.api.master_slave_config)(
                self.handle.as_ptr(),
                location.alias,
                location.position,
                identity.vendor_id,
                identity.product_code,
            )
        };
        let handle =
            NonNull::new(raw).ok_or(EthercatFfiError::NullHandle("ecrt_master_slave_config"))?;

        Ok(SlaveConfig {
            api: Arc::clone(&self.api),
            handle,
            master_alive: Arc::clone(&self.alive),
            location,
            identity,
            keepalive: Vec::new(),
        })
    }

    pub fn activate(&mut self) -> Result<()> {
        self.ensure_alive()?;
        if self.activated {
            return Ok(());
        }

        let code = unsafe { (self.api.master_activate)(self.handle.as_ptr()) };
        check_code("ecrt_master_activate", code)?;
        self.activated = true;
        Ok(())
    }

    pub fn deactivate(&mut self) -> Result<()> {
        self.ensure_alive()?;
        if !self.activated {
            return Ok(());
        }

        let code = unsafe { (self.api.master_deactivate)(self.handle.as_ptr()) };
        check_code("ecrt_master_deactivate", code)?;
        self.activated = false;
        Ok(())
    }

    pub fn set_application_time(&mut self, app_time_ns: u64) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe { (self.api.master_application_time)(self.handle.as_ptr(), app_time_ns) };
        check_code("ecrt_master_application_time", code)
    }

    pub fn info(&mut self) -> Result<MasterInfo> {
        self.ensure_alive()?;
        let mut raw = EcMasterInfoRaw {
            slave_count: 0,
            link_up_bits: 0,
            scan_busy: 0,
            _padding: [0; 7],
            app_time: 0,
        };
        let code = unsafe { (self.api.master_info)(self.handle.as_ptr(), &mut raw) };
        check_code("ecrt_master", code)?;

        Ok(MasterInfo {
            slave_count: raw.slave_count,
            link_up: (raw.link_up_bits & 0x1) != 0,
            scan_busy: raw.scan_busy != 0,
            app_time_ns: raw.app_time,
        })
    }

    pub fn select_reference_clock(&mut self, config: &SlaveConfig) -> Result<()> {
        self.ensure_alive()?;
        config.ensure_alive()?;
        let code = unsafe {
            (self.api.master_select_reference_clock)(self.handle.as_ptr(), config.handle.as_ptr())
        };
        check_code("ecrt_master_select_reference_clock", code)
    }

    pub fn sync_reference_clock(&mut self) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe { (self.api.master_sync_reference_clock)(self.handle.as_ptr()) };
        check_code("ecrt_master_sync_reference_clock", code)
    }

    pub fn sync_slave_clocks(&mut self) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe { (self.api.master_sync_slave_clocks)(self.handle.as_ptr()) };
        check_code("ecrt_master_sync_slave_clocks", code)
    }

    pub fn send(&mut self) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe { (self.api.master_send)(self.handle.as_ptr()) };
        check_code("ecrt_master_send", code)
    }

    pub fn receive(&mut self) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe { (self.api.master_receive)(self.handle.as_ptr()) };
        check_code("ecrt_master_receive", code)
    }

    pub fn release(&mut self) {
        if !self.alive.swap(false, Ordering::SeqCst) {
            return;
        }

        if self.activated {
            let _ = unsafe { (self.api.master_deactivate)(self.handle.as_ptr()) };
            self.activated = false;
        }

        unsafe { (self.api.release_master)(self.handle.as_ptr()) };
    }

    pub fn is_activated(&self) -> bool {
        self.activated
    }

    pub fn library_path(&self) -> &Path {
        self.api.path()
    }

    fn ensure_alive(&self) -> Result<()> {
        if self.alive.load(Ordering::SeqCst) {
            Ok(())
        } else {
            Err(EthercatFfiError::MasterReleased)
        }
    }
}

impl Drop for Master {
    fn drop(&mut self) {
        self.release();
    }
}

pub struct Domain {
    api: Arc<EthercatLibrary>,
    handle: NonNull<ec_domain_t>,
    master_alive: Arc<AtomicBool>,
    mapped_len: usize,
    offsets: BTreeMap<PdoEntryAddress, PdoEntryOffset>,
    keepalive: Vec<DomainRegistrationKeepalive>,
}

impl Domain {
    pub fn register_pdo_entries(
        &mut self,
        entries: &[PdoEntryRegistration],
    ) -> Result<Vec<PdoEntryOffset>> {
        self.ensure_alive()?;
        if entries.is_empty() {
            return Ok(Vec::new());
        }

        let mut regs = vec![EcPdoEntryReg::default(); entries.len() + 1];
        let mut offsets = vec![0_u32; entries.len()];
        let mut bit_positions = vec![0_u32; entries.len()];

        for (idx, entry) in entries.iter().enumerate() {
            regs[idx] = EcPdoEntryReg {
                alias: entry.slave.alias,
                position: entry.slave.position,
                vendor_id: entry.identity.vendor_id,
                product_code: entry.identity.product_code,
                index: entry.entry.index,
                subindex: entry.entry.subindex,
                offset: &mut offsets[idx],
                bit_position: &mut bit_positions[idx],
            };
        }

        let code =
            unsafe { (self.api.domain_reg_pdo_entry_list)(self.handle.as_ptr(), regs.as_ptr()) };
        check_code("ecrt_domain_reg_pdo_entry_list", code)?;

        let mut resolved = Vec::with_capacity(entries.len());
        for (idx, entry) in entries.iter().enumerate() {
            let resolved_entry = PdoEntryOffset {
                entry: entry.entry,
                byte_offset: offsets[idx],
                bit_position: bit_positions[idx],
                bit_length: entry.bit_length,
            };
            let width = bytes_for_entry(entry.bit_length);
            self.mapped_len = self
                .mapped_len
                .max(resolved_entry.byte_offset as usize + width.max(1));
            self.offsets.insert(entry.entry, resolved_entry.clone());
            resolved.push(resolved_entry);
        }

        self.keepalive.push(DomainRegistrationKeepalive {
            regs,
            offsets,
            bit_positions,
        });

        Ok(resolved)
    }

    pub fn offset(&self, entry: PdoEntryAddress) -> Option<&PdoEntryOffset> {
        self.offsets.get(&entry)
    }

    pub fn mapped_len(&self) -> usize {
        self.mapped_len
    }

    pub fn process(&mut self) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe { (self.api.domain_process)(self.handle.as_ptr()) };
        check_code("ecrt_domain_process", code)
    }

    pub fn queue(&mut self) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe { (self.api.domain_queue)(self.handle.as_ptr()) };
        check_code("ecrt_domain_queue", code)
    }

    pub fn state(&self) -> Result<DomainState> {
        self.ensure_alive()?;
        let mut raw = EcDomainStateRaw::default();
        let code = unsafe { (self.api.domain_state)(self.handle.as_ptr(), &mut raw) };
        check_code("ecrt_domain_state", code)?;

        Ok(DomainState {
            working_counter: raw.working_counter,
            wc_state: WorkingCounterState::from_raw(raw.wc_state),
            redundancy_active: raw.redundancy_active != 0,
        })
    }

    pub fn read_bytes(&self, offset: usize, size: usize) -> Result<Vec<u8>> {
        self.ensure_alive()?;
        self.check_bounds(offset, size)?;
        let ptr = self.data_ptr()?;
        let slice = unsafe { std::slice::from_raw_parts(ptr.as_ptr().add(offset), size) };
        Ok(slice.to_vec())
    }

    pub fn write_bytes(&mut self, offset: usize, data: &[u8]) -> Result<()> {
        self.ensure_alive()?;
        self.check_bounds(offset, data.len())?;
        let ptr = self.data_ptr()?;
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), ptr.as_ptr().add(offset), data.len());
        }
        Ok(())
    }

    pub fn read_u8(&self, offset: usize) -> Result<u8> {
        Ok(self.read_bytes(offset, 1)?[0])
    }

    pub fn read_u16(&self, offset: usize) -> Result<u16> {
        let raw = self.read_bytes(offset, 2)?;
        Ok(u16::from_le_bytes([raw[0], raw[1]]))
    }

    pub fn read_i16(&self, offset: usize) -> Result<i16> {
        Ok(self.read_u16(offset)? as i16)
    }

    pub fn read_u32(&self, offset: usize) -> Result<u32> {
        let raw = self.read_bytes(offset, 4)?;
        Ok(u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
    }

    pub fn read_i32(&self, offset: usize) -> Result<i32> {
        Ok(self.read_u32(offset)? as i32)
    }

    pub fn write_u8(&mut self, offset: usize, value: u8) -> Result<()> {
        self.write_bytes(offset, &[value])
    }

    pub fn write_u16(&mut self, offset: usize, value: u16) -> Result<()> {
        self.write_bytes(offset, &value.to_le_bytes())
    }

    pub fn write_i16(&mut self, offset: usize, value: i16) -> Result<()> {
        self.write_bytes(offset, &value.to_le_bytes())
    }

    pub fn write_u32(&mut self, offset: usize, value: u32) -> Result<()> {
        self.write_bytes(offset, &value.to_le_bytes())
    }

    pub fn write_i32(&mut self, offset: usize, value: i32) -> Result<()> {
        self.write_bytes(offset, &value.to_le_bytes())
    }

    fn data_ptr(&self) -> Result<NonNull<u8>> {
        let raw = unsafe { (self.api.domain_data)(self.handle.as_ptr()) };
        NonNull::new(raw).ok_or(EthercatFfiError::NullHandle("ecrt_domain_data"))
    }

    fn check_bounds(&self, offset: usize, size: usize) -> Result<()> {
        if size == 0 {
            return Ok(());
        }

        let end = offset.saturating_add(size);
        if self.mapped_len == 0 || end > self.mapped_len {
            return Err(EthercatFfiError::OutOfBounds {
                offset,
                size,
                mapped_len: self.mapped_len,
            });
        }
        Ok(())
    }

    fn ensure_alive(&self) -> Result<()> {
        if self.master_alive.load(Ordering::SeqCst) {
            Ok(())
        } else {
            Err(EthercatFfiError::MasterReleased)
        }
    }
}

pub struct SlaveConfig {
    api: Arc<EthercatLibrary>,
    handle: NonNull<ec_slave_config_t>,
    master_alive: Arc<AtomicBool>,
    pub location: SlaveLocation,
    pub identity: SlaveIdentity,
    keepalive: Vec<SyncConfigKeepalive>,
}

impl SlaveConfig {
    pub fn configure_pdos(&mut self, syncs: &[SyncManagerConfig]) -> Result<()> {
        self.ensure_alive()?;

        let mut entry_keepalive: Vec<Vec<Vec<EcPdoEntryInfo>>> = Vec::with_capacity(syncs.len());
        let mut pdo_keepalive: Vec<Vec<EcPdoInfo>> = Vec::with_capacity(syncs.len());
        let mut sync_keepalive: Vec<EcSyncInfo> = Vec::with_capacity(syncs.len() + 1);

        for sync in syncs {
            let mut raw_entries_for_sync = Vec::with_capacity(sync.pdos.len());
            let mut raw_pdos_for_sync = Vec::with_capacity(sync.pdos.len());

            for pdo in &sync.pdos {
                let raw_entries: Vec<EcPdoEntryInfo> = pdo
                    .entries
                    .iter()
                    .map(|entry| EcPdoEntryInfo {
                        index: entry.index,
                        subindex: entry.subindex,
                        bit_length: entry.bit_length,
                    })
                    .collect();
                raw_entries_for_sync.push(raw_entries);
            }

            for (pdo_idx, pdo) in sync.pdos.iter().enumerate() {
                let entry_ptr = raw_entries_for_sync[pdo_idx].as_ptr();
                raw_pdos_for_sync.push(EcPdoInfo {
                    index: pdo.index,
                    n_entries: pdo.entries.len() as c_uint,
                    entries: entry_ptr,
                });
            }

            sync_keepalive.push(EcSyncInfo {
                index: sync.index,
                dir: sync.direction as c_int,
                n_pdos: raw_pdos_for_sync.len() as c_uint,
                pdos: raw_pdos_for_sync.as_ptr(),
                watchdog_mode: sync.watchdog_mode as c_int,
            });

            entry_keepalive.push(raw_entries_for_sync);
            pdo_keepalive.push(raw_pdos_for_sync);
        }

        sync_keepalive.push(EcSyncInfo {
            index: 0xff,
            dir: 0,
            n_pdos: 0,
            pdos: std::ptr::null(),
            watchdog_mode: WatchdogMode::Default as c_int,
        });

        let code = unsafe {
            (self.api.slave_config_pdos)(
                self.handle.as_ptr(),
                syncs.len() as c_uint,
                sync_keepalive.as_ptr(),
            )
        };
        check_code("ecrt_slave_config_pdos", code)?;

        self.keepalive.push(SyncConfigKeepalive {
            syncs: sync_keepalive,
            pdos: pdo_keepalive,
            entries: entry_keepalive,
        });

        Ok(())
    }

    pub fn configure_dc(&mut self, dc: DistributedClockConfig) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe {
            (self.api.slave_config_dc)(
                self.handle.as_ptr(),
                dc.assign_activate,
                dc.sync0_cycle_ns,
                dc.sync0_shift_ns,
                dc.sync1_cycle_ns,
                dc.sync1_shift_ns,
            )
        };
        check_code("ecrt_slave_config_dc", code)
    }

    pub fn configure_sdo(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<()> {
        self.ensure_alive()?;
        let code = unsafe {
            (self.api.slave_config_sdo)(
                self.handle.as_ptr(),
                index,
                subindex,
                data.as_ptr(),
                data.len(),
            )
        };
        check_code("ecrt_slave_config_sdo", code)
    }

    pub fn configure_watchdog(&mut self, watchdog: WatchdogConfig) -> Result<()> {
        self.ensure_alive()?;
        let watchdog_fn = self
            .api
            .slave_config_watchdog
            .ok_or(EthercatFfiError::Unsupported("ecrt_slave_config_watchdog"))?;
        let code =
            unsafe { watchdog_fn(self.handle.as_ptr(), watchdog.divider, watchdog.intervals) };
        check_code("ecrt_slave_config_watchdog", code)
    }

    pub fn state(&self) -> Result<SlaveConfigState> {
        self.ensure_alive()?;
        let mut raw = EcSlaveConfigStateRaw { bits: 0 };
        let code = unsafe { (self.api.slave_config_state)(self.handle.as_ptr(), &mut raw) };
        check_code("ecrt_slave_config_state", code)?;

        Ok(SlaveConfigState {
            online: (raw.bits & 0b000001) != 0,
            operational: (raw.bits & 0b000010) != 0,
            al_state: ((raw.bits >> 2) & 0b00_1111) as u8,
        })
    }

    fn ensure_alive(&self) -> Result<()> {
        if self.master_alive.load(Ordering::SeqCst) {
            Ok(())
        } else {
            Err(EthercatFfiError::MasterReleased)
        }
    }
}

fn check_code(function: &'static str, code: c_int) -> Result<()> {
    if code == 0 {
        Ok(())
    } else {
        Err(EthercatFfiError::CallFailed {
            function,
            code: code as i32,
        })
    }
}

fn bytes_for_entry(bit_length: Option<u8>) -> usize {
    let bits = usize::from(bit_length.unwrap_or(8).max(1));
    bits.div_ceil(8)
}
