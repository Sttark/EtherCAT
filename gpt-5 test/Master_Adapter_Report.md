# EtherCAT Master Adapter: Microscopic Type/Handle Report (V1 vs V2)

This report documents every relevant C type, function signature, and handle behavior bridging IgH EtherCAT to Python in V1 and V2. It highlights minute discrepancies to ensure the adapter is correct, predictable, and maintainable.

Scope:
- V1: SttarkStandardLibrary pyethercat bindings (imported in legacy code)
- V2: Local ctypes-based `ethercat_v2.master_adapter.Master` plus optional wrapper `IGHMasterAdapter` that delegates to pyethercat

---

## Library loading
```python
_libec = ctypes.CDLL("libethercat.so.1")
```
- Must succeed at import time; otherwise adapter unusable.
- Keep versioned so name resolution is stable; log error and fail fast if missing.

---

## Opaque handles
- `ec_master_t`, `ec_slave_config_t`, `ec_domain_t`, `ec_sdo_request_t`
  - Defined as empty `ctypes.Structure` classes; only pointers are used.
  - All API functions accept/return `POINTER(type)`; NULL pointer checks are mandatory before dereference.

---

## PDO entry registration struct
```python
class ec_pdo_entry_reg_t(ctypes.Structure):
    _fields_ = [
        ("alias", c_uint16),
        ("position", c_uint16),
        ("vendor_id", c_uint32),
        ("product_code", c_uint32),
        ("index", c_uint16),
        ("subindex", c_uint8),
        ("offset", POINTER(c_uint)),       # OUTPUT
        ("bit_position", POINTER(c_uint)), # OUTPUT
    ]
```
Minute details:
- Output fields must be persistent pointers. Use `ctypes.pointer(c_uint())`, not `byref()`. `byref()` can create a temporary that may not outlive the call.
- Registration list must be sentinel-terminated when using list APIs (a zeroed trailing element is sufficient; V2 builds arrays of length N+1).
- Return code of `ecrt_domain_reg_pdo_entry_list()` must be explicitly checked for 0.

---

## PDO config structs
```python
class ec_pdo_entry_info_t(ctypes.Structure):
    _fields_ = [("index", c_uint16), ("subindex", c_uint8), ("bit_length", c_uint8)]

class ec_pdo_info_t(ctypes.Structure):
    _fields_ = [("index", c_uint16), ("n_entries", c_uint), ("entries", POINTER(ec_pdo_entry_info_t))]

class ec_sync_info_t(ctypes.Structure):
    _fields_ = [("index", c_uint8), ("dir", c_int), ("n_pdos", c_uint), ("pdos", POINTER(ec_pdo_info_t)), ("watchdog_mode", c_int)]
```
Critical discrepancy (historical bug fixed):
- `ec_pdo_info_t.n_entries` must be `c_uint` (NOT `c_uint8`). Using `c_uint8` can corrupt memory or fail silently with large mappings.
Array termination:
- `ec_sync_info_t` arrays must be null-terminated with a “0xFF” sentinel element.
Memory lifetime:
- Store nested arrays on the Python object (`_sync_array`, `_sync_infos`) to prevent garbage collection before the C call completes.

---

## Master info and slave info
```python
class ec_master_info_t(ctypes.Structure):
    _fields_ = [("slave_count", c_uint), ("link_up", c_int), ("scan_busy", c_int), ("app_time", c_uint64)]

class ec_slave_info_t(ctypes.Structure):
    _fields_ = [
        ("position", c_uint16),
        ("vendor_id", c_uint32),
        ("product_code", c_uint32),
        ("revision_number", c_uint32),
        ("serial_number", c_uint32),
        ("alias", c_uint16),
        ("current_on_ebus", c_int16),  # signed!
        ("al_state", c_uint8),
        ("error_flag", c_uint8),
        ("sync_count", c_uint8),
        ("sdo_count", c_uint16),
        ("name", c_char * 200),
    ]
```
Minute details:
- `current_on_ebus` is signed 16-bit.
- `name` is a fixed-size char array; decode and trim trailing nulls.
- Convert bool-like ints explicitly (`bool(info.error_flag)`), don’t rely on implicit truthiness.

---

## Function prototypes (selected)
- `ecrt_open_master(master_index: c_uint) -> POINTER(ec_master_t)`
- `ecrt_request_master(master_index: c_uint) -> POINTER(ec_master_t)`
- `ecrt_master_activate(ptr) -> c_int` (0 = OK)
- `ecrt_master_application_time(ptr, c_uint64)` (void)
- `ecrt_master(...) -> c_int` (fills `ec_master_info_t`)
- `ecrt_master_get_slave(ptr, c_uint16 position, POINTER(ec_slave_info_t)) -> c_int`
- `ecrt_master_sdo_download(..., POINTER(c_uint8) data, c_size_t size, POINTER(c_uint32) abort_code) -> c_int`
- `ecrt_master_sdo_upload(..., POINTER(c_uint8) out_buf, c_size_t buf_sz, POINTER(c_size_t) out_size, POINTER(c_uint32) abort_code) -> c_int`
- `ecrt_master_create_domain(ptr) -> POINTER(ec_domain_t)`
- `ecrt_domain_data(domain) -> POINTER(c_uint8)` (valid after activation)
- `ecrt_domain_process(domain) -> c_int`
- `ecrt_domain_queue(domain) -> void`
- `ecrt_domain_reg_pdo_entry_list(domain, POINTER(ec_pdo_entry_reg_t)) -> c_int`
- `ecrt_master_slave_config(master, alias, position, vendor, product) -> POINTER(ec_slave_config_t)`
- `ecrt_slave_config_pdos(slave_cfg, c_uint sync_count, POINTER(ec_sync_info_t)) -> c_int`
- `ecrt_slave_config_reg_pdo_entry(slave_cfg, index: c_uint16, subindex: c_uint8, domain, POINTER(c_uint) offset) -> c_int`
- `ecrt_slave_config_dc(slave_cfg, assign_activate: c_uint16, sync0_cycle_time_ns: c_uint32, sync0_shift_ns: c_int32, sync1_cycle_time_ns: c_uint32, sync1_shift_ns: c_int32) -> void`
- `ecrt_master_select_reference_clock(master, slave_cfg) -> void`

Minute checks:
- Always check `!= 0` for error; some IgH calls return negative error codes.
- Pass exact integer widths; `subindex` is `c_uint8`, not a wider type.

---

## Domain read/write helpers (V1 vs V2)
- V1 (pyethercat) exposes `read_domain_data(domain, offset, size) -> bytes` and `write_domain_data(domain, offset, bytes) -> bool`.
- V2 direct `Master` wrapper currently DOES NOT expose `read_domain`/`write_domain`, but `process_manager` calls those methods.
  - This is a functional mismatch. Fix by either:
    1) Switching `process_manager` to use `IGHMasterAdapter` (wrapper around pyethercat) which implements these methods, or
    2) Implementing `read_domain`/`write_domain` in the direct `Master` wrapper using `ecrt_domain_data()` pointer arithmetic with explicit bounds checking and bytearray slicing.

Recommendation:
- Add `read_domain(self, domain, offset, size)` and `write_domain(self, domain, offset, data)` to the direct wrapper, and cache the domain pointer acquired post-activation.

---

## Reference clock and DC timing
- `ecrt_master_application_time()` must be called before activation and periodically during cyclic operation.
- `ecrt_master_select_reference_clock()` is bound and available; evaluate selecting the AS715N (or another stable clock) explicitly for multi-slave setups.

---

## Minute discrepancies and resolutions
1) `ec_pdo_info_t.n_entries` width
   - Use `c_uint`. Verified in V2; treat as regression test case.
2) Pointer semantics in registration
   - Use `pointer(c_uint())` for `offset`/`bit_position` fields; avoid `byref()` temporaries.
3) Domain helpers absent in direct wrapper
   - Resolve mismatch vs. `process_manager` expectations (see above).
4) Sentinel elements
   - Ensure PDO and Sync arrays are created with a terminal sentinel element (`index=0xFF`) and that registration arrays have a zeroed terminator.
5) Signedness and endianness
   - Confirm position/velocity/probe latched values are read/written as little-endian with correct signedness (position/velocity are signed 32-bit in typical CiA 402 mappings).

---

## Test recommendations (adapter-level)
- Registration tests with >255 PDO entries to assert `c_uint` handling.
- Explicit sentinel validation tests for PDO/Sync arrays.
- Round-trip tests for SDO upload/download (including abort codes).
- Domain read/write fuzz tests at random offsets (within registered ranges).
- DC application time monotonicity and bounds tests.

---

The adapter layer is the foundation of reliable servo control. The above microscopic type rules eliminate entire classes of “it works but it’s flaky” failures and should be enforced with small, deterministic tests.


