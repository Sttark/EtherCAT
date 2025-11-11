# EtherCAT Master Adapter: Detailed Type and Handle Analysis

## Overview
This document provides a microscopic analysis of the EtherCAT master adapter component, which bridges C types and handles from the IgH EtherCAT library to Python. Every type discrepancy, handle management detail, and potential pitfall is documented.

---

## 1. Library Loading and Initialization

### Library Path
```python
_libec = ctypes.CDLL("libethercat.so.1")
```
**Critical Details:**
- Library name: `libethercat.so.1` (versioned .so file)
- Loaded via `ctypes.CDLL()` - uses dlopen() under the hood
- If library not found, `_libec = None` and all operations will fail
- No explicit error handling for missing library - operations will raise `OSError`

**V1 vs V2:**
- V1: Uses pyethercat wrapper (imports from SttarkStandardLibrary)
- V2: Direct ctypes binding OR uses IGHMasterAdapter which wraps pyethercat
- V2 has BOTH implementations: standalone Master class (direct ctypes) AND IGHMasterAdapter (wraps pyethercat)

---

## 2. Opaque Handle Structures

### Master Handle: `ec_master_t`
```python
class ec_master_t(ctypes.Structure):
    """Opaque master handle."""
    pass
```
**Type Details:**
- Empty structure - fields are opaque (not exposed to Python)
- C library manages internal state
- Python only holds pointer to structure
- Return type: `ctypes.POINTER(ec_master_t)`

**Critical:**
- Handle is a POINTER, not the structure itself
- Must check for NULL pointer (`if not handle:`) before use
- Handle lifetime managed by C library (request/release)

### Slave Config Handle: `ec_slave_config_t`
```python
class ec_slave_config_t(ctypes.Structure):
    """Opaque slave configuration handle."""
    pass
```
**Type Details:**
- Opaque handle returned by `ecrt_master_slave_config()`
- Return type: `ctypes.POINTER(ec_slave_config_t)`
- Stored in `SlaveConfig._config_handle` for later use

**Critical:**
- Handle must be stored and reused for PDO configuration
- Handle becomes invalid after master release
- NULL check required before use

### Domain Handle: `ec_domain_t`
```python
class ec_domain_t(ctypes.Structure):
    """Opaque domain handle."""
    pass
```
**Type Details:**
- Opaque handle returned by `ecrt_master_create_domain()`
- Return type: `ctypes.POINTER(ec_domain_t)`
- Used for PDO data access and processing

**Critical:**
- Domain must be created AFTER master request but BEFORE activation
- Domain data pointer valid only after activation
- Domain lifetime tied to master

---

## 3. PDO Entry Registration Structure

### `ec_pdo_entry_reg_t`
```python
class ec_pdo_entry_reg_t(ctypes.Structure):
    _fields_ = [
        ("alias", ctypes.c_uint16),        # Slave alias
        ("position", ctypes.c_uint16),     # Slave position
        ("vendor_id", ctypes.c_uint32),    # Vendor ID
        ("product_code", ctypes.c_uint32), # Product code
        ("index", ctypes.c_uint16),        # Object index
        ("subindex", ctypes.c_uint8),      # Object subindex
        ("offset", ctypes.POINTER(ctypes.c_uint)),      # OUTPUT: byte offset
        ("bit_position", ctypes.POINTER(ctypes.c_uint)), # OUTPUT: bit position
    ]
```

**Critical Type Details:**

1. **`offset` and `bit_position` are OUTPUT parameters:**
   - Must be `ctypes.POINTER(ctypes.c_uint)` - library writes to these
   - Must create `ctypes.c_uint()` variables and pass `ctypes.byref()` or `ctypes.pointer()`
   - Values are filled by library during registration

2. **Array Termination:**
   - Array must be NULL-terminated (index = 0xFFFF or special sentinel)
   - V2 implementation uses array size `(len(entries) + 1)` with sentinel at end
   - Sentinel: `index=0xFF, subindex=0xFF` or similar

3. **Memory Management:**
   - Array must persist until registration completes
   - Variables for offset/bit_position must persist until values are read
   - Example:
     ```python
     offset_var = ctypes.c_uint()
     bit_pos_var = ctypes.c_uint()
     reg.offset = ctypes.pointer(offset_var)  # NOT byref() - needs pointer
     reg.bit_position = ctypes.pointer(bit_pos_var)
     # After registration:
     actual_offset = offset_var.value  # Read the value
     ```

**V1 vs V2 Discrepancy:**
- V1: Uses `ctypes.byref()` in some places
- V2: Uses `ctypes.pointer()` - CORRECT (pointer persists)
- Critical: `byref()` creates temporary reference, `pointer()` creates persistent pointer

---

## 4. PDO Configuration Structures

### `ec_pdo_entry_info_t`
```python
class ec_pdo_entry_info_t(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint16),      # Object index
        ("subindex", ctypes.c_uint8),     # Object subindex
        ("bit_length", ctypes.c_uint8),  # Bit length (8, 16, 32)
    ]
```

**Type Details:**
- Used in arrays for PDO configuration
- Array must be cast to `ctypes.POINTER(ec_pdo_entry_info_t)` for C function
- Array lifetime must persist until `ecrt_slave_config_pdos()` completes

### `ec_pdo_info_t`
```python
class ec_pdo_info_t(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint16),                    # PDO index (e.g., 0x1600)
        ("n_entries", ctypes.c_uint),                  # Number of entries
        ("entries", ctypes.POINTER(ec_pdo_entry_info_t)), # Pointer to entry array
    ]
```

**CRITICAL TYPE DISCREPANCY:**

**`n_entries` is `ctypes.c_uint`, NOT `ctypes.c_uint8`!**

**Historical Bug:**
- Original implementation may have used `c_uint8` (max 255 entries)
- Correct type is `c_uint` (platform-dependent, typically 32-bit)
- This bug caused `ecrt_slave_config_pdos()` to fail silently or corrupt memory

**V2 Implementation:**
```python
pdo_info.n_entries = len(entries)  # Python int -> c_uint (correct)
```

**Memory Layout:**
- `entries` pointer must point to valid array
- Array must be cast: `ctypes.cast(entry_array, ctypes.POINTER(ec_pdo_entry_info_t))`
- Array must persist until function returns

### `ec_sync_info_t`
```python
class ec_sync_info_t(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint8),                    # Sync manager index (0-3)
        ("dir", ctypes.c_int),                        # Direction (0=RX, 1=TX)
        ("n_pdos", ctypes.c_uint),                    # Number of PDOs
        ("pdos", ctypes.POINTER(ec_pdo_info_t)),      # Pointer to PDO array
        ("watchdog_mode", ctypes.c_int),              # Watchdog mode
    ]
```

**Type Details:**
- `index`: Sync manager index (typically 2 for RX, 3 for TX)
- `dir`: 0 = master-to-slave (RX), 1 = slave-to-master (TX)
- `n_pdos`: Number of PDOs in this sync manager
- Array must be NULL-terminated (sentinel with `index=0xFF`)

**Array Termination:**
```python
sync_array = (ec_sync_info_t * (len(sync_infos) + 1))()
# ... fill array ...
sync_array[len(sync_infos)].index = 0xFF  # Sentinel
sync_array[len(sync_infos)].dir = 0
sync_array[len(sync_infos)].n_pdos = 0
sync_array[len(sync_infos)].pdos = None
```

---

## 5. Master Info Structures

### `ec_master_info_t`
```python
class ec_master_info_t(ctypes.Structure):
    _fields_ = [
        ("slave_count", ctypes.c_uint),
        ("link_up", ctypes.c_int),      # Boolean-like (0/1)
        ("scan_busy", ctypes.c_int),    # Boolean-like (0/1)
        ("app_time", ctypes.c_uint64),  # Application time (nanoseconds)
    ]
```

**Type Details:**
- `link_up`: C int (0 = down, non-zero = up) - NOT Python bool
- `scan_busy`: C int (0 = idle, non-zero = busy) - NOT Python bool
- `app_time`: 64-bit unsigned integer (nanoseconds since epoch)

**Usage:**
```python
info = ec_master_info_t()
result = _libec.ecrt_master(self._master_handle, ctypes.byref(info))
if result != 0:
    raise MasterException(f"Failed: {result}")
slave_count = info.slave_count  # c_uint -> Python int
```

### `ec_slave_info_t`
```python
class ec_slave_info_t(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_uint16),
        ("vendor_id", ctypes.c_uint32),
        ("product_code", ctypes.c_uint32),
        ("revision_number", ctypes.c_uint32),
        ("serial_number", ctypes.c_uint32),
        ("alias", ctypes.c_uint16),
        ("current_on_ebus", ctypes.c_int16),  # Signed! Can be negative
        ("al_state", ctypes.c_uint8),         # Application layer state
        ("error_flag", ctypes.c_uint8),       # Error flag (0/1)
        ("sync_count", ctypes.c_uint8),
        ("sdo_count", ctypes.c_uint16),
        ("name", ctypes.c_char * 200),        # Fixed-size char array
    ]
```

**Critical Type Details:**

1. **`current_on_ebus` is SIGNED (`c_int16`):**
   - Can be negative (current draw direction)
   - Must handle negative values in Python

2. **`name` is fixed-size char array:**
   - Size: 200 bytes (not null-terminated guaranteed)
   - Must decode: `info.name.decode('utf-8')` or `info.name.decode('utf-8', errors='ignore')`
   - May contain null bytes - use `split('\x00')[0]` or similar

3. **`error_flag` is `c_uint8`, NOT bool:**
   - Convert: `bool(info.error_flag)` or `info.error_flag != 0`

---

## 6. Function Prototypes: Return Types and Parameters

### Master Request Functions

#### `ecrt_open_master`
```python
_libec.ecrt_open_master.argtypes = [ctypes.c_uint]
_libec.ecrt_open_master.restype = ctypes.POINTER(ec_master_t)
```
**Details:**
- Input: `master_index` (unsigned int, typically 0)
- Output: Pointer to master handle (NULL on failure)
- Used for SDO-only access (no PDO)

#### `ecrt_request_master`
```python
_libec.ecrt_request_master.argtypes = [ctypes.c_uint]
_libec.ecrt_request_master.restype = ctypes.POINTER(ec_master_t)
```
**Details:**
- Input: `master_index` (unsigned int)
- Output: Pointer to master handle (NULL on failure)
- Used for full PDO access
- **Critical:** Must check for NULL pointer return

### Master Activation

#### `ecrt_master_activate`
```python
_libec.ecrt_master_activate.argtypes = [ctypes.POINTER(ec_master_t)]
_libec.ecrt_master_activate.restype = ctypes.c_int
```
**Details:**
- Input: Master handle pointer
- Output: Error code (0 = success, non-zero = error)
- **Critical:** Must check return value, not just truthiness
- Must be called AFTER all slave/PDO configuration

### Application Time

#### `ecrt_master_application_time`
```python
_libec.ecrt_master_application_time.argtypes = [ctypes.POINTER(ec_master_t), ctypes.c_uint64]
_libec.ecrt_master_application_time.restype = None
```
**Details:**
- Input: Master handle, time in nanoseconds (64-bit unsigned)
- Output: None (void function)
- **Critical:** Time must be in nanoseconds (not milliseconds or seconds)
- Must be called BEFORE activation for DC sync
- Should be updated periodically during cyclic operation

**V2 Implementation:**
```python
time_ns = int(time.time() * 1_000_000_000)  # Convert to nanoseconds
self.adapter.set_application_time(time_ns)
```

### SDO Operations

#### `ecrt_master_sdo_download`
```python
_libec.ecrt_master_sdo_download.argtypes = [
    ctypes.POINTER(ec_master_t),
    ctypes.c_uint16,           # slave_position
    ctypes.c_uint16,           # index
    ctypes.c_uint8,            # subindex
    ctypes.POINTER(ctypes.c_uint8),  # data buffer
    ctypes.c_size_t,           # data size
    ctypes.POINTER(ctypes.c_uint32)  # OUTPUT: abort_code
]
_libec.ecrt_master_sdo_download.restype = ctypes.c_int
```

**Critical Type Details:**

1. **Data buffer is `POINTER(c_uint8)`, not bytes:**
   - Must create array: `(ctypes.c_uint8 * len(data))(*data)`
   - Or: `ctypes.create_string_buffer(data)`

2. **Abort code is OUTPUT parameter:**
   - Must create: `abort_code = ctypes.c_uint32(0)`
   - Pass: `ctypes.byref(abort_code)`
   - Read: `abort_code.value` after call

3. **Return value:**
   - 0 = success
   - Non-zero = error (check abort_code for details)

**V2 Implementation:**
```python
abort_code = ctypes.c_uint32(0)
data_array = (ctypes.c_uint8 * len(data))(*data)
result = _libec.ecrt_master_sdo_download(
    self._master_handle,
    slave_position,
    index,
    subindex,
    data_array,
    len(data),
    ctypes.byref(abort_code)
)
if result != 0:
    raise SDOException(f"abort_code=0x{abort_code.value:08X}")
```

#### `ecrt_master_sdo_upload`
```python
_libec.ecrt_master_sdo_upload.argtypes = [
    ctypes.POINTER(ec_master_t),
    ctypes.c_uint16,           # slave_position
    ctypes.c_uint16,           # index
    ctypes.c_uint8,            # subindex
    ctypes.POINTER(ctypes.c_uint8),  # OUTPUT: buffer
    ctypes.c_size_t,           # buffer size
    ctypes.POINTER(ctypes.c_size_t), # OUTPUT: actual size
    ctypes.POINTER(ctypes.c_uint32)  # OUTPUT: abort_code
]
_libec.ecrt_master_sdo_upload.restype = ctypes.c_int
```

**Critical Type Details:**

1. **Buffer is OUTPUT parameter:**
   - Must pre-allocate: `buffer = (ctypes.c_uint8 * max_size)()`
   - Library writes to buffer

2. **Actual size is OUTPUT:**
   - Must create: `result_size = ctypes.c_size_t(0)`
   - Pass: `ctypes.byref(result_size)`
   - Read: `result_size.value` after call

3. **Return value:**
   - 0 = success
   - Non-zero = error

**V2 Implementation:**
```python
buffer = (ctypes.c_uint8 * max_size)()
result_size = ctypes.c_size_t(0)
abort_code = ctypes.c_uint32(0)
result = _libec.ecrt_master_sdo_upload(
    self._master_handle,
    slave_position,
    index,
    subindex,
    buffer,
    max_size,
    ctypes.byref(result_size),
    ctypes.byref(abort_code)
)
if result != 0:
    raise SDOException(f"abort_code=0x{abort_code.value:08X}")
return bytes(buffer[:result_size.value])
```

### Domain Operations

#### `ecrt_domain_data`
```python
_libec.ecrt_domain_data.argtypes = [ctypes.POINTER(ec_domain_t)]
_libec.ecrt_domain_data.restype = ctypes.POINTER(ctypes.c_uint8)
```
**Details:**
- Returns pointer to domain data buffer
- **Critical:** Valid only AFTER master activation
- Pointer points to shared memory (managed by library)
- Can be used for direct memory access (dangerous but fast)

**V2 Usage:**
```python
domain_ptr = self.adapter.get_domain_data(domain)
if domain_ptr:
    # Access via pointer arithmetic (dangerous)
    # Or use read_domain/write_domain wrapper methods
```

#### `ecrt_domain_process`
```python
_libec.ecrt_domain_process.argtypes = [ctypes.POINTER(ec_domain_t)]
_libec.ecrt_domain_process.restype = ctypes.c_int
```
**Details:**
- Processes received PDO data
- Must be called every cycle AFTER `receive()`
- Return value: Number of working counter violations (0 = OK)

### PDO Configuration

#### `ecrt_slave_config_pdos`
```python
_libec.ecrt_slave_config_pdos.argtypes = [
    ctypes.POINTER(ec_slave_config_t),
    ctypes.c_uint,                      # sync_count
    ctypes.POINTER(ec_sync_info_t)      # sync_infos array
]
_libec.ecrt_slave_config_pdos.restype = ctypes.c_int
```

**Critical Type Details:**

1. **`sync_count` is `c_uint`, NOT `c_uint8`:**
   - Can be > 255 sync managers
   - Must match array size (minus sentinel)

2. **Array must be NULL-terminated:**
   - Sentinel: `index=0xFF, dir=0, n_pdos=0, pdos=None`

3. **Memory management:**
   - All nested arrays (PDO info, entry info) must persist
   - Arrays are stored in `slave_config._sync_array` and `_sync_infos` to prevent GC

**V2 Implementation:**
```python
sync_array = (ec_sync_info_t * (len(sync_infos) + 1))()
# ... fill array ...
sync_array[len(sync_infos)].index = 0xFF  # Sentinel
result = _libec.ecrt_slave_config_pdos(
    slave_config._config_handle,
    len(sync_infos),  # c_uint (correct)
    ctypes.cast(sync_array, ctypes.POINTER(ec_sync_info_t))
)
# Store arrays to prevent GC
slave_config._sync_array = sync_array
slave_config._sync_infos = sync_infos
```

---

## 7. IGHMasterAdapter: Wrapper Around pyethercat

### Architecture
```python
class IGHMasterAdapter(MasterAdapter):
    def __init__(self, master_index: int = 0):
        from pyethercat.master import Master
        self._cls_Master = Master
        self._master = self._cls_Master(master_index)
```

**Type Details:**
- Wraps `pyethercat.master.Master` class
- Delegates all operations to wrapped master
- Provides abstraction layer for v2 process manager

**Critical:**
- `pyethercat` handles all ctypes details internally
- V2 can use either direct `Master` class OR `IGHMasterAdapter`
- `IGHMasterAdapter` is preferred for compatibility with existing code

**Method Mapping:**
- `read_domain()` → `master.read_domain_data()`
- `write_domain()` → `master.write_domain_data()`
- All other methods map 1:1

---

## 8. Summary: Critical Type Discrepancies

1. **`ec_pdo_info_t.n_entries`: `c_uint`, NOT `c_uint8`**
   - Historical bug: Using `c_uint8` limits to 255 entries
   - Correct: `c_uint` (platform-dependent, typically 32-bit)

2. **`offset` and `bit_position` in `ec_pdo_entry_reg_t`: Use `pointer()`, NOT `byref()`**
   - `byref()` creates temporary reference
   - `pointer()` creates persistent pointer (required)

3. **SDO buffer types: `POINTER(c_uint8)`, not bytes**
   - Must create array: `(ctypes.c_uint8 * len(data))(*data)`

4. **Output parameters: Use `byref()` for simple types, `pointer()` for structure fields**
   - Simple types: `ctypes.byref(c_uint())`
   - Structure fields: `ctypes.pointer(c_uint())`

5. **Array termination: Must include sentinel**
   - PDO entry registration: `index=0xFFFF` or similar
   - Sync info array: `index=0xFF, dir=0, n_pdos=0, pdos=None`

6. **Memory management: Arrays must persist**
   - Store arrays in object attributes to prevent GC
   - Arrays used in C function calls must exist until function returns

7. **Return value checking: Use explicit comparison**
   - `if result != 0:` NOT `if result:`
   - Some functions return negative values on error

8. **Boolean-like fields: Use explicit comparison**
   - `bool(info.error_flag)` or `info.error_flag != 0`
   - NOT `if info.error_flag:` (may be non-zero but not 1)

---

## 9. Testing Recommendations

1. **Test with maximum PDO entries (>255)**
   - Verify `n_entries` as `c_uint` works correctly

2. **Test array termination**
   - Verify sentinel values are correct

3. **Test memory persistence**
   - Verify arrays don't get GC'd during C function calls

4. **Test NULL pointer handling**
   - Verify all handle checks work correctly

5. **Test error code propagation**
   - Verify abort codes are read correctly from OUTPUT parameters

6. **Test type conversions**
   - Verify Python int → c_uint conversions
   - Verify bytes → c_uint8 array conversions

---

**All type discrepancies documented above must be verified and tested in V2 implementation.**



