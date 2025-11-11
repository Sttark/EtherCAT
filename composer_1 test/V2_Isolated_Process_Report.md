# V2 Isolated Process Implementation: Detailed Architecture Report

## Overview
This document provides a comprehensive analysis of the V2 EtherCAT driver's isolated process architecture. The V2 design separates the real-time EtherCAT cyclic task into a separate process, with the application layer communicating via command/status queues. This ensures real-time guarantees while allowing the application to remain non-realtime.

---

## 1. Architecture Overview

### Process Separation
```
┌─────────────────────────────────────────────────────────────┐
│ Application Process (Non-RT)                                │
│  - User code, business logic                                │
│  - Async/await, blocking operations OK                      │
│  - Communicates via queues                                  │
└─────────────────┬───────────────────────────────────────────┘
                  │ Command Queue (mp.Queue)
                  │ Status Queue (mp.Queue)
                  ▼
┌─────────────────────────────────────────────────────────────┐
│ EtherCAT Process (RT-capable)                               │
│  - Owns EtherCAT master handle                              │
│  - Cyclic PDO exchange (5ms cycle)                          │
│  - Command processing                                       │
│  - Status publishing                                        │
└─────────────────────────────────────────────────────────────┘
```

**Key Benefits:**
1. **Real-time isolation:** EtherCAT process can run with RT priority without affecting application
2. **Fault isolation:** EtherCAT crashes don't crash application
3. **Clean separation:** Application doesn't need to know about EtherCAT internals
4. **Non-blocking API:** Application methods return immediately (queue commands)

---

## 2. Process Manager: `EtherCATProcessManager`

### Initialization
```python
class EtherCATProcessManager:
    def __init__(self, cfg: EthercatNetworkConfig):
        self.cfg = cfg
        self._cmd_q: mp.Queue = mp.Queue(maxsize=1024)
        self._status_q: mp.Queue = mp.Queue(maxsize=64)
        self._proc: Optional[mp.Process] = None
```

**Critical Details:**

1. **Queue Sizes:**
   - Command queue: 1024 entries (large buffer for burst commands)
   - Status queue: 64 entries (smaller, status is published frequently)
   - Both use `mp.Queue` (multiprocessing queue, thread-safe)

2. **Queue Behavior:**
   - `put_nowait()`: Non-blocking, raises `queue.Full` if full
   - `get_nowait()`: Non-blocking, raises `queue.Empty` if empty
   - Application must handle queue full/empty conditions

3. **Process Lifecycle:**
   - Process created on `start()`
   - Process terminated on `stop()`
   - Process is daemon (terminates when parent dies)

### Start Method
```python
def start(self):
    if self._proc and self._proc.is_alive():
        return
    target = EtherCATProcess(self.cfg, self._cmd_q, self._status_q)
    self._proc = mp.Process(target=target.run, daemon=True)
    self._proc.start()
```

**Details:**
- Checks if process already running (idempotent)
- Creates `EtherCATProcess` instance (NOT in separate process yet)
- Spawns new process with `target.run` as entry point
- Process is daemon (auto-terminates on parent exit)

**Critical:**
- Process starts immediately (no blocking wait)
- Application must wait for initialization (check status queue)
- Process may take several seconds to initialize (DC sync, etc.)

### Stop Method
```python
def stop(self):
    if self._proc and self._proc.is_alive():
        self._proc.terminate()
        self._proc.join(timeout=2.0)
        self._proc = None
```

**Details:**
- Sends SIGTERM to process (graceful shutdown)
- Waits up to 2 seconds for process to exit
- Sets `_proc = None` after termination

**Critical:**
- `terminate()` sends SIGTERM (can be caught by process)
- Process should cleanup (deactivate master, release handles) in `finally` block
- If process doesn't exit in 2 seconds, it's killed (no cleanup)

---

## 3. EtherCAT Process: `EtherCATProcess`

### Initialization
```python
class EtherCATProcess:
    def __init__(self, cfg: EthercatNetworkConfig, cmd_q: mp.Queue, status_q: mp.Queue):
        self.cfg = cfg
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.adapter = IGHMasterAdapter(master_index=cfg.master_index)
        self.domain = None
        self.slave_handles: Dict[int, Any] = {}
        self.offsets: Dict[int, Dict[Tuple[int, int], int]] = {}
        self.features: Dict[int, Dict[str, Any]] = {}
        self.last_mode_cmd: Dict[int, Optional[int]] = {}
        self.last_velocity_cmd: Dict[int, Optional[float]] = {}
        self.last_position_cmd: Dict[int, Optional[float]] = {}
        self.warned_missing_pdo: Dict[int, set] = {}
        self.last_probe_arm: Dict[int, Optional[int]] = {}
        self.pdo_maps: Dict[int, Dict[str, Dict[int, list]]] = {}
```

**State Management:**

1. **Command Intent Tracking:**
   - `last_mode_cmd`: Last mode command per slave (PP/PV/CSP)
   - `last_velocity_cmd`: Last velocity command per slave
   - `last_position_cmd`: Last position command per slave
   - `last_probe_arm`: Last probe arm command per slave

2. **PDO Offset Tracking:**
   - `offsets[slave_pos][(index, subindex)]` → byte offset in domain
   - Used for fast PDO read/write

3. **Feature Tracking:**
   - `features[slave_pos]`: Device capabilities (from XML)
   - Used for capability checks

4. **PDO Map Tracking:**
   - `pdo_maps[slave_pos]`: RX/TX PDO mappings
   - Used for PDO health reporting

---

## 4. Setup Phase: `_setup()`

### Master Request
```python
if not self.adapter.request(sdo_only=self.cfg.sdo_only):
    logger.error("Failed to request master")
    return False
```

**Details:**
- Requests master handle (SDO-only or full PDO access)
- Must succeed before any other operations
- Returns False on failure (process will exit)

### Domain Creation
```python
if not self.cfg.sdo_only:
    self.domain = self.adapter.create_domain()
```

**Details:**
- Domain created only if PDO is enabled
- Domain handle stored for later use
- Domain used for PDO data access

### Slave Configuration
```python
for dcfg in self.cfg.slaves:
    s = self.adapter.config_slave(dcfg.alias, dcfg.position, dcfg.vendor_id, dcfg.product_code)
    
    # Build PDO config from XML + overrides
    features = parse_esi_features(dcfg.xml.xml_file)
    rx_pdos = dcfg.pdo.rx_pdos if dcfg.pdo and dcfg.pdo.rx_pdos else features['rx_pdos']
    tx_pdos = dcfg.pdo.tx_pdos if dcfg.pdo and dcfg.pdo.tx_pdos else features['tx_pdos']
```

**Critical Details:**

1. **XML Parsing:**
   - XML file parsed to extract PDO mappings and features
   - Custom PDO config can override XML defaults
   - Features extracted for capability checks

2. **PDO Mapping:**
   - RX PDOs: Master-to-slave (controlword, targets)
   - TX PDOs: Slave-to-master (statusword, feedback)
   - Custom PDO config allows adding entries not in XML

3. **Sync Manager Configuration:**
   ```python
   sync_configs = []
   if rx_pdos:
       sync_configs.append((2, 0, [(p, rx_pdo_map.get(p, [])) for p in rx_pdos]))
   if tx_pdos:
       sync_configs.append((3, 1, [(p, tx_pdo_map.get(p, [])) for p in tx_pdos]))
   ```
   - Sync manager 2: RX (direction 0)
   - Sync manager 3: TX (direction 1)
   - Each PDO has list of entries (index, subindex, bit_length)

4. **PDO Registration:**
   ```python
   register_list = list(set(register_pairs))
   offsets = self.adapter.register_pdo_entry_list(
       self.domain, dcfg.alias, dcfg.position, dcfg.vendor_id, dcfg.product_code, register_list
   )
   ```
   - All PDO entries registered to domain
   - Returns offset map: `(index, subindex) → byte_offset`
   - Offsets used for fast PDO access

### Master Activation
```python
if not self.cfg.sdo_only:
    self.adapter.activate()
```

**Details:**
- Master activated AFTER all configuration
- Domain data becomes valid after activation
- Process can now start cyclic operation

---

## 5. Command Handling: `_handle_command()`

### Command Processing
```python
def _handle_command(self, cmd: Command):
    if cmd.type == CommandType.SET_VELOCITY_MODE:
        self.last_mode_cmd[cmd.target_id] = MODE_PV
    elif cmd.type == CommandType.SET_POSITION_MODE:
        self.last_mode_cmd[cmd.target_id] = MODE_PP
    elif cmd.type == CommandType.SET_CSP_MODE:
        self.last_mode_cmd[cmd.target_id] = MODE_CSP
    elif cmd.type == CommandType.SET_VELOCITY:
        self.last_velocity_cmd[cmd.target_id] = float(cmd.value or 0.0)
    elif cmd.type in (CommandType.SET_POSITION, CommandType.SET_POSITION_CSP):
        self.last_position_cmd[cmd.target_id] = float(cmd.value or 0.0)
    elif cmd.type == CommandType.ARM_PROBE:
        probe_value = cmd.params.get('probe_function') if cmd.params else None
        if probe_value is not None:
            self.last_probe_arm[cmd.target_id] = int(probe_value)
```

**Critical Design:**

1. **Non-Blocking Command Processing:**
   - Commands are stored as "intent" (last_mode_cmd, etc.)
   - No immediate action taken
   - Cyclic task realizes intent via PDO/SDO writes

2. **Intent Persistence:**
   - Intent persists until new command overwrites it
   - Allows cyclic task to maintain state (e.g., keep velocity at last commanded value)

3. **Probe Arm Write-Once:**
   ```python
   elif cmd.type == CommandType.DISABLE_PROBE:
       self.last_probe_arm[cmd.target_id] = 0
   ```
   - Probe arm command cleared after one write (see `_cyclic_write()`)
   - Prevents re-arming probe every cycle

---

## 6. Cyclic Write: `_cyclic_write()`

### Mode Maintenance
```python
mode = self.last_mode_cmd.get(slave_pos)
if mode is not None:
    if (MODES_OP_INDEX, 0) in entries:
        self.adapter.write_domain(self.domain, entries[(MODES_OP_INDEX, 0)], bytes([mode]))
    else:
        # Fallback to SDO
        self.adapter.sdo_download(slave_pos, MODES_OP_INDEX, 0, bytes([mode]))
```

**Critical Details:**

1. **PDO vs SDO Fallback:**
   - If mode in PDO: Write via PDO (fast, every cycle)
   - If mode NOT in PDO: Write via SDO (slow, but works)
   - Warning logged once per missing PDO entry

2. **Mode Persistence:**
   - Mode written every cycle while `last_mode_cmd` is set
   - Ensures mode doesn't change unexpectedly
   - Some drives require mode to be maintained

### Velocity Maintenance
```python
vel = self.last_velocity_cmd.get(slave_pos)
if vel is not None:
    v = int(vel)
    if (TARGET_VELOCITY_INDEX, 0) in entries:
        self.adapter.write_domain(self.domain, entries[(TARGET_VELOCITY_INDEX, 0)], 
                                  v.to_bytes(4, byteorder='little', signed=True))
```

**Details:**
- Velocity written every cycle (maintains commanded velocity)
- Converted to signed 32-bit integer
- Little-endian byte order (standard for EtherCAT)

### Position Maintenance
```python
pos = self.last_position_cmd.get(slave_pos)
if pos is not None:
    p = int(pos)
    if (TARGET_POSITION_INDEX, 0) in entries:
        self.adapter.write_domain(self.domain, entries[(TARGET_POSITION_INDEX, 0)], 
                                  p.to_bytes(4, byteorder='little', signed=True))
```

**Details:**
- Position written every cycle (for CSP mode, streams position)
- For PP mode, position written once (but maintained every cycle)
- Bit 4 (new set-point) pulsed when position changes (see controlword)

### Controlword Maintenance
```python
if (CW_INDEX, 0) in entries:
    cw_offset = entries[(CW_INDEX, 0)]
    cw = 0
    cw |= CW_ENABLE_OP_SIMPLIFIED
    if pos is not None:
        cw |= (1 << CW_BIT_NEW_SET_POINT)
    self.adapter.write_domain(self.domain, cw_offset, cw.to_bytes(2, byteorder='little'))
```

**Critical Details:**

1. **Enable Operation Bit:**
   - `CW_ENABLE_OP_SIMPLIFIED` maintains drive enabled state
   - Written every cycle (required for drive to stay enabled)

2. **New Set-Point Bit (Bit 4):**
   - Set when position command exists
   - Pulses bit 4 to trigger position move in PP mode
   - **Issue:** Bit 4 is set every cycle, not just when position changes
   - **V1 Workaround:** Cycle bit 4 (clear then set) when position stalls

3. **Controlword Format:**
   - 16-bit unsigned integer
   - Little-endian byte order

### Probe Arm Maintenance
```python
probe_val = self.last_probe_arm.get(slave_pos)
if probe_val is not None:
    if (PROBE_FUNCTION_INDEX, 0) in entries:
        self.adapter.write_domain(self.domain, entries[(PROBE_FUNCTION_INDEX, 0)], 
                                  probe_val.to_bytes(2, 'little'))
    # Clear after one write
    self.last_probe_arm[slave_pos] = None
```

**Critical Design:**

1. **Write-Once Behavior:**
   - Probe arm command written once, then cleared
   - Prevents re-arming probe every cycle
   - **V1 Requirement:** Must disable probe (0x0000) before re-arming

2. **V2 Implementation Gap:**
   - V2 doesn't implement disable-before-re-arm sequence
   - May cause probe arming failures (see V1 nuances document)

---

## 7. Cyclic Loop: `run()`

### Main Loop Structure
```python
cycle_time_s = self.cfg.cycle_time_ms / 1000.0
last_status = 0.0
last_cycle_start = time.time()
try:
    while True:
        # Pump commands
        for _ in range(16):
            try:
                cmd = self.cmd_q.get_nowait()
                self._handle_command(cmd)
            except queue.Empty:
                break

        # Cyclic PDO exchange
        if not self.cfg.sdo_only and self.domain is not None:
            self.adapter.set_application_time(int(time.time() * 1_000_000_000))
            self.adapter.receive()
            self.adapter.process_domain(self.domain)
            self._cyclic_write()
            self.adapter.queue_domain(self.domain)
            self.adapter.send()

        # Periodic status publish
        now = time.time()
        if now - last_status > 0.05:
            self._publish_status()
            last_status = now
        
        time.sleep(cycle_time_s)
finally:
    self._teardown()
```

**Critical Details:**

1. **Command Pumping:**
   - Processes up to 16 commands per cycle (prevents command queue backlog)
   - Non-blocking (`get_nowait()`)
   - Commands processed before PDO exchange

2. **PDO Exchange Sequence:**
   ```
   1. Set application time (for DC sync)
   2. Receive (read PDO data from slaves)
   3. Process domain (update working counters)
   4. Cyclic write (apply command intents)
   5. Queue domain (prepare for send)
   6. Send (write PDO data to slaves)
   ```
   - Sequence is critical - must be in this order
   - Application time updated every cycle (for DC sync)

3. **Status Publishing:**
   - Published every 50ms (decoupled from cycle rate)
   - Prevents status queue overflow
   - Application can read latest status (drops old status)

4. **Cycle Timing:**
   - `time.sleep(cycle_time_s)` maintains cycle time
   - **Issue:** `time.sleep()` is not real-time accurate
   - **V1:** Uses RT thread with SCHED_FIFO priority
   - **V2:** Process should be run with RT priority externally

5. **Error Handling:**
   - `try/finally` ensures cleanup (`_teardown()`)
   - Process exits on exception (application must restart)

---

## 8. Status Publishing: `_publish_status()`

### Status Structure
```python
status = NetworkStatus(drives={})
status.timestamp_ns = int(time.time() * 1_000_000_000)
status.cycle_time_ms_config = self.cfg.cycle_time_ms
status.sdo_only = self.cfg.sdo_only
```

**Details:**
- Status published as `NetworkStatus` object
- Timestamp in nanoseconds
- Configuration included for debugging

### Drive Status Fields
```python
for slave_pos, entries in self.offsets.items():
    drive = {}
    if (SW_INDEX, 0) in entries:
        raw = self.adapter.read_domain(self.domain, entries[(SW_INDEX, 0)], 2) or b"\x00\x00"
        drive['statusword'] = int.from_bytes(raw, 'little')
    if (MODES_OP_DISPLAY_INDEX, 0) in entries:
        raw = self.adapter.read_domain(self.domain, entries[(MODES_OP_DISPLAY_INDEX, 0)], 1) or b"\x00"
        drive['mode_display'] = raw[0]
    if (POSITION_ACTUAL_INDEX, 0) in entries:
        raw = self.adapter.read_domain(self.domain, entries[(POSITION_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
        drive['position_actual'] = int.from_bytes(raw, 'little', signed=True)
    if (VELOCITY_ACTUAL_INDEX, 0) in entries:
        raw = self.adapter.read_domain(self.domain, entries[(VELOCITY_ACTUAL_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
        drive['velocity_actual'] = int.from_bytes(raw, 'little', signed=True)
```

**Critical Details:**

1. **PDO Read:**
   - Reads from domain using byte offset
   - Default values if PDO entry missing
   - Little-endian byte order
   - Signed integers for position/velocity

2. **Probe Status:**
   ```python
   if (PROBE_STATUS_INDEX, 0) in entries:
       raw = self.adapter.read_domain(self.domain, entries[(PROBE_STATUS_INDEX, 0)], 2) or b"\x00\x00"
       drive['probe_active'] = bool(int.from_bytes(raw, 'little') & 0x0001)
   if (PROBE_POS1_INDEX, 0) in entries:
       raw = self.adapter.read_domain(self.domain, entries[(PROBE_POS1_INDEX, 0)], 4) or b"\x00\x00\x00\x00"
       drive['probe_pos1'] = int.from_bytes(raw, 'little', signed=True)
   ```
   - Probe status: Bit 0 = active
   - Probe positions: Signed 32-bit integers

3. **PDO Health:**
   ```python
   pdo_health = {}
   for key_idx in [MODES_OP_INDEX, MODES_OP_DISPLAY_INDEX, CW_INDEX, SW_INDEX, ...]:
       state = 'missing'
       if (key_idx, 0) in entries:
           state = 'pdo'
       pdo_health[f"0x{key_idx:04X}:0"] = state
   ```
   - Reports which objects are in PDO vs missing
   - Used for debugging and capability detection

### Status Queue Publishing
```python
try:
    self.status_q.put_nowait(status)
except queue.Full:
    pass
```

**Details:**
- Non-blocking put (drops if queue full)
- Application should read frequently to prevent queue full
- Latest status is most important (old status can be dropped)

---

## 9. Client API: `attach_drive_handle()`

### Drive Handle Creation
```python
def attach_drive_handle(manager: EtherCATProcessManager, slave_position: int) -> CiA402Drive:
    drive = CiA402Drive(slave_position=slave_position)
    status = StatusProxy(manager)
    
    def enqueue(pos: int, cmd_type: CommandType, value, params: Dict):
        manager.send_command(Command(target_id=pos, type=cmd_type, value=value, params=params))
    
    def read_status(pos: int, key: str):
        return status.get_field(pos, key)
    
    drive._enqueue_command = enqueue
    drive._read_status = read_status
    return drive
```

**Critical Design:**

1. **Method Injection:**
   - `_enqueue_command` and `_read_status` injected into drive object
   - Drive object doesn't know about process manager
   - Clean separation of concerns

2. **Status Proxy:**
   ```python
   class StatusProxy:
       def _refresh(self):
           now = time.time()
           if now - self._last_at > 0.02:
               latest = self._manager.get_latest_status()
               if latest is not None:
                   self._last = latest
                   self._last_at = now
   ```
   - Throttles status refresh (20ms minimum)
   - Caches latest status
   - Prevents hammering status queue

3. **Non-Blocking API:**
   - All drive methods return immediately
   - Commands queued, not executed synchronously
   - Status read from cache (may be slightly stale)

---

## 10. V1 vs V2 Differences

### Architecture
- **V1:** Direct driver in same process as application
- **V2:** Isolated process, communication via queues

### Command Execution
- **V1:** Commands execute immediately (blocking SDO writes)
- **V2:** Commands queued, executed in cyclic task (non-blocking)

### Status Reading
- **V1:** Direct PDO read via network object
- **V2:** Status read from queue (cached, may be stale)

### Mode Switching
- **V1:** SDO write + verification polling
- **V2:** Command queued, mode maintained every cycle

### Error Handling
- **V1:** Exceptions propagate to application
- **V2:** Errors logged in process, application must check status

### Real-Time Guarantees
- **V1:** Application must manage RT thread
- **V2:** Process can be run with RT priority externally

---

## 11. Implementation Gaps and Requirements

### Missing V1 Features

1. **Probe Disable-Before-Re-Arm:**
   - V1 requires writing 0x0000 before re-arming probe
   - V2 doesn't implement this sequence
   - **Required:** Add disable step before probe arm

2. **Mode Verification:**
   - V1 polls mode display until mode changes
   - V2 doesn't verify mode changes
   - **Required:** Add mode verification polling (non-blocking)

3. **Position Move Completion:**
   - V1 implements bit 4 cycling for stalled moves
   - V2 doesn't detect stalled moves
   - **Required:** Add position move completion detection

4. **Error Checking:**
   - V1 checks drive errors before operations
   - V2 doesn't check errors proactively
   - **Required:** Add error checking in cyclic task

5. **Unit Conversion:**
   - V1 converts user units to pulses
   - V2 doesn't implement unit conversion
   - **Required:** Add unit conversion layer

6. **Homing:**
   - V1 implements homing sequence
   - V2 queues homing command but doesn't implement sequence
   - **Required:** Implement homing state machine

7. **DC Synchronization:**
   - V1 sets application time before activation
   - V2 sets application time every cycle (correct)
   - **Status:** Implemented correctly

8. **NIC IRQ Configuration:**
   - V1 configures NIC IRQ threads
   - V2 doesn't configure IRQ threads
   - **Required:** Add IRQ thread configuration (or document external requirement)

---

## 12. Testing Recommendations

1. **Command Queue Overflow:**
   - Test behavior when command queue is full
   - Verify commands are not lost

2. **Status Queue Overflow:**
   - Test behavior when status queue is full
   - Verify latest status is preserved

3. **Process Crash:**
   - Test behavior when EtherCAT process crashes
   - Verify application can detect and restart

4. **Mode Switching:**
   - Test mode switches with verification
   - Verify mode is maintained every cycle

5. **Probe Arming:**
   - Test probe disable-before-re-arm sequence
   - Verify probe arms correctly

6. **Position Moves:**
   - Test position move completion detection
   - Verify bit 4 cycling for stalled moves

7. **Real-Time Performance:**
   - Test cycle time jitter
   - Verify RT priority works correctly

---

## Summary

The V2 isolated process architecture provides clean separation between application and EtherCAT driver, but requires additional features to match V1 functionality:

1. **Probe disable-before-re-arm sequence**
2. **Mode verification polling**
3. **Position move completion detection**
4. **Error checking**
5. **Unit conversion**
6. **Homing implementation**
7. **IRQ thread configuration**

All of these features must be implemented cleanly in V2 to ensure reliable operation matching V1 behavior.

