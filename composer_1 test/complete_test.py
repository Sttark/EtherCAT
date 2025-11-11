#!/usr/bin/env python3
"""
COMPLETE TEST - AS715N Servo Drive using EtherCAT V2 Driver

This is a complete duplicate of test_mode_switching.py with all movement and arming features,
but using the ethercat_v2 isolated process driver instead of the v1 direct driver.

Tests dynamic mode switching between PP, PV, and CSP using the v2 driver.
Includes full probe functionality, homing, and bidirectional probe detection.
"""

import sys
import os
import time
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/tmp/v2_mode_switching_debug.log', mode='w'),
        logging.StreamHandler(sys.stdout)
    ]
)

# Set console to INFO, file keeps DEBUG
console = logging.StreamHandler(sys.stdout)
console.setLevel(logging.INFO)
console.setFormatter(logging.Formatter('%(message)s'))
logging.getLogger().handlers[1] = console

# Reduce noise
logging.getLogger('ethercat_v2').setLevel(logging.WARNING)

print("="*80)
print("COMPLETE TEST - AS715N Servo using EtherCAT V2 Driver")
print("="*80)
print()

# Add ethercat_v2 to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from ethercat_v2.config_schema import (
    EthercatNetworkConfig, DriveConfig, XmlConfig, 
    PdoSelection, HomingConfig, UnitConversion
)
from ethercat_v2.process_manager import EtherCATProcessManager
from ethercat_v2.client import attach_drive_handle
from ethercat_v2.constants import MODE_PP, MODE_PV, MODE_CSP, MODE_HM

print("="*80)
print("STEP 1: Configure network and drive...")
print("="*80)

# XML file for AS715N servo
servo_xml = "/home/sttark/Desktop/github/Core-Cutter/ECAT-config/STEPPERONLINE_A6_Servo_V0.04.xml"

# Define custom PDO mapping for AS715N (adds velocity + probe)
# NOTE: 0x6060/0x6061 (Modes of Operation) NOT in PDO - causes faults on AS715N!
as715n_custom_pdos = {
    'rx_entries': [
        (0x6040, 0, 16),  # Controlword
        (0x607A, 0, 32),  # Target Position
        (0x60FF, 0, 32),  # Target Velocity
        (0x60B8, 0, 16),  # Touch Probe Function
    ],
    'tx_entries': [
        (0x6041, 0, 16),  # Statusword
        (0x6064, 0, 32),  # Position Actual
        (0x606C, 0, 32),  # Velocity Actual
        (0x60B9, 0, 16),  # Touch Probe Status
        (0x60BA, 0, 32),  # Probe-1 Positive Edge
        (0x60BB, 0, 32),  # Probe-1 Negative Edge
        (0x60FD, 0, 32),  # Digital Inputs
        (0x603F, 0, 16),  # Error Code
    ]
}

# Create network configuration
cfg = EthercatNetworkConfig(
    master_index=0,
    cycle_time_ms=5.0,
    sdo_only=False,
    slaves=[
        DriveConfig(
            position=0,
            alias=0,
            vendor_id=None,  # Will be read from XML
            product_code=None,  # Will be read from XML
            enable_dc=True,  # Required for AS715N
            operation_mode=1,  # PP mode initially
            profile_velocity=500,  # 500 units/s = 5 in/s
            profile_acceleration=1000,  # 1000 units/s² = 10 in/s²
            max_velocity=500,  # Safety limit
            homing=HomingConfig(
                method=1,  # Method 1: Negative limit switch with index pulse
                search_vel=400,  # 400 units/s = 4.0 in/s
                zero_vel=50,  # 50 units/s = 0.5 in/s
                accel=1000,  # 1000 units/s² = 10 in/s²
                offset=99,  # 99 units = 0.99 inches
                unit='native'
            ),
            rotation_direction='reverse',  # Reverse motor direction
            inertia_ratio=3.0,  # 3:1 load/motor inertia
            position_limits=(0, 0, 'disable'),  # Disable software limits
            xml=XmlConfig(xml_file=servo_xml),
            pdo=PdoSelection(
                rx_pdos=[0x1600],
                tx_pdos=[0x1A00],
                custom_pdo_config=as715n_custom_pdos
            ),
            unit_conversion=UnitConversion(
                units_per_pulse=1.0,
                scale_factor=1040.42  # 1040.42 pulses = 1 user unit = 0.01 inch
            )
        )
    ]
)

print("✓ Configuration created")
print("  - Custom PDO mapping (velocity + probe)")
print("  - DC synchronization enabled")
print("  - Unit conversion: 1040.42 pulses = 1 user unit")
print("  - Homing: Method 1, 400 units/s search")
print("  - Rotation: REVERSED, Inertia: 3:1")
print()

print("="*80)
print("STEP 2: Start EtherCAT process manager...")
print("="*80)

# Create and start process manager
mgr = EtherCATProcessManager(cfg)
mgr.start()

# Wait for process to initialize
print("Waiting for EtherCAT process to initialize...")
time.sleep(2.0)

# Attach drive handle
drive = attach_drive_handle(mgr, 0)

print("✓ Drive handle attached")
print()

print("="*80)
print("STEP 3: Wait for drive to enable...")
print("="*80)

# Wait for drive to be enabled (v2 handles this automatically)
for sec in range(15):
    time.sleep(1)
    status = mgr.get_latest_status()
    if status and status.drives.get(0):
        drive_status = status.drives[0]
        statusword = drive_status.get('statusword', 0)
        enabled = bool(statusword & 0x000F) == 0x0007  # Operation Enabled state
        print(f"[{sec+1}s] Statusword: 0x{statusword:04X}, Enabled: {enabled}")
        if enabled:
            print("✅ DRIVE ENABLED!")
            break
else:
    print("⚠️  Drive may not be enabled yet, continuing...")

print()

print("="*80)
print("✅ DRIVE READY - FULL FUNCTIONALITY")
print("="*80)
print()
print("AS715N Servo: Custom PDO via v2 driver")
print("  ✓ DC synchronized (reference clock)")
print("  ✓ Target Velocity (0x60FF) in PDO - PV mode works!")
print("  ✓ Velocity Actual (0x606C) in PDO - real-time feedback")
print("  ✓ Touch Probe functionality (0x60B8/B9/BA/BB)")
print("  ✓ Full PP/PV/CSP mode switching")
print()

# Get initial position
initial_pos = drive.get_position() or 0.0
print(f"Initial position: {initial_pos:.2f} units ({initial_pos*0.01:.4f} inches)")
print()

# Read initial operation mode
status = mgr.get_latest_status()
mode = None
if status and status.drives.get(0):
    mode = status.drives[0].get('mode_display')
mode_names = {0: "NO_MODE", 1: "PP", 2: "VL", 3: "PV", 6: "HM", 8: "CSP", 9: "CSV", 10: "CST"}
print(f"Initial operation mode: {mode} ({mode_names.get(mode, 'UNKNOWN')})")
print()

print("="*80)
print("INTERACTIVE MODE SWITCHING TEST - AS715N SERVO")
print("="*80)
print()
print("Units: Position/velocity in 0.01 inch increments (1 = 0.01\", 100 = 1.00\")")
print("       Driver automatically converts to/from raw pulses (1040.42 pulses/unit)")
print()
print("Commands:")
print()
print("  === Mode Switching ===")
print("  pp [safe]       - Switch to PP (Profile Position) mode")
print("  pv [safe]       - Switch to PV (Profile Velocity) mode")
print("  csp [safe]      - Switch to CSP (Cyclic Synchronous Position) mode")
print()
print("  === Motion Commands (in 0.01 inch units) ===")
print("  pos <value>     - Move to position (PP mode) [e.g., 'pos 100' = 1.00 inch]")
print("  vel <value>     - Set velocity (PV mode) [e.g., 'vel 50' = 0.5 in/s]")
print("  csp_pos <value> - Set target position (CSP mode - real-time streaming)")
print("  stop            - Stop motor (set velocity to 0 in PV, hold in CSP)")
print("  home            - Execute homing (Method 1: neg limit, 4in/s search)")
print()
print("  === Touch Probe ===")
print("  probe arm rising   - Arm probe for rising edge (LOW→HIGH)")
print("  probe arm falling  - Arm probe for falling edge (HIGH→LOW)")
print("  probe status       - Show probe state and captured positions")
print("  probe disable      - Disable probe")
print()
print("  === Status ===")
print("  status / st     - Show current position, velocity, mode, and probe state")
print("  quit / q        - Exit")
print()
print("  Note: Add 'safe' to mode switch commands to disable operation before change")
print("  Note: CSP mode streams position every cycle - immediate response, no trajectory")
print()
print("="*80)
print()

def switch_to_pp_mode(drive, safe=False):
    """Switch drive to PP (Profile Position) mode - non-blocking"""
    safe_str = " (safe)" if safe else ""
    print(f"\n>>> Switching to PP (Profile Position) mode{safe_str}...")
    drive.set_position_mode(safe_switch=safe)
    print(f"✓ PP mode command queued")
    print(f"   (Verification will happen in background)")
    return True

def switch_to_pv_mode(drive, safe=False):
    """Switch drive to PV (Profile Velocity) mode - non-blocking"""
    safe_str = " (safe)" if safe else ""
    print(f"\n>>> Switching to PV (Profile Velocity) mode{safe_str}...")
    drive.set_velocity_mode(safe_switch=safe)
    print(f"✓ PV mode command queued")
    print(f"   (Verification will happen in background)")
    return True

def switch_to_csp_mode(drive, safe=False):
    """Switch drive to CSP (Cyclic Synchronous Position) mode - non-blocking"""
    safe_str = " (safe)" if safe else ""
    print(f"\n>>> Switching to CSP (Cyclic Synchronous Position) mode{safe_str}...")
    drive.set_csp_mode(safe_switch=safe)
    print(f"✓ CSP mode command queued")
    print(f"   Position will be streamed continuously every cycle")
    return True

def set_position_pp(drive, position):
    """Set position in PP mode - non-blocking"""
    pos_inches = position * 0.01
    print(f"Moving to position {position} units ({pos_inches:.4f} inches)...")
    drive.set_position_absolute(position)
    print(f"✓ Position command queued")
    return True

def set_velocity_pv(drive, velocity):
    """Set velocity in PV mode - non-blocking"""
    vel_inches_per_s = velocity * 0.01
    print(f"Setting velocity to {velocity} units/s ({vel_inches_per_s:.4f} in/s)...")
    drive.set_velocity(velocity)
    print(f"✓ Velocity command queued")
    return True

def set_position_csp(drive, position):
    """Set position in CSP mode - non-blocking"""
    pos_inches = position * 0.01
    print(f"Setting CSP position to {position} units ({pos_inches:.4f} inches)...")
    drive.set_position_csp(position)
    print(f"✓ CSP position command queued")
    return True

# Interactive control loop
try:
    while True:
        cmd = input("> ").strip().lower()
        
        if not cmd:
            continue
        
        if cmd in ["quit", "exit", "q"]:
            print("Exiting...")
            break
        
        elif cmd in ["status", "st"]:
            # Get latest status
            status = mgr.get_latest_status()
            if not status or not status.drives.get(0):
                print("⚠️  No status available")
                continue
            
            drive_status = status.drives[0]
            
            # Read position (in user units)
            pos_units = drive.get_position() or 0.0
            pos_inches = pos_units * 0.01
            
            # Read velocity (in user units)
            vel_units = drive.get_velocity() or 0.0
            vel_inches_per_s = vel_units * 0.01
            
            # Read mode
            mode = drive_status.get('mode_display')
            
            # Read status word
            statusword = drive_status.get('statusword', 0)
            
            print(f"\n{'='*60}")
            print(f"AS715N SERVO STATUS")
            print(f"{'='*60}")
            print(f"  Mode:     {mode_names.get(mode, 'UNKNOWN')} ({mode})")
            print(f"  Status:   0x{statusword:04X}")
            print(f"  Position: {pos_units:.2f} units = {pos_inches:.4f} inches")
            print(f"  Velocity: {vel_units:.2f} units/s = {vel_inches_per_s:.4f} in/s")
            
            # Read Digital Inputs
            di_value = drive_status.get('digital_inputs', 0)
            di_str = format(di_value, '032b')
            active_bits = [i for i in range(32) if (di_value >> i) & 0x01]
            
            print(f"  Digital Inputs (0x60FD): 0x{di_value:08X}")
            print(f"    Binary: {di_str[:16]} {di_str[16:]}")
            print(f"    Bits set: {', '.join([f'Bit{i}' for i in active_bits]) if active_bits else 'None'}")
            
            # Read probe status
            probe_active = drive_status.get('probe_active', False)
            probe_pos1 = drive_status.get('probe_pos1')
            probe_pos2 = drive_status.get('probe_pos2')
            
            print(f"  Probe:")
            print(f"    Active: {'YES' if probe_active else 'NO'}")
            if probe_pos1 is not None:
                print(f"    Positive edge: {probe_pos1/100.0:.2f} units ({probe_pos1/10000.0:.4f} in)")
            if probe_pos2 is not None:
                print(f"    Negative edge: {probe_pos2/100.0:.2f} units ({probe_pos2/10000.0:.4f} in)")
            
            print(f"{'='*60}\n")
        
        else:
            # Parse command
            parts = cmd.split()
            subcmd = parts[0]
            safe = (len(parts) >= 2 and parts[-1] == "safe")
            
            # Mode switching commands
            if subcmd == "pp":
                switch_to_pp_mode(drive, safe)
            
            elif subcmd == "pv":
                switch_to_pv_mode(drive, safe)
            
            elif subcmd == "csp":
                switch_to_csp_mode(drive, safe)
            
            # Motion commands
            elif subcmd == "pos":
                if len(parts) >= 2 and parts[1] != "safe":
                    try:
                        pos = int(parts[1])
                        set_position_pp(drive, pos)
                    except ValueError:
                        print("Invalid position value")
                else:
                    print("Usage: pos <value>")
            
            elif subcmd == "vel":
                if len(parts) >= 2 and parts[1] != "safe":
                    try:
                        vel = int(parts[1])
                        set_velocity_pv(drive, vel)
                    except ValueError:
                        print("Invalid velocity value")
                else:
                    print("Usage: vel <value>")
            
            elif subcmd == "csp_pos":
                if len(parts) >= 2:
                    try:
                        pos = int(parts[1])
                        set_position_csp(drive, pos)
                    except ValueError:
                        print("Invalid position value")
                else:
                    print("Usage: csp_pos <value>")
            
            elif subcmd == "stop":
                print("Stopping motor...")
                drive.set_velocity(0)
                print("✓ Stop command sent")
            
            elif subcmd == "probe":
                if len(parts) < 2:
                    print("Usage: probe <arm rising|arm falling|status|disable>")
                else:
                    probe_cmd = parts[1]
                    
                    if probe_cmd == "arm":
                        if len(parts) < 3:
                            print("Usage: probe arm <rising|falling>")
                        elif parts[2] == "rising":
                            print("Arming Probe 1 for RISING edge (LOW→HIGH)...")
                            if drive.arm_probe(edge='positive'):
                                print("✓ Probe armed for RISING edge")
                            else:
                                print("❌ Failed to arm probe")
                        elif parts[2] == "falling":
                            print("Arming Probe 1 for FALLING edge (HIGH→LOW)...")
                            if drive.arm_probe(edge='negative'):
                                print("✓ Probe armed for FALLING edge")
                            else:
                                print("❌ Failed to arm probe")
                        else:
                            print("Usage: probe arm <rising|falling>")
                    
                    elif probe_cmd == "disable":
                        print("Disabling touch probe...")
                        if drive.disable_probe():
                            print("✓ Probe disabled")
                        else:
                            print("❌ Failed to disable probe")
                    
                    elif probe_cmd == "status":
                        status = mgr.get_latest_status()
                        if status and status.drives.get(0):
                            drive_status = status.drives[0]
                            di_value = drive_status.get('digital_inputs', 0)
                            probe_active = drive_status.get('probe_active', False)
                            probe_pos1 = drive_status.get('probe_pos1')
                            probe_pos2 = drive_status.get('probe_pos2')
                            
                            print(f"\n{'='*60}")
                            print(f"TOUCH PROBE STATUS")
                            print(f"{'='*60}")
                            print(f"Digital Inputs (0x60FD): 0x{di_value:08X}")
                            print(f"Probe Active: {'YES' if probe_active else 'NO'}")
                            if probe_pos1 is not None:
                                print(f"Positive edge: {probe_pos1/100.0:.2f} units ({probe_pos1/10000.0:.4f} in)")
                            if probe_pos2 is not None:
                                print(f"Negative edge: {probe_pos2/100.0:.2f} units ({probe_pos2/10000.0:.4f} in)")
                            print(f"{'='*60}\n")
                        else:
                            print("⚠️  No status available")
            
            elif subcmd == "home":
                print(f"\n{'='*60}")
                print(f"HOMING AS715N SERVO")
                print(f"{'='*60}")
                print("Method: 1 (Negative limit switch with index pulse)")
                print("Search: 400 units/s (4.0 in/s), Zero: 50 units/s (0.5 in/s)")
                print("")
                
                print("Starting homing...")
                if drive.start_homing(wait_for_completion=True, timeout_s=30.0):
                    new_pos = drive.get_position() or 0.0
                    new_pos_inches = new_pos * 0.01
                    print("")
                    print(f"✅ HOMING COMPLETE")
                    print(f"   Home position: {new_pos:.2f} units ({new_pos_inches:.4f} inches)")
                else:
                    print("")
                    print(f"❌ HOMING FAILED")
                    print(f"   Check drive status and limit switches")
                
                print(f"{'='*60}\n")
            
            else:
                print("Unknown command. Type 'status' for current state or 'quit' to exit")

except KeyboardInterrupt:
    print("\n\nExiting...")

# Cleanup
print()
print("Cleaning up...")

# Stop motor before exit
try:
    drive.set_velocity(0)
    time.sleep(0.1)
except:
    pass

mgr.stop()
print("\n✓ Test complete")
print()
print("="*80)
print("To view detailed debug logs:")
print("  cat /tmp/v2_mode_switching_debug.log")
print("="*80)



