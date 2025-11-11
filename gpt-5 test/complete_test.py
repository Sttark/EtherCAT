#!/usr/bin/env python3
"""
COMPLETE TEST - AS715N Servo Drive using EtherCAT V2 Driver (gpt-5 test)

This is a complete duplicate of the interactive mode/motion/probe test, but wired
to the ethercat_v2 isolated process driver. It exercises:
- Mode switching (PP/PV/CSP)
- Motion commands (PP, PV, CSP)
- Homing (Method 1)
- Touch probe arm/disable/status
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

# Reduce noise from v2 internals
logging.getLogger('ethercat_v2').setLevel(logging.WARNING)

print("="*80)
print("COMPLETE TEST - AS715N Servo using EtherCAT V2 Driver (gpt-5 test)")
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
from ethercat_v2.constants import MODE_PP, MODE_PV, MODE_CSP

print("="*80)
print("STEP 1: Configure network and drive...")
print("="*80)

# XML file for AS715N servo
servo_xml = "/home/sttark/Desktop/github/Core-Cutter/ECAT-config/STEPPERONLINE_A6_Servo_V0.04.xml"

# Custom PDO mapping for AS715N (keep modes out of PDO)
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
        (0x60BB, 0, 32),  # Probe-1 Negative Edge (device-dependent; 0x60BC on some ESI)
        (0x60FD, 0, 32),  # Digital Inputs
        (0x603F, 0, 16),  # Error Code
    ]
}

# Create network configuration (single slave for servo)
cfg = EthercatNetworkConfig(
    master_index=0,
    cycle_time_ms=5.0,
    sdo_only=False,
    slaves=[
        DriveConfig(
            position=0,
            alias=0,
            # If vendor_id/product_code are omitted, make sure the process can derive them or accept 0
            vendor_id=None,
            product_code=None,
            enable_dc=True,
            operation_mode=1,  # PP initially
            profile_velocity=500,      # 500 UU/s = 5 in/s
            profile_acceleration=1000, # 1000 UU/s² = 10 in/s²
            max_velocity=500,          # safety limit
            homing=HomingConfig(
                method=1,
                search_vel=400,
                zero_vel=50,
                accel=1000,
                offset=99,
                unit='native'
            ),
            rotation_direction='reverse',
            inertia_ratio=3.0,
            position_limits=(0, 0, 'disable'),
            xml=XmlConfig(xml_file=servo_xml),
            pdo=PdoSelection(
                rx_pdos=[0x1600],
                tx_pdos=[0x1A00],
                custom_pdo_config=as715n_custom_pdos
            ),
            unit_conversion=UnitConversion(
                units_per_pulse=1.0,
                scale_factor=1040.42  # 1040.42 pulses = 1 UU (0.01")
            )
        )
    ]
)

print("✓ Configuration created")
print("  - Custom PDO mapping (velocity + probe; modes via SDO)")
print("  - DC synchronization enabled")
print("  - Unit conversion target: 1040.42 pulses = 1 user unit")
print("  - Homing: Method 1, 400 UU/s search")
print("  - Rotation: REVERSED, Inertia: 3:1")
print()

print("="*80)
print("STEP 2: Start EtherCAT process manager...")
print("="*80)

mgr = EtherCATProcessManager(cfg)
mgr.start()

print("Waiting for EtherCAT process to initialize...")
time.sleep(2.0)

drive = attach_drive_handle(mgr, 0)
print("✓ Drive handle attached")
print()

print("="*80)
print("STEP 3: Wait for drive to enable...")
print("="*80)

# Wait for drive to be enabled (statusword Operation Enabled)
for sec in range(15):
    time.sleep(1)
    status = mgr.get_latest_status()
    enabled = False
    sw = 0
    if status and status.drives.get(0):
        sw = status.drives[0].get('statusword', 0)
        enabled = (sw & 0x000F) == 0x0007
    print(f"[{sec+1}s] Statusword: 0x{sw:04X}, Enabled: {enabled}")
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
print("AS715N Servo via v2 driver:")
print("  ✓ DC synchronized")
print("  ✓ PP/PV/CSP supported; 0x6060 via SDO if not in PDO")
print("  ✓ Probe function supported via PDO write (arm/disable), DI published")
print()

# Helpers
def switch_to_pp_mode(d, safe=False):
    print(f"\n>>> Switching to PP{' (safe)' if safe else ''}...")
    d.set_position_mode(safe_switch=safe)
    print("✓ PP queued")
    return True

def switch_to_pv_mode(d, safe=False):
    print(f"\n>>> Switching to PV{' (safe)' if safe else ''}...")
    d.set_velocity_mode(safe_switch=safe)
    print("✓ PV queued")
    return True

def switch_to_csp_mode(d, safe=False):
    print(f"\n>>> Switching to CSP{' (safe)' if safe else ''}...")
    d.set_csp_mode(safe_switch=safe)
    print("✓ CSP queued (position will stream)")
    return True

def set_position_pp(d, position):
    inches = position * 0.01
    print(f"Move to {position} UU ({inches:.4f} in)...")
    d.set_position_absolute(position)
    print("✓ Position queued")
    return True

def set_velocity_pv(d, velocity):
    print(f"Set velocity {velocity} UU/s ({velocity*0.01:.4f} in/s)...")
    d.set_velocity(velocity)
    print("✓ Velocity queued")
    return True

def set_position_csp(d, position):
    print(f"Set CSP position {position} UU ({position*0.01:.4f} in)...")
    d.set_position_csp(position)
    print("✓ CSP queued")
    return True

print("="*80)
print("INTERACTIVE MODE SWITCHING TEST - AS715N SERVO (v2)")
print("="*80)
print()
print("Units: 1 UU = 0.01 inches")
print("Commands:")
print("  pp | pv | csp")
print("  pos <UU> | vel <UU/s> | csp_pos <UU>")
print("  probe arm rising|falling | probe disable | probe status")
print("  home | stop | status | quit")
print()

try:
    while True:
        cmd = input("> ").strip().lower()
        if not cmd:
            continue
        if cmd in ("quit", "q", "exit"):
            print("Exiting...")
            break
        elif cmd in ("status", "st"):
            status = mgr.get_latest_status()
            if not status or not status.drives.get(0):
                print("⚠️  No status available")
                continue
            s = status.drives[0]
            mode = s.get('mode_display')
            sw = s.get('statusword', 0)
            pos = drive.get_position() or 0.0
            vel = drive.get_velocity() or 0.0
            di = s.get('digital_inputs', 0)
            probe_active = s.get('probe_active', False)
            p1 = s.get('probe_pos1')
            p2 = s.get('probe_pos2')
            print("\n" + "="*60)
            print("AS715N STATUS")
            print("="*60)
            print(f"  Mode: {mode}  Status: 0x{sw:04X}")
            print(f"  Position: {pos:.2f} UU ({pos*0.01:.4f} in)")
            print(f"  Velocity: {vel:.2f} UU/s ({vel*0.01:.4f} in/s)")
            print(f"  DI: 0x{di:08X}")
            print(f"  Probe active: {'YES' if probe_active else 'NO'}")
            if p1 is not None:
                print(f"  Pos edge: {p1/100.0:.2f} UU ({p1/10000.0:.4f} in)")
            if p2 is not None:
                print(f"  Neg edge: {p2/100.0:.2f} UU ({p2/10000.0:.4f} in)")
            print("="*60 + "\n")
        else:
            parts = cmd.split()
            sub = parts[0]
            safe = (len(parts) >= 2 and parts[-1] == "safe")
            if sub == "pp":
                switch_to_pp_mode(drive, safe)
            elif sub == "pv":
                switch_to_pv_mode(drive, safe)
            elif sub == "csp":
                switch_to_csp_mode(drive, safe)
            elif sub == "pos":
                if len(parts) >= 2 and parts[1] != "safe":
                    try:
                        set_position_pp(drive, int(parts[1]))
                    except ValueError:
                        print("Invalid position")
                else:
                    print("Usage: pos <UU>")
            elif sub == "vel":
                if len(parts) >= 2 and parts[1] != "safe":
                    try:
                        set_velocity_pv(drive, int(parts[1]))
                    except ValueError:
                        print("Invalid velocity")
                else:
                    print("Usage: vel <UU/s>")
            elif sub == "csp_pos":
                if len(parts) >= 2:
                    try:
                        set_position_csp(drive, int(parts[1]))
                    except ValueError:
                        print("Invalid position")
                else:
                    print("Usage: csp_pos <UU>")
            elif sub == "stop":
                print("Stopping motor...")
                drive.set_velocity(0)
                print("✓ Stop sent")
            elif sub == "probe":
                if len(parts) < 2:
                    print("Usage: probe arm rising|falling | probe disable | probe status")
                else:
                    p = parts[1]
                    if p == "arm":
                        if len(parts) < 3:
                            print("Usage: probe arm rising|falling")
                        elif parts[2] == "rising":
                            print("Arming probe for RISING edge...")
                            print("✓" if drive.arm_probe(edge='positive') else "❌ Failed")
                        elif parts[2] == "falling":
                            print("Arming probe for FALLING edge...")
                            print("✓" if drive.arm_probe(edge='negative') else "❌ Failed")
                        else:
                            print("Usage: probe arm rising|falling")
                    elif p == "disable":
                        print("Disabling probe...")
                        print("✓" if drive.disable_probe() else "❌ Failed")
                    elif p == "status":
                        status = mgr.get_latest_status()
                        if status and status.drives.get(0):
                            s = status.drives[0]
                            di = s.get('digital_inputs', 0)
                            probe_active = s.get('probe_active', False)
                            p1 = s.get('probe_pos1')
                            p2 = s.get('probe_pos2')
                            print("\n" + "="*60)
                            print("TOUCH PROBE STATUS")
                            print("="*60)
                            print(f"  DI: 0x{di:08X}")
                            print(f"  Active: {'YES' if probe_active else 'NO'}")
                            if p1 is not None:
                                print(f"  Pos edge: {p1/100.0:.2f} UU ({p1/10000.0:.4f} in)")
                            if p2 is not None:
                                print(f"  Neg edge: {p2/100.0:.2f} UU ({p2/10000.0:.4f} in)")
                            print("="*60 + "\n")
                        else:
                            print("⚠️  No status available")
                    else:
                        print("Usage: probe arm rising|falling | probe disable | probe status")
            elif sub == "home":
                print("\n" + "="*60)
                print("HOMING AS715N SERVO")
                print("="*60)
                print("Method: 1 (negative limit + index)")
                print("Search: 400 UU/s, Zero: 50 UU/s, Accel: 1000 UU/s²")
                print("")
                if drive.start_homing(wait_for_completion=True, timeout_s=30.0):
                    new_pos = drive.get_position() or 0.0
                    print("✅ HOMING COMPLETE")
                    print(f"   Home: {new_pos:.2f} UU ({new_pos*0.01:.4f} in)")
                else:
                    print("❌ HOMING FAILED")
                print("="*60 + "\n")
            else:
                print("Unknown command. Try: status, pp, pv, csp, pos, vel, csp_pos, probe, home, stop, quit")
except KeyboardInterrupt:
    print("\n\nExiting...")

# Cleanup
print()
print("Cleaning up...")
try:
    drive.set_velocity(0)
    time.sleep(0.1)
except Exception:
    pass

mgr.stop()
print("\n✓ Test complete")
print()
print("="*80)
print("To view detailed debug logs:")
print("  cat /tmp/v2_mode_switching_debug.log")
print("="*80)




