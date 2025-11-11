#!/usr/bin/env python3
"""
Isolated test for mode switching and basic motion using ethercat_v2.
Non-blocking commands; cyclic task maintains PDO controlword/modes/targets.
"""

import time
import logging

from .config_schema import EthercatNetworkConfig, DriveConfig, XmlConfig
from .process_manager import EtherCATProcessManager
from .client import attach_drive_handle


logging.basicConfig(level=logging.INFO)


def run():
    cfg = EthercatNetworkConfig(
        master_index=0,
        cycle_time_ms=5.0,
        sdo_only=False,
        slaves=[
            DriveConfig(
                position=0,
                vendor_id=0x0,
                product_code=0x0,
                xml=XmlConfig(xml_file="/home/sttark/Desktop/github/Core-Cutter/ECAT-config/STEPPERONLINE_A6_Servo_V0.04.xml"),
            )
        ],
    )

    mgr = EtherCATProcessManager(cfg)
    mgr.start()

    drive = attach_drive_handle(mgr, 0)

    print("Network ready. Interactive mode switching test.")
    print("Type: pp | pv | csp | status | quit")
    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd in ("quit", "q", "exit"):
                break
            elif cmd == "pp":
                drive.set_position_mode()
                print("PP mode command queued")
            elif cmd == "pv":
                drive.set_velocity_mode()
                print("PV mode command queued")
            elif cmd == "csp":
                drive.set_csp_mode()
                print("CSP mode command queued")
            elif cmd == "status":
                print("mode:", drive._read_status(0, 'mode_display'))
                print("statusword:", drive.get_status_word())
            else:
                print("unknown command")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    run()


