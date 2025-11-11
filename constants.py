"""
EtherCAT CiA 402 and device object dictionary constants.
Avoid hard-coded hex values across the codebase.
"""

# Object indices (CoE)
CW_INDEX = 0x6040  # Controlword
SW_INDEX = 0x6041  # Statusword

MODES_OP_INDEX = 0x6060             # Modes of operation (command)
MODES_OP_DISPLAY_INDEX = 0x6061     # Modes of operation display (actual)

TARGET_POSITION_INDEX = 0x607A
TARGET_VELOCITY_INDEX = 0x60FF

POSITION_ACTUAL_INDEX = 0x6064
VELOCITY_ACTUAL_INDEX = 0x606C

ERROR_CODE_INDEX = 0x603F

# Touch probe objects (drive-specific but common on some servos)
PROBE_FUNCTION_INDEX = 0x60B8
PROBE_STATUS_INDEX = 0x60B9
PROBE_POS1_INDEX = 0x60BA
PROBE_POS2_INDEX = 0x60BC
DIGITAL_INPUTS_INDEX = 0x60FD

# Modes of operation values (CiA 402)
MODE_NO_MODE = 0
MODE_PP = 1
MODE_VL = 2
MODE_PV = 3
MODE_HM = 6
MODE_CSP = 8
MODE_CSV = 9
MODE_CST = 10

# Controlword bits/masks (simplified for maintenance)
CW_BIT_NEW_SET_POINT = 4  # PP set-point bit (also used by some drives for homing start in MODE_HM)

# Commonly used masks for enabling operation (simplified sequence)
CW_ENABLE_OP_SIMPLIFIED = 0x000F

# Probe function bits (example for some drives)
PROBE_FUNC_ENABLE_PROBE1 = 0x0001
PROBE_FUNC_PROBE1_POS_EDGE = 0x0004
PROBE_FUNC_PROBE1_NEG_EDGE = 0x0008



