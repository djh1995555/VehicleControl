# Description the status of a module or a device

string name
int32  id
string description

# -----------------------------------

uint32 state

# State is described by several parts:
# B0: Extra
# B1: Reserved
# B2: Reserved
# B3.L: Working state
# B3.H: Basic State
# ------------------------------------

uint32 UNDEFINED = 0  # Undefined

# ------------------------------------

uint32 BASIC_MASK = 4026531840  # 0xF0000000
uint32 NORMAL     = 268435456   # 0x10000000
uint32 OFFLINE    = 536870912   # 0x20000000
uint32 DISABLE    = 805306368   # 0x30000000

# ------------------------------------

uint32 WORKING_MASK = 251658240 # 0x0F000000
uint32 FREE         = 16777216  # 0x01000000
uint32 BUSY         = 33554432  # 0x02000000
uint32 WAITING      = 50331648  # 0x03000000

# ------------------------------------

uint32 EXTRA_MASK   = 255 # 0xFF



