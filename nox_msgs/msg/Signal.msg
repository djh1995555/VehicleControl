# Description a signal

# -----------------------------------

int32 flag

uint32 GENERAL   = 0
uint32 EMERGENCY = 1
uint32 ERROR     = 2
uint32 SYSTEM    = 3

# ------------------------------------

int32 type

int32 code

string description

std_msgs/Byte[] data