uint64 id       # 覆盖信息本身的ID
uint64 on       # 被覆盖物的ID
uint8 type      # 被覆盖物的类型
float64 start   # 被覆盖的范围起始（时间或距离，由语境定义）
float64 end     # 被覆盖的范围终止

# -----------------------------------
# Type to overlap

uint8 UNKNOWN = 0
uint8 LANE = 1

# -----------------------------------