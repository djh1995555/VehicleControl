# Description the state of chassis

Header header

int8    gear       # 档位，1, 2, 3..., 0表示无档位
float64 speed      # 秒速
float64 steering   # 轮偏角，弧度
uint32  mode       # 无人车模式（保留）

# -----------------------------------

int8 GEAR_NONE = 0
int8 GERA_P = 16
int8 GEAR_R = 32
int8 GEAR_N = 48
int8 GEAR_D = 64
int8 GEAR_1 = 1
int8 GEAR_2 = 2
int8 GEAR_3 = 3
int8 GEAR_4 = 4
int8 GEAR_5 = 5
int8 GEAR_6 = 6