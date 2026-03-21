# 地面机器人位置数据 (裁判系统串口协议 V1.2.0 0x020B)
# 2026 season update: removed standard_5, using float32 directly
# 场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向

float32 hero_position_x           # 己方英雄机器人位置 x 轴坐标，单位：m
float32 hero_position_y           # 己方英雄机器人位置 y 轴坐标，单位：m
float32 engineer_position_x       # 己方工程机器人位置 x 轴坐标，单位：m
float32 engineer_position_y       # 己方工程机器人位置 y 轴坐标，单位：m
float32 standard_3_position_x     # 己方 3 号步兵机器人位置 x 轴坐标，单位：m
float32 standard_3_position_y     # 己方 3 号步兵机器人位置 y 轴坐标，单位：m
float32 standard_4_position_x     # 己方 4 号步兵机器人位置 x 轴坐标，单位：m
float32 standard_4_position_y     # 己方 4 号步兵机器人位置 y 轴坐标，单位：m
