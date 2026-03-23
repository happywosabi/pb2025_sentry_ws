# 比赛状态数据 (裁判系统串口协议 V1.7.0 0x0001)

# constants for game progress

# 1 RoboMaster 机甲大师超级对抗赛
# 2 RoboMaster 机甲大师高校单项赛
# 3 ICRA RoboMaster 高校人工智能挑战赛
# 4 RoboMaster 机甲大师高校联盟赛 3V3 对抗 
# 5 RoboMaster 机甲大师高校联盟赛步兵对抗
uint8 game_type

uint8 NOT_START = 0                 # 未开始比赛
uint8 PREPARATION = 1               # 准备阶段
uint8 SELF_CHECKING = 2             # 十五秒裁判系统自检阶段
uint8 COUNT_DOWN = 3                # 五秒倒计时
uint8 RUNNING = 4                   # 比赛中
uint8 GAME_OVER = 5                 # 比赛结算中

uint8 game_progress
int32 stage_remain_time             # 当前阶段剩余时间，单位：秒
