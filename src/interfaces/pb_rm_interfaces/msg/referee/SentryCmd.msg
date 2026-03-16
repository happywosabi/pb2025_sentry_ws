# 哨兵自主决策指令 (裁判系统串口协议 V1.2.0 0x0120)

# sentry_cmd (uint32, bit 0-31)
bool confirm_respawn                       # bit 0：是否确认复活
bool confirm_pay_respawn                   # bit 1：是否确认兑换立即复活
uint16 projectile_allowance_to_exchange    # bit 2-12：将要兑换的发弹量值
uint8 remote_projectile_request_count      # bit 13-16：远程兑换发弹量的请求次数
uint8 remote_hp_request_count              # bit 17-20：远程兑换血量的请求次数
uint8 posture_command                      # bit 21-22：修改当前姿态指令（1进攻，2防御，3移动）
bool activate_rune                         # bit 23：是否确认使能量机关进入正在激活状态

# Constants
uint8 POSTURE_ATTACK = 1
uint8 POSTURE_DEFENSE = 2
uint8 POSTURE_MOVE = 3