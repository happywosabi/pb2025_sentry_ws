# 机器人增益和底盘能量数据 (裁判系统串口协议 V1.2.0 0x0204)

uint8 recovery_buff           # 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
uint16 cooling_buff           # 机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）  //
uint8 defence_buff            # 机器人防御增益（百分比，值为 50 表示 50% 防御增益）
uint8 vulnerability_buff      # 机器人负防御增益（百分比，值为 30 表示 -30% 防御增益）
uint16 attack_buff            # 机器人攻击增益（百分比，值为 50 表示 50% 攻击增益）
uint8 remaining_energy        # 机器人剩余能量值反馈（bit 0-6），仅在剩余能量小于 50% 时反馈，其余默认反馈 0x80