# 哨兵自主决策信息同步 (裁判系统串口协议 V1.2.0 0x020D)

# sentry_info (uint32, bit 0-31)
uint16 exchanged_projectile_allowance      # bit 0-10：除远程兑换外，哨兵成功兑换的允许发弹量
uint8 remote_projectile_exchange_count     # bit 11-14：哨兵成功远程兑换允许发弹量的次数
uint8 remote_hp_exchange_count             # bit 15-18：哨兵成功远程兑换血量的次数
bool can_free_respawn                      # bit 19：是否可以确认免费复活
bool can_pay_respawn                       # bit 20：是否可以兑换立即复活
uint16 respawn_gold_cost                   # bit 21-30：兑换立即复活需要花费的金币数

# sentry_info_2 (uint16, bit 0-15)
bool is_out_of_combat                      # bit 0：脱战状态（V1.2.0新增）
uint16 team_projectile_allowance_remaining # bit 1-11：队伍17mm剩余可兑换数（V1.2.0新增）
uint8 current_posture                      #和 bit 12-13
bool can_activate_rune                     # bit 14


# Constants
uint8 POSTURE_ATTACK = 1
uint8 POSTURE_DEFENSE = 2
uint8 POSTURE_MOVE = 3