# 机器人 RFID 模块状态 (裁判系统串口协议 V1.2.0 0x0209)

# const for RFID status
uint8 NOT_DETECTED = 0                              # RFID card not detected
uint8 DETECTED = 1                                  # RFID card detected

bool base_gain_point                                # 己方基地增益点
bool central_highland_gain_point                    # 己方中央高地增益点
bool enemy_central_highland_gain_point              # 对方中央高地增益点
bool friendly_trapezoidal_highland_gain_point       # 己方梯形高地增益点
bool enemy_trapezoidal_highland_gain_point          # 对方梯形高地增益点
bool friendly_fly_ramp_front_gain_point             # 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
bool friendly_fly_ramp_back_gain_point              # 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
bool enemy_fly_ramp_front_gain_point                # 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
bool enemy_fly_ramp_back_gain_point                 # 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
bool friendly_central_highland_lower_gain_point     # 己方地形跨越增益点（中央高地下方）
bool friendly_central_highland_upper_gain_point     # 己方地形跨越增益点（中央高地上方）
bool enemy_central_highland_lower_gain_point        # 对方地形跨越增益点（中央高地下方）
bool enemy_central_highland_upper_gain_point        # 对方地形跨越增益点（中央高地上方）
bool friendly_highway_lower_gain_point              # 己方地形跨越增益点（公路下方）
bool friendly_highway_upper_gain_point              # 己方地形跨越增益点（公路上方）
bool enemy_highway_lower_gain_point                 # 对方地形跨越增益点（公路下方）
bool enemy_highway_upper_gain_point                 # 对方地形跨越增益点（公路上方）
bool friendly_fortress_gain_point                   # 己方堡垒增益点
bool friendly_outpost_gain_point                    # 己方前哨站增益点
bool friendly_supply_zone_non_exchange              # 己方与兑换区不重叠的补给区/RMUL 补给区
bool friendly_supply_zone_exchange                  # 己方与兑换区重叠的补给区
bool friendly_assembly_gain_point                     # bit 21 己方装配增益点（旧名：friendly_big_resource_island，V1.2.0协议改名）
bool enemy_assembly_gain_point                        # bit 22 对方装配增益点（旧名：enemy_big_resource_island，V1.2.0协议改名）
bool center_gain_point                              # 控制区(仅 RMUL 适用) bit 23
