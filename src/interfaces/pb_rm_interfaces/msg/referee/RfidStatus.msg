# 机器人 RFID 模块状态 (裁判系统串口协议 V1.7.0 0x0209)

# const for RFID status
uint8 NOT_DETECTED = 0                              # RFID card not detected
uint8 DETECTED = 1                                  # RFID card detected

bool own_base
bool own_central_highland
bool opponent_central_highland
bool own_trapezoidal_highland
bool opponent_trapezoidal_highland

# 飞坡地形跨越增益点
bool own_terrain_cross_ramp_front
bool own_terrain_cross_ramp_rear
bool opponent_terrain_cross_ramp_front
bool opponent_terrain_cross_ramp_rear

# 中央高地地形跨越增益点
bool own_terrain_cross_central_highland_lower
bool own_terrain_cross_central_highland_upper
bool opponent_terrain_cross_central_highland_lower
bool opponent_terrain_cross_central_highland_upper

# 公路地形跨越增益点
bool own_terrain_cross_highway_lower
bool own_terrain_cross_highway_upper
bool opponent_terrain_cross_highway_lower
bool opponent_terrain_cross_highway_upper

# 建筑/补给类增益点
bool own_fortress
bool own_outpost
bool own_supply_zone_non_overlap_resource
bool own_supply_zone_overlap_resource
bool own_assembly
bool opponent_assembly
bool central_rmul_only
bool opponent_fortress
bool opponent_outpost

    # 己方隧道地形跨越增益点
bool own_terrain_cross_tunnel_highway_lower
bool own_terrain_cross_tunnel_highway_middle
bool own_terrain_cross_tunnel_highway_upper
bool own_terrain_cross_tunnel_trapezoidal_low
bool own_terrain_cross_tunnel_trapezoidal_middle
bool own_terrain_cross_tunnel_trapezoidal_high

    # 对方隧道地形跨越增益点
bool opponent_terrain_cross_tunnel_highway_lower
bool opponent_terrain_cross_tunnel_highway_middle
bool opponent_terrain_cross_tunnel_highway_upper
bool opponent_terrain_cross_tunnel_trapezoidal_low
bool opponent_terrain_cross_tunnel_trapezoidal_middle
bool opponent_terrain_cross_tunnel_trapezoidal_high