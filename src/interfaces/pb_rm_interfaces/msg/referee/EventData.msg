# 场地事件数据 (裁判系统串口协议 V1.2.0 0x0101)

# Constants for the occupation and activation states
uint8 UNOCCUPIED = 0                # Not occupied or not activated
uint8 OCCUPIED_FRIEND = 1           # Occupied or activated by friendly side
uint8 OCCUPIED_ENEMY = 2            # Occupied or activated by enemy side
uint8 OCCUPIED_BOTH = 3             # Occupied or activated by both sides

uint8 non_overlapping_supply_zone   # bit 0：己方与资源区不重叠的补给区占领状态，1 为已占领
uint8 overlapping_supply_zone       # bit 1：己方与资源区重叠的补给区占领状态，1 为已占领
uint8 supply_zone                   # bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）

uint8 small_energy                  # bit 3-4：己方小能量机关的激活状态，0=未激活，1=已激活，2=正在激活 //
uint8 big_energy                    # bit 5-6：己方大能量机关的激活状态，0=未激活，1=已激活，2=正在激活  //

uint8 central_highland              # bit 7-8：己方中央高地的占领状态，1=被己方占领，2=被对方占领
uint8 trapezoidal_highland          # bit 9-10：己方梯形高地的占领状态，1=已占领

uint8 center_gain_zone              # bit 23-24：控制区的占领状态，0=未被占领，1=被己方占领，2=被对方占领，3=被双方占领（仅 RMUL 适用）