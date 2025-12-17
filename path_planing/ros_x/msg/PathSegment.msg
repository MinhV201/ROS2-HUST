
uint8 TYPE_LINE = 0
uint8 TYPE_ARC = 1
uint8 TYPE_SPIN = 2

uint8 DIR_LEFT = 0   
uint8 DIR_RIGHT = 1  


uint8 type                  # 0: Line, 1: Arc 2: Spin
geometry_msgs/Point start   
geometry_msgs/Point end     
float64 radius              
uint8 direction            