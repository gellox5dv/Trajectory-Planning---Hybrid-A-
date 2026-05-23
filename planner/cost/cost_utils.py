from models.models import *
import math




def get_x_y_yaw_from_state(state: EgoStateStamped) -> Tuple[float, float, float]:
    return state.state.pos.x, state.state.pos.y, state.state.yaw

def get_bbox_corners(
    center_x: float, 
    center_y: float, 
    yaw: float, 
    length: float, 
    width: float
) -> List[Vector2D]:
    
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    hl = length / 2.0
    hw = width / 2.0
    
    local_corners = [
        (hl, hw),
        (hl, -hw),
        (-hl, -hw),
        (-hl, hw)
    ]
    
    corners = []
    for lx, ly in local_corners:
        global_x = center_x + (lx * cos_y - ly * sin_y)
        global_y = center_y + (lx * sin_y + ly * cos_y)
        corners.append(Vector2D(x=global_x, y=global_y))
        
    return corners