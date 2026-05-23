import math
from models.models import EgoStateStamped



#--- Comfort Costs ---------------------------------------------------------------------

def get_dt_seconds(prev_state: EgoStateStamped, curr_state: EgoStateStamped) -> float:
    dt = (curr_state.timestamp - prev_state.timestamp) / 1000.0
    # Prevent division by zero in case of identical timestamps
    return max(dt, 1e-6)

def cost_jerk(prev_state: EgoStateStamped, curr_state: EgoStateStamped):
    dt = get_dt_seconds(prev_state, curr_state)
    
    # Calculate global jerk via finite differences
    jerk_x = (curr_state.state.acceleration.x - prev_state.state.acceleration.x) / dt
    jerk_y = (curr_state.state.acceleration.y - prev_state.state.acceleration.y) / dt
    
    # Project into local vehicle coordinate system using current yaw
    yaw = curr_state.state.yaw
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    jerk_long = jerk_x * cos_y + jerk_y * sin_y
    jerk_lat = -jerk_x * sin_y + jerk_y * cos_y
    
    # Cost calculation (separated for easy adjustments)
    cost_long = jerk_long ** 2
    cost_lat = jerk_lat ** 2
    
    return {"long": cost_long, "lat": cost_lat}

def cost_acceleration(curr_state: EgoStateStamped):
    # Acceleration is already present in the state, no finite difference needed
    accel_x = curr_state.state.acceleration.x
    accel_y = curr_state.state.acceleration.y
    
    yaw = curr_state.state.yaw
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    accel_long = accel_x * cos_y + accel_y * sin_y
    accel_lat = -accel_x * sin_y + accel_y * cos_y
    
    # Cost calculation (separated for easy adjustments)
    cost_long = accel_long ** 2
    cost_lat = accel_lat ** 2
    
    return {"long": cost_long, "lat": cost_lat}


#--- Safety Costs ------------------------------------------------------------------------

def cost_target_speed_delta(
    curr_state: EgoStateStamped, 
    target_speed: float,
    weight_underspeed: float = 1.0,      
    weight_overspeed: float = 5.0,       
    exp_growth_factor: float = 1.0,      
    max_penalty: float = 1e6             
) -> float:
    
    vx = curr_state.state.velocity.x
    vy = curr_state.state.velocity.y
    current_speed = math.hypot(vx, vy)
    
    delta_v = current_speed - target_speed
    
    if delta_v < 0:
        # Linear penalty for underspeed
        return weight_underspeed * abs(delta_v)
        
    # Exponential barrier for overspeed with overflow protection
    safe_exponent = min(exp_growth_factor * delta_v, 50.0)
    raw_cost = weight_overspeed * (math.exp(safe_exponent) - 1.0)
    
    return min(raw_cost, max_penalty) 