import math
from planner.cost.cost_utils import *
from models.models import EgoStateStamped
from shapely.geometry import Polygon
from collision.collision import get_distance_to_objects
from models.models import *


#--- Comfort Costs ---------------------------------------------------------------------

def get_dt_seconds(prev_state: EgoStateStamped, curr_state: EgoStateStamped) -> float:
    """
    Calculates the time difference between two states in seconds.

    Args:
        prev_state (EgoStateStamped): The previous state of the ego vehicle.
        curr_state (EgoStateStamped): The current state of the ego vehicle.

    Returns:
        float: Time difference in seconds, safely clamped to a minimum of 1e-6 to prevent division by zero.
    """
    dt = (curr_state.timestamp - prev_state.timestamp) / 1000.0
    return max(dt, 1e-6)

def cost_jerk(prev_state: EgoStateStamped, curr_state: EgoStateStamped):
    """
    Calculates the squared jerk costs in the vehicle's local longitudinal and lateral directions.

    Args:
        prev_state (EgoStateStamped): The previous state of the ego vehicle.
        curr_state (EgoStateStamped): The current state of the ego vehicle.

    Returns:
        dict: A dictionary containing the squared jerk costs with keys "long" and "lat".
    """
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
    
    # Cost calculation 
    cost_long = jerk_long ** 2
    cost_lat = jerk_lat ** 2
    
    return {"long": cost_long, "lat": cost_lat}

def cost_acceleration(prev_state: EgoStateStamped, curr_state: EgoStateStamped):
    """
    Calculates the squared acceleration costs in local longitudinal and lateral directions 
    using the average acceleration over the time step.

    Args:
        prev_state (EgoStateStamped): The previous state of the ego vehicle.
        curr_state (EgoStateStamped): The current state of the ego vehicle.

    Returns:
        dict: A dictionary containing the squared acceleration costs with keys "long" and "lat".
    """
    dt = get_dt_seconds(prev_state, curr_state)
    
    accel_x = (curr_state.state.velocity.x - prev_state.state.velocity.x) / dt
    accel_y = (curr_state.state.velocity.y - prev_state.state.velocity.y) / dt
    
    yaw = curr_state.state.yaw
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    accel_long = accel_x * cos_y + accel_y * sin_y
    accel_lat = -accel_x * sin_y + accel_y * cos_y
    
    # Cost calculation
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
    """
    Calculates the cost for deviating from the target speed. Applies a linear penalty 
    for driving too slow and an exponential penalty barrier for driving too fast.

    Args:
        curr_state (EgoStateStamped): The current state of the ego vehicle.
        target_speed (float): The desired target speed [m/s].
        weight_underspeed (float): Linear weight applied when velocity is below target.
        weight_overspeed (float): Base weight applied when velocity exceeds target.
        exp_growth_factor (float): Factor controlling the steepness of the overspeed exponential curve.
        max_penalty (float): Absolute maximum cost limit to prevent math overflows.

    Returns:
        float: The calculated penalty cost for speed deviation.
    """
    
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


def cost_objects_force_field(
    current_ego: EgoStateStamped,
    previous_ego: EgoStateStamped,
    predicted_env: PredictedEnvironment,
    vehicle_params: VehicleParameters,
    resolution_ms: int,
    ego_safe_margin: float = 0.5,        
    obj_safe_margin: float = 0.5,        
    speed_expansion_factor: float = 0.4, 
    weight_ff_high: float = 50.0,        
    weight_ff_low: float = 1.0,          
    dist_epsilon: float = 0.1
) -> float:
    """
    Calculates proximity costs to dynamic objects using a dual force-field approach.

    Args:
        current_ego (EgoStateStamped): The current state of the ego vehicle.
        previous_ego (EgoStateStamped): The previous state of the ego vehicle (for continuous collision check).
        predicted_env (PredictedEnvironment): Environment data containing dynamic object predictions.
        vehicle_params (VehicleParameters): Physical properties of the ego vehicle.
        resolution_ms (int): Interpolation resolution for the collision checker [ms].
        ego_safe_margin (float): Margin to inflate the ego vehicle's bounding box [m].
        obj_safe_margin (float): Base margin to inflate dynamic object bounding boxes [m].
        speed_expansion_factor (float): Multiplier to dynamically extend the object's force field based on speed [s].
        weight_ff_high (float): High penalty weight applied when intersecting an object's high force field.
        weight_ff_low (float): Base proximity weight applied outside the high force field.
        dist_epsilon (float): Small value to prevent division by zero in distance calculations.

    Returns:
        float: The maximum calculated proximity cost across all objects, or float('inf') if a hard collision occurs.
    """
    
    # 1. Get exact distances and check for hard collisions in the current time step
    distances, is_collision = get_distance_to_objects(
        current_ego, previous_ego, predicted_env, vehicle_params, resolution_ms
    )
    
    if is_collision:
        return float('inf')
        
    if not distances:
        return 0.0

    # 2. Construct the Ego "Safe Area" Polygon
    ego_x, ego_y, ego_yaw = get_x_y_yaw_from_state(current_ego)
    
    # Shift reference point from rear axle to the geometric center
    ego_x_center = ego_x + (vehicle_params.length / 2 - vehicle_params.rear_to_wheel) * math.cos(ego_yaw)
    ego_y_center = ego_y + (vehicle_params.length / 2 - vehicle_params.rear_to_wheel) * math.sin(ego_yaw)
    
    # Inflate the ego bounding box by the safety margin
    ego_corners = get_bbox_corners(
        ego_x_center, ego_y_center, ego_yaw,
        vehicle_params.length + 2 * ego_safe_margin,
        vehicle_params.width + 2 * ego_safe_margin
    )
    ego_safe_polygon = Polygon([(c.x, c.y) for c in ego_corners])

    max_cost = 0.0
    
    # 3. Evaluate force fields for each relevant object
    for obj_id, base_dist in distances:
        obj_states = predicted_env.objects.get(obj_id, [])
        if not obj_states:
            continue
            
        # Find the object state that chronologically matches the current ego timestamp
        current_obj_state = obj_states[0]
        for state in obj_states:
            if state.timestamp >= current_ego.timestamp:
                current_obj_state = state
                break

        # Extract object coordinates directly (since get_x_y_yaw is ego-only)
        obj_x = current_obj_state.state.pos.x
        obj_y = current_obj_state.state.pos.y
        obj_yaw = current_obj_state.state.yaw
        obj_speed = abs(current_obj_state.state.velocity)
        
        # 4. Construct the Object's "Force Field High"
        # The longitudinal length expands dynamically based on the object's speed
        ff_high_length = current_obj_state.state.length + 2 * obj_safe_margin + (obj_speed * speed_expansion_factor)
        ff_high_width = current_obj_state.state.width + 2 * obj_safe_margin
        
        obj_corners = get_bbox_corners(obj_x, obj_y, obj_yaw, ff_high_length, ff_high_width)
        obj_ff_high_polygon = Polygon([(c.x, c.y) for c in obj_corners])

        # 5. Apply Cost Mapping based on Force Field intersection
        if ego_safe_polygon.intersects(obj_ff_high_polygon):
            # Ego's safety area overlaps with object's High Force Field
            cost = weight_ff_high / (base_dist + dist_epsilon)
        else:
            # Ego is outside High Force Field, apply base safety cost (Low Force Field)
            cost = weight_ff_low / (base_dist + dist_epsilon)
            
        # 6. Aggregate by keeping the maximum cost encountered
        if cost > max_cost:
            max_cost = cost
            
    return max_cost