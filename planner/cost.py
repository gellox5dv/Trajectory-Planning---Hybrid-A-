from models.models import *
from planner.cost_centers import *
from planner.dubins_path_planner import get_dubins_path_length
from planner.motion_primitives import _get_max_steering_angle

import unittest
from unittest.mock import patch

def calculate_cost(
    prev_state: EgoStateStamped, 
    curr_state: EgoStateStamped,
    request: PlanningRequest,
    weights: Dict[str, float]
) -> float:
    
    if weights is None:
        raise ValueError("Weights dictionary must be provided.")
        
    total_cost = 0.0
    
    # 1. Comfort: Jerk
    jerk_costs = cost_jerk(prev_state, curr_state)
    total_cost += weights["jerk_long"] * jerk_costs["long"]
    total_cost += weights["jerk_lat"] * jerk_costs["lat"]
    
    # 2. Comfort: Acceleration
    accel_costs = cost_acceleration(curr_state)
    total_cost += weights["accel_long"] * accel_costs["long"]
    total_cost += weights["accel_lat"] * accel_costs["lat"]
    
    # 3. Efficiency: Target Speed
    speed_cost = cost_target_speed_delta(curr_state, request.target_speed)
    total_cost += weights["speed"] * speed_cost
    
    # 4. Safety: Object Force Fields
    object_cost = cost_objects_force_field(
        current_ego=curr_state,
        previous_ego=prev_state,
        predicted_env=request.environment,
        vehicle_params=request.vehicle_params,
        resolution_ms=request.dt_interpolation
    )
    total_cost += weights["objects"] * object_cost
    
    return total_cost


def calculate_heuristic_cost(state: EgoStateStamped, request: PlanningRequest):

    max_steer = _get_max_steering_angle(state.state.velocity, request.max_a_lat, request.vehicle_params)
    curvature = math.tan(max_steer) / request.vehicle_params.length

    length = get_dubins_path_length(state.state.pos.x,
                                    state.state.pos.y,
                                    state.state.yaw,
                                    request.goal_region.center.x,
                                    request.goal_region.center.y,
                                    request.goal_region.yaw,curvature)
    
    return length



if __name__ == '__main__':
    unittest.main()
