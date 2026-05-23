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
    """
    Calculates the total weighted cost for a specific state transition.

    Args:
        prev_state (EgoStateStamped): The previous state of the ego vehicle.
        curr_state (EgoStateStamped): The current state of the ego vehicle to be evaluated.
        request (PlanningRequest): The planning request containing environment, target speed, and vehicle params.
        weights (Dict[str, float]): Dictionary containing weighting factors for each cost component.
                                    Expected keys: 'jerk_long', 'jerk_lat', 'accel_long', 'accel_lat', 'speed', 'objects'.

    Returns:
        float: The aggregated total cost. Raises ValueError if weights are not provided.
    """
    
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


def calculate_heuristic_cost(state: EgoStateStamped, request: PlanningRequest) -> float:
    """
    Calculates the heuristic cost from a given state to the goal region using a Dubins path.

    Args:
        state (EgoStateStamped): The starting state for the heuristic calculation.
        request (PlanningRequest): The planning request containing the goal region, maximum lateral 
                                   acceleration (max_a_lat), and vehicle parameters.

    Returns:
        float: The estimated distance (Dubins path length) to the goal region.
    """
    
    # 1. Calculate scalar speed from Vector2D
    current_speed = math.hypot(state.state.velocity.x, state.state.velocity.y)

    # 2. Get max steer angle for this specific speed
    max_steer = _get_max_steering_angle(current_speed, request.max_a_lat, request.vehicle_params)
    
    # 3. Calculate curvature using the WHEELBASE, not the total length
    curvature = math.tan(max_steer) / request.vehicle_params.wheel_base

    # 4. Compute Dubins path length
    length = get_dubins_path_length(
        state.state.pos.x,
        state.state.pos.y,
        state.state.yaw,
        request.goal_region.center.x,
        request.goal_region.center.y,
        request.goal_region.yaw,
        curvature
    )



