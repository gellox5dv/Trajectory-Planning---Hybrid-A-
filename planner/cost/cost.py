from models.models import *
from planner.cost.transition_cost import *
from planner.cost.dubins_path_planner import get_dubins_path_length
from planner.motion_primitives import _get_max_steering_angle

import unittest
from unittest.mock import patch

from omegaconf import DictConfig

from models.models import EgoStateStamped, PlanningRequest
from planner.cost.transition_cost import (
    cost_jerk,
    cost_acceleration,
    cost_target_speed_delta,
    cost_objects_force_field
)



def calculate_total_cost(
    path_cost: float, 
    node_cost: float, 
    heuristic_cost: float, 
    cost_cfg: DictConfig
) -> float:
    """
    Calculates the total weighted cost f(n) for a node in the A* search.

    The function combines the accumulated path cost from the parent, 
    the transition cost to the current node, and the estimated heuristic 
    cost to the goal, scaled by their respective weights.

    Args:
        path_cost (float): The accumulated transition cost from the root to the parent node.
        node_cost (float): The specific transition cost from the parent to the current node.
        heuristic_cost (float): The estimated distance/cost from the current node to the goal.
        cfg (DictConfig): Hydra configuration object containing the search weights 
                          (expected to have `cfg.search.w_g` and `cfg.search.w_h`).

    Returns:
        float: The total weighted cost f(n) used to prioritize nodes in the open set.
    """
    
    # g(n): Total accumulated cost to reach the current node
    g_cost = path_cost + node_cost
    
    # f(n) = w_g * g(n) + w_h * h(n)
    total_cost = (cost_cfg.search.w_g * g_cost) + (cost_cfg.search.w_h * heuristic_cost)
    
    return total_cost


def calculate_node_cost(
    prev_state: EgoStateStamped, 
    curr_state: EgoStateStamped,
    request: PlanningRequest,
    cost_cfg: DictConfig
) -> float:
    """
    Calculates the total weighted cost for a specific state transition.

    Args:
        prev_state (EgoStateStamped): The previous state of the ego vehicle.
        curr_state (EgoStateStamped): The current state of the ego vehicle to be evaluated.
        request (PlanningRequest): The planning request containing environment, target speed, and vehicle params.
        cfg (DictConfig): The Hydra configuration object containing transition weights and function-specific parameters.

    Returns:
        float: The aggregated total cost. Raises ValueError if config is missing.
    """
    
    if cost_cfg is None:
        raise ValueError("Hydra configuration (cfg) must be provided.")
        
    total_cost = 0.0
    
    # Extract the transition weights using OmegaConf dot-notation
    weights = cost_cfg.transition_weights
    
    # ---------------------------------------------------------
    # 1. Comfort: Jerk
    # ---------------------------------------------------------
    jerk_costs = cost_jerk(prev_state, curr_state)
    total_cost += weights.jerk_long * jerk_costs["long"]
    total_cost += weights.jerk_lat * jerk_costs["lat"]
    
    # ---------------------------------------------------------
    # 2. Comfort: Acceleration
    # ---------------------------------------------------------
    accel_costs = cost_acceleration(prev_state, curr_state)
    total_cost += weights.accel_long * accel_costs["long"]
    total_cost += weights.accel_lat * accel_costs["lat"]
    
    # ---------------------------------------------------------
    # 3. Efficiency: Target Speed
    # ---------------------------------------------------------
    speed_cost = cost_target_speed_delta(
        curr_state=curr_state, 
        target_speed=request.target_speed,
        **cost_cfg.cost_target_speed_delta
    )
    total_cost += weights.speed * speed_cost
    
    # ---------------------------------------------------------
    # 4. Safety: Object Force Fields
    # ---------------------------------------------------------
    object_cost = cost_objects_force_field(
        current_ego=curr_state,
        previous_ego=prev_state,
        predicted_env=request.environment,
        vehicle_params=request.vehicle_params,
        resolution_ms=request.dt_interpolation,
        **cost_cfg.cost_objects_force_field
    )
    total_cost += weights.objects * object_cost
    
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

    return length





