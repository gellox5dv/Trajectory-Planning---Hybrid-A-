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
    detailed_costs: Dict[str, float],
    cost_cfg: DictConfig
) -> Tuple[float, Dict[str, float]]:
    """
    Calculates the total weighted cost f(n) for a node in the A* search and 
    updates the detailed cost breakdown with the heuristic value.

    The function combines the accumulated path cost from the parent, 
    the transition cost to the current node, and the estimated heuristic 
    cost to the goal, scaled by their respective weights.

    Args:
        path_cost (float): The accumulated transition cost from the root to the parent node.
        node_cost (float): The specific transition cost from the parent to the current node.
        heuristic_cost (float): The estimated distance/cost from the current node to the goal.
        detailed_costs (Dict[str, float]): The dictionary containing the breakdown of the node_cost.
        cost_cfg (DictConfig): Hydra configuration object containing the search weights 
                               (expected to have `cost_cfg.search.w_g` and `cost_cfg.search.w_h`).

    Returns:
        Tuple[float, Dict[str, float]]: 
            - The total weighted cost f(n) used to prioritize nodes in the open set.
            - The updated detailed_costs dictionary including the weighted heuristic.
    """
    
    # g(n): Total accumulated cost to reach the current node
    g_cost = path_cost + node_cost
    
    # Calculate the weighted heuristic
    weighted_heuristic = cost_cfg.search.w_h * heuristic_cost
    
    # Add the heuristic to our cost breakdown dictionary
    detailed_costs["heuristic"] = weighted_heuristic
    
    # f(n) = w_g * g(n) + w_h * h(n)
    total_cost = (cost_cfg.search.w_g * g_cost) + weighted_heuristic
    
    return total_cost, detailed_costs


def calculate_node_cost(
    prev_state: EgoStateStamped, 
    curr_state: EgoStateStamped,
    request: PlanningRequest,
    cost_cfg: DictConfig
) -> Tuple[float, Dict[str, float]]:
    """
    Calculates the total weighted cost for a specific state transition and provides a detailed breakdown.

    Args:
        prev_state (EgoStateStamped): The previous state of the ego vehicle.
        curr_state (EgoStateStamped): The current state of the ego vehicle to be evaluated.
        request (PlanningRequest): The planning request containing environment, target speed, and vehicle params.
        cost_cfg (DictConfig): The Hydra configuration object containing transition weights and parameters.

    Returns:
        Tuple[float, Dict[str, float]]: 
            - The aggregated total cost (float).
            - A dictionary containing the weighted individual cost components for debugging and analysis.
    """
    
    if cost_cfg is None:
        raise ValueError("Hydra configuration (cfg) must be provided.")
        
    detailed_costs: Dict[str, float] = {}
    
    # Extract the transition weights using OmegaConf dot-notation
    weights = cost_cfg.transition_weights
    
    # ---------------------------------------------------------
    # 1. Comfort: Jerk
    # ---------------------------------------------------------
    jerk_costs = cost_jerk(prev_state, curr_state)
    detailed_costs["jerk_long"] = weights.jerk_long * jerk_costs["long"]
    detailed_costs["jerk_lat"] = weights.jerk_lat * jerk_costs["lat"]
    
    # ---------------------------------------------------------
    # 2. Comfort: Acceleration
    # ---------------------------------------------------------
    accel_costs = cost_acceleration(prev_state, curr_state)
    detailed_costs["accel_long"] = weights.accel_long * accel_costs["long"]
    detailed_costs["accel_lat"] = weights.accel_lat * accel_costs["lat"]
    
    # ---------------------------------------------------------
    # 3. Efficiency: Target Speed
    # ---------------------------------------------------------
    speed_cost_raw = cost_target_speed_delta(
        curr_state=curr_state, 
        target_speed=request.target_speed,
        **cost_cfg.cost_target_speed_delta
    )
    detailed_costs["speed"] = weights.speed * speed_cost_raw
    
    # ---------------------------------------------------------
    # 4. Safety: Object Force Fields
    # ---------------------------------------------------------
    object_cost_raw = cost_objects_force_field(
        current_ego=curr_state,
        previous_ego=prev_state,
        predicted_env=request.environment,
        vehicle_params=request.vehicle_params,
        resolution_ms=request.dt_interpolation,
        **cost_cfg.cost_objects_force_field
    )
    detailed_costs["objects"] = weights.objects * object_cost_raw
    
    # ---------------------------------------------------------
    # Aggregation
    # ---------------------------------------------------------
    # The total cost is simply the sum of all weighted components in the dictionary
    total_cost = sum(detailed_costs.values())
    
    return total_cost, detailed_costs


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





