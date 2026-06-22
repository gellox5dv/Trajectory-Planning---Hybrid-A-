import math
import time
import heapq
from typing import Optional, List

from omegaconf import DictConfig
from shapely.geometry import Polygon

from models.models import PlanningRequest, PlanResult, EgoStateStamped, Trajectory, EgoInput, GoalRegion
from planner.motion_primitives import get_motion_primitives
from planner.state_node import StateNode
from motion.bicycle import kinematic_bicycle
from utils.helper import get_signed_magnitude
from planner.cost.cost import calculate_node_cost, calculate_heuristic_cost, calculate_total_cost


class IDProvider:
    """
    Provides unique, incrementing integer IDs for search tree nodes.
    """
    def __init__(self, start_id=0):
        self.current_id = start_id
        
    def get_id(self) -> int:
        result = self.current_id
        self.current_id += 1
        return result


def plan(request: PlanningRequest, cfg: DictConfig, debug: bool = False) -> PlanResult:
    """
    Computes a trajectory using a Hybrid A* search algorithm with state pruning.

    This function expands a search tree from the vehicle's starting state towards a goal region. 
    It evaluates dynamically feasible motion primitives and prioritizes nodes based on 
    accumulated path costs and a heuristic estimate to the goal. 
    
    It utilizes a grid-based spatial hashing technique (State Pruning) to discard 
    duplicate states that have similar coordinates and kinematics at the same time step.

    Args:
        request (PlanningRequest): The planning request containing the initial state, 
                                   the predicted environment, the goal region, and target speed.
        cfg (DictConfig): The Hydra configuration object containing parameters for the 
                          planner, cost weights, motion primitives, and vehicle dynamics.
        debug (bool): If True, prints detailed performance KPIs and statistics to the console.

    Returns:
        PlanResult: An object containing the success flag, the final trajectory (if successful), 
                    the total cost, a diagnostic status message, and the root node.
    """

    if hasattr(cfg.planner, "tolerances"):
        tol_pos = cfg.planner.tolerances.get("pos", 0.5)
        tol_yaw = cfg.planner.tolerances.get("yaw", 0.087)
        tol_vel = cfg.planner.tolerances.get("velocity", 0.5)
        tol_acc = cfg.planner.tolerances.get("acceleration", 0.5)
    else:
        tol_pos, tol_yaw, tol_vel, tol_acc = 0.5, 0.087, 0.5, 0.5

    inv_tol_pos = 1.0 / tol_pos
    inv_tol_yaw = 1.0 / tol_yaw
    inv_tol_vel = 1.0 / tol_vel
    inv_tol_acc = 1.0 / tol_acc

    open_nodes_pq: list[StateNode] = []
    end_nodes_pq: list[StateNode] = []
    id_provider = IDProvider()
    
    visited_states = set()
    
    max_depth = math.ceil(cfg.planner.horizon / cfg.planner.dt_sim)
    call_time = time.perf_counter()

    total_generated_nodes = 1  
    total_pruned_nodes = 0
    max_depth_reached = 0
    ttfs_sec: Optional[float] = None

    if is_in_goal_region(request.start_state, request.goal_region, cfg.vehicle):
        return PlanResult(
            success=False,
            goal_region_reached=None,
            trajectory=None,
            cost=None,
            status_message='Vehicle is already in the goal region at function call.',
            debug_root_node=None
        )

    root = StateNode(
        id=id_provider.get_id(),
        state_stamped=request.start_state,
        node_cost=0.0,
        heuristic_cost=0.0,
        path_cost=0.0,
        total_cost=0.0,
        detailed_costs=None,
        goal_region_reached=False,
        node_depth=0,
        parent=None
    )
    
    heapq.heappush(open_nodes_pq, root)

    r_v = get_signed_magnitude(root.state_stamped.state.velocity, root.state_stamped.state.yaw)
    r_a = get_signed_magnitude(root.state_stamped.state.acceleration, root.state_stamped.state.yaw)
    root_key = (
        root.state_stamped.timestamp,
        int(root.state_stamped.state.pos.x * inv_tol_pos),
        int(root.state_stamped.state.pos.y * inv_tol_pos),
        int(((root.state_stamped.state.yaw + math.pi) % (2 * math.pi)) * inv_tol_yaw),
        int(r_v * inv_tol_vel),
        int(r_a * inv_tol_acc)
    )
    visited_states.add(root_key)

    while True:
        loop_start_time = time.perf_counter()

        if not open_nodes_pq:
            break

        prev_node = heapq.heappop(open_nodes_pq)
        prev_stamped_state = prev_node.state_stamped

        motion_primitives = get_motion_primitives(
            velocity=get_signed_magnitude(prev_stamped_state.state.velocity, prev_stamped_state.state.yaw),
            steering_angle=prev_stamped_state.state.steering_angle,
            veh_cfg=cfg.vehicle,
            velocity_limit=request.target_speed,
            acceleration=get_signed_magnitude(prev_stamped_state.state.acceleration, prev_stamped_state.state.yaw),
            mp_cfg=cfg.motion_primitives,
            internal_dt=cfg.planner.dt_sim
        )
        
        for mp in motion_primitives:
            
            curr_state = kinematic_bicycle(
                stamped_state=prev_stamped_state, 
                control=EgoInput(mp.steering_angle, mp.acceleration), 
                dt=mp.dt, 
                vehicle_params=cfg.vehicle
            )
            
            c_pos = curr_state.state.pos
            c_yaw = curr_state.state.yaw
            c_v = get_signed_magnitude(curr_state.state.velocity, c_yaw)
            c_a = get_signed_magnitude(curr_state.state.acceleration, c_yaw)
            
            state_key = (
                curr_state.timestamp,
                int(c_pos.x * inv_tol_pos),
                int(c_pos.y * inv_tol_pos),
                int(((c_yaw + math.pi) % (2 * math.pi)) * inv_tol_yaw),
                int(c_v * inv_tol_vel),
                int(c_a * inv_tol_acc)
            )
            
            if state_key in visited_states:
                total_pruned_nodes += 1
                continue
                
            visited_states.add(state_key)
            total_generated_nodes += 1
            
            path_cost = prev_node.path_cost + prev_node.node_cost

            node_cost, detailed_costs = calculate_node_cost(
                prev_state=prev_stamped_state, curr_state=curr_state,
                request=request, cost_cfg=cfg.cost, veh_cfg=cfg.vehicle
            )
            
            heuristic_cost = calculate_heuristic_cost(
                state=curr_state, request=request, veh_cfg=cfg.vehicle, mp_cfg=cfg.motion_primitives
            )
            
            total_cost, detailed_costs = calculate_total_cost(
                path_cost=path_cost, node_cost=node_cost, heuristic_cost=heuristic_cost, 
                detailed_costs=detailed_costs, cost_cfg=cfg.cost
            )

            goal_region_reached = is_in_goal_region(curr_state, request.goal_region, cfg.vehicle)
            
            node_depth = prev_node.node_depth + 1
            max_depth_reached = max(max_depth_reached, node_depth)

            new_node = StateNode(
                id=id_provider.get_id(), state_stamped=curr_state,
                node_cost=node_cost, heuristic_cost=heuristic_cost, path_cost=path_cost,
                total_cost=total_cost, detailed_costs=detailed_costs,
                goal_region_reached=goal_region_reached, node_depth=node_depth,
                parent=prev_node, motion_primitive=mp
            )
            
            if not new_node.goal_region_reached and new_node.node_depth < max_depth:
                heapq.heappush(open_nodes_pq, new_node)
            else:
                heapq.heappush(end_nodes_pq, new_node)
                if ttfs_sec is None:
                    ttfs_sec = time.perf_counter() - call_time
        
        loop_stop_time = time.perf_counter()
        loop_duration = loop_stop_time - loop_start_time

        time_elapsed = loop_stop_time - call_time
        time_budget_sec = cfg.planner.max_compute_time / 1000.0
        extract_time_buffer_sec = cfg.planner.extract_path_time / 1000.0
        
        if (time_elapsed + loop_duration + extract_time_buffer_sec > time_budget_sec):
            break

    best_end_node = None
    if end_nodes_pq:
        best_end_node = heapq.heappop(end_nodes_pq)

    success = best_end_node is not None and not math.isinf(best_end_node.total_cost)

    path_length_raw = 0
    if success:
        path = extract_path_stamped_states(best_end_node)
        path_length_raw = len(path)

        rescaling_factor = int(cfg.planner.dt_output / cfg.planner.dt_sim)
        path = path[::rescaling_factor]
        trajectory = Trajectory(states=path)

        result = PlanResult(
            success=True,
            goal_region_reached=best_end_node.goal_region_reached,
            trajectory=trajectory,
            cost=best_end_node.total_cost,
            status_message=None,
            debug_root_node=root
        )
    else:
        status_message = f"End Node reached: {best_end_node is not None}"
        if best_end_node is not None:
            status_message += f" | Node details: {best_end_node.__repr__()}"

        result = PlanResult(
            success=False,
            goal_region_reached=None,
            trajectory=None,
            cost=None,
            status_message=status_message,
            debug_root_node=root
        )

    if debug:
        total_time_ms = (time.perf_counter() - call_time) * 1000.0
        ms_per_node = total_time_ms / total_generated_nodes if total_generated_nodes > 0 else 0.0
        efficiency_pct = (path_length_raw / total_generated_nodes) * 100 if total_generated_nodes > 0 else 0.0
        ttfs_str = f"{(ttfs_sec * 1000.0):.2f} ms" if ttfs_sec is not None else "N/A (Goal not reached)"

        total_attempted_nodes = total_generated_nodes + total_pruned_nodes
        pruned_pct = (total_pruned_nodes / total_attempted_nodes) * 100 if total_attempted_nodes > 0 else 0.0

        print("\n" + "="*50)
        print("PLANNER DEBUG STATISTICS")
        print("="*50)
        print(f"Time Budget       : {cfg.planner.max_compute_time:.2f} ms")
        print(f"Compute Time      : {total_time_ms:.2f} ms")
        print(f"TTFS              : {ttfs_str}")
        print(f"Speed             : {ms_per_node:.4f} ms / node")
        print("-" * 50)
        print(f"Nodes Attempted   : {total_attempted_nodes}")
        print(f"Nodes Generated   : {total_generated_nodes}")
        print(f"Nodes Pruned      : {total_pruned_nodes} ({pruned_pct:.1f} % saved)")
        print("-" * 50)
        print(f"Search Efficiency : {efficiency_pct:.2f} %")
        print(f"Max Depth Reached : {max_depth_reached} / {max_depth}")
        print(f"Success           : {success}")
        print("="*50 + "\n")

    return result          


def extract_path_stamped_states(end_node: StateNode) -> List[EgoStateStamped]:
    """
    Backtracks from the end node through parent links to extract the final trajectory sequence.

    Args:
        end_node (StateNode): The successful node in the goal region.

    Returns:
        List[EgoStateStamped]: Chronological sequence of stamped states.
    """
    node = end_node
    path: List[EgoStateStamped] = []

    while node is not None:
        path.append(node.state_stamped)
        node = node.parent

    path.reverse()
    return path


def is_in_goal_region(
    state_stamped: EgoStateStamped, 
    goal_region: GoalRegion, 
    veh_cfg: DictConfig
) -> bool:
    """
    Checks if the ego vehicle's bounding box intersects with the goal region's bounding box.

    Args:
        state_stamped (EgoStateStamped): The current state of the vehicle (position at rear axle).
        goal_region (GoalRegion): The target area defined by center, dimensions, and yaw.
        veh_cfg (DictConfig): The vehicle configuration containing dimensions (length, width, rear_to_wheel).

    Returns:
        bool: True if the vehicle's bounding box overlaps with the goal region, False otherwise.
    """
    
    ego_x = state_stamped.state.pos.x
    ego_y = state_stamped.state.pos.y
    ego_yaw = state_stamped.state.yaw
    
    axle_to_center = (veh_cfg.length / 2.0) - veh_cfg.rear_to_wheel
    
    ego_center_x = ego_x + axle_to_center * math.cos(ego_yaw)
    ego_center_y = ego_y + axle_to_center * math.sin(ego_yaw)
    
    ego_polygon = _create_oriented_bounding_box(
        center_x=ego_center_x, 
        center_y=ego_center_y, 
        width=veh_cfg.width, 
        length=veh_cfg.length, 
        yaw=ego_yaw
    )
    
    goal_polygon = _create_oriented_bounding_box(
        center_x=goal_region.center.x, 
        center_y=goal_region.center.y, 
        width=goal_region.width, 
        length=goal_region.length, 
        yaw=goal_region.yaw
    )
    
    return ego_polygon.intersects(goal_polygon)


def _create_oriented_bounding_box(
    center_x: float, 
    center_y: float, 
    width: float, 
    length: float, 
    yaw: float
) -> Polygon:
    """
    Helper function to create a shapely Polygon for an oriented bounding box.
    """
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    hw = width / 2.0
    hl = length / 2.0
    
    corners_local = [
        (hl, hw),   
        (hl, -hw),  
        (-hl, -hw), 
        (-hl, hw)   
    ]
    
    corners_global = []
    for lx, ly in corners_local:
        gx = center_x + (lx * cos_y - ly * sin_y)
        gy = center_y + (lx * sin_y + ly * cos_y)
        corners_global.append((gx, gy))
        
    return Polygon(corners_global)