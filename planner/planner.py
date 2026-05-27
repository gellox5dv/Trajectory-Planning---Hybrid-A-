from models.models import *
from anytree import NodeMixin
from anytree.search import findall
from motion_primitives import MotionPrimitive
from motion_primitives import get_motion_primitives
from state_node import StateNode
from shapely.geometry import Polygon
import heapq
import math
from omegaconf import DictConfig, OmegaConf
import time

from utils.helper import get_magnitude
from planner.cost.cost import calculate_node_cost, calculate_heuristic_cost, calculate_total_cost

class IDProvider:
    def __init__(self, start_id=0):
        self.current_id = start_id
        
    def get_id(self):
        result = self.current_id
        self.current_id += 1
        return result


def plan(request: PlanningRequest, cfg: DictConfig) -> PlanResult:
    """
    Computes a trajectory using a Hybrid A* search algorithm.

    This function expands a search tree from the vehicle's starting state towards a goal region. 
    It evaluates dynamically feasible motion primitives and prioritizes nodes based on 
    accumulated path costs and a heuristic estimate to the goal. The search is constrained 
    by physical limits, a maximum horizon depth, and a strict time budget.

    Args:
        request (PlanningRequest): The planning request containing the initial state, 
                                   the environment, the goal region, and target speed.
        cfg (DictConfig): The Hydra configuration object containing parameters for the 
                          planner, cost weights, motion primitives, and vehicle dynamics.

    Returns:
        PlanResult: An object containing the success flag, the final trajectory (if successful), 
                    the total cost, and a diagnostic status message.
    """
    
    # ---------------------------------------------------------
    # 1. Initialization
    # ---------------------------------------------------------
    open_nodes_pq: list[StateNode] = []
    end_nodes_pq: list[StateNode] = []
    id_provider = IDProvider()
    
    # Calculate the maximum allowable depth based on horizon and time step
    max_depth = math.ceil(cfg.planner.horizon / cfg.planner.dt_sim)
    call_time = time.perf_counter()

    # Immediate success check: Is the vehicle already inside the goal region?
    if is_in_goal_region(request.start_state, request.goal_region, cfg.vehicle):
        return PlanResult(
            success=False,
            goal_region_reached=None,
            trajectory=None,
            cost=None,
            status_message='Vehicle is already in the goal region at function call.'
        )

    # Initialize the root node of the search tree
    root = StateNode(
        id=id_provider.get_id(),
        state_stamped=request.start_state,
        node_cost=0.0,
        heuristic_cost=0.0,
        path_cost=0.0,
        total_cost=0.0,
        detailed_costs=None,
        goal_region_reached=False,
        depth=0,
        parent=None
    )
    
    # Push the root node onto the open priority queue
    heapq.heappush(open_nodes_pq, root)

    # ---------------------------------------------------------
    # 2. Main A* Search Loop
    # ---------------------------------------------------------
    while True:
        loop_start_time = time.perf_counter()

        # Terminate if there are no more nodes to explore
        if not open_nodes_pq:
            break

        # Pop the node with the lowest total cost f(n)
        prev_node = heapq.heappop(open_nodes_pq)
        prev_stamped_state = prev_node.state_stamped

        # Generate dynamically feasible motion primitives for the current state
        motion_primitives = get_motion_primitives(
            velocity=get_magnitude(prev_stamped_state.state.velocity),
            steering_angle=prev_stamped_state.state.steering_angle,
            veh_cfg=cfg.vehicle,
            velocity_limit=request.velocity_limit,
            acceleration=get_magnitude(prev_stamped_state.state.acceleration),
            mp_cfg=cfg.motion_primitives
        )
        
        # Expand the current node using the generated primitives
        for mp in motion_primitives:
            
            # TODO: Propagate state using the bicycle model
            # curr_state = bicycle_model(prev_stamped_state, mp, cfg.planner.dt_sim)
            curr_state = prev_stamped_state
            
            # Accumulated cost up to the start of this new edge
            path_cost = prev_node.path_cost + prev_node.node_cost

            # Calculate transition cost for the current edge (without heuristic)
            node_cost, detailed_costs = calculate_node_cost(
                prev_state=prev_stamped_state,
                curr_state=curr_state,
                request=request,
                cost_cfg=cfg.cost,
                veh_cfg=cfg.vehicle
            )
            
            # Estimate cost from the new state to the goal
            heuristic_cost = calculate_heuristic_cost(
                state=curr_state,
                request=request,
                veh_cfg=cfg.vehicle,
                max_a_lat=cfg.motion_primitives.max_a_lat
            )
            
            # Aggregate total weighted cost f(n) for the priority queue
            total_cost, detailed_costs = calculate_total_cost(
                path_cost=path_cost, 
                node_cost=node_cost, 
                heuristic_cost=heuristic_cost, 
                detailed_costs=detailed_costs, 
                cost_cfg=cfg.cost
            )

            # Check if this new state intersects with the goal region
            goal_region_reached = is_in_goal_region(curr_state, request.goal_region, cfg.vehicle)

            # Create the expanded child node
            new_node = StateNode(
                id=id_provider.get_id(),
                state_stamped=curr_state,
                node_cost=node_cost,
                heuristic_cost=heuristic_cost,
                path_cost=path_cost,
                total_cost=total_cost,
                detailed_costs=detailed_costs,
                goal_region_reached=goal_region_reached,
                depth=prev_node.depth + 1,
                parent=prev_node,
                motion_primitive=mp
            )
            
            # Sort the node into the appropriate queue based on termination conditions
            if not new_node.goal_region_reached and new_node.depth < max_depth:
                heapq.heappush(open_nodes_pq, new_node)
            else:
                # Goal reached or max depth exceeded; consider this an end node
                heapq.heappush(end_nodes_pq, new_node)
        
        # ---------------------------------------------------------
        # 3. Time Budget Evaluation
        # ---------------------------------------------------------
        loop_stop_time = time.perf_counter()
        loop_duration = loop_stop_time - loop_start_time

        # Check if the compute time budget (in seconds) has been exceeded
        time_elapsed = loop_stop_time - call_time
        time_budget_sec = cfg.planner.max_compute_time / 1000.0
        extract_time_buffer_sec = cfg.planner.extract_path_time / 1000.0
        
        if (time_elapsed + loop_duration + extract_time_buffer_sec > time_budget_sec):
            break

    # ---------------------------------------------------------
    # 4. Trajectory Extraction & Result Construction
    # ---------------------------------------------------------
    best_end_node = None

    # Retrieve the best trajectory end point if any were found
    if end_nodes_pq:
        best_end_node = heapq.heappop(end_nodes_pq)

    # A plan is successful if an end node exists and its cost is not infinite (e.g., no hard collisions)
    success = best_end_node is not None and not math.isinf(best_end_node.total_cost)

    if success:
        # Backtrack from the end node to the root to build the trajectory
        path = extract_path_stamped_states(best_end_node)

        # Downsample the trajectory to match the requested output resolution
        rescaling_factor = int(cfg.planner.dt_output / cfg.planner.dt_sim)
        path = path[::rescaling_factor]

        return PlanResult(
            success=True,
            goal_region_reached=best_end_node.goal_region_reached,
            trajectory=path,
            cost=best_end_node.total_cost,
            status_message=None
        )
    
    else:
        # Construct detailed failure diagnostics
        status_message = f"End Node reached: {best_end_node is not None}"
        if best_end_node is not None:
            status_message += f" | Node details: {best_end_node.__repr__()}"

        return PlanResult(
            success=False,
            goal_region_reached=None,
            trajectory=None,
            cost=None,
            status_message=status_message
        )                                 
                                 

def extract_path_stamped_states(end_node: StateNode) -> List[EgoStateStamped]:
    node = end_node
    path:List[EgoStateStamped] = []

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
    
    # --- 1. Construct Vehicle Polygon ---
    ego_x = state_stamped.state.pos.x
    ego_y = state_stamped.state.pos.y
    ego_yaw = state_stamped.state.yaw
    
    # Calculate distance from rear axle to the geometric center of the vehicle
    axle_to_center = (veh_cfg.length / 2.0) - veh_cfg.rear_to_wheel
    
    # Shift to geometric center
    ego_center_x = ego_x + axle_to_center * math.cos(ego_yaw)
    ego_center_y = ego_y + axle_to_center * math.sin(ego_yaw)
    
    # Create the vehicle bounding box polygon
    ego_polygon = _create_oriented_bounding_box(
        center_x=ego_center_x, 
        center_y=ego_center_y, 
        width=veh_cfg.width, 
        length=veh_cfg.length, 
        yaw=ego_yaw
    )
    
    # --- 2. Construct Goal Region Polygon ---
    
    goal_polygon = _create_oriented_bounding_box(
        center_x=goal_region.center.x, 
        center_y=goal_region.center.y, 
        width=goal_region.width, 
        length=goal_region.length, 
        yaw=goal_region.yaw
    )
    
    # --- 3. Check for Intersection ---
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
    
    # Half dimensions
    hw = width / 2.0
    hl = length / 2.0
    
    # The 4 corners relative to the center (unrotated)
    corners_local = [
        (hl, hw),   # Front Left
        (hl, -hw),  # Front Right
        (-hl, -hw), # Rear Right
        (-hl, hw)   # Rear Left
    ]
    
    # Rotate and translate corners to global coordinates
    corners_global = []
    for lx, ly in corners_local:
        gx = center_x + (lx * cos_y - ly * sin_y)
        gy = center_y + (lx * sin_y + ly * cos_y)
        corners_global.append((gx, gy))
        
    return Polygon(corners_global)