import heapq
from math import pi, cos, sin
from utils.helper import get_bbox_corners, get_magnitude, get_vector
from typing import Tuple, List, Optional, overload, Union, Sequence
from models.models import (
    EgoState, EgoStateStamped, DynamicObject, DynamicObjectStamped,
    Lane, Environment, PredictedEnvironment, Vector2D
)
from shapely.geometry import Polygon
from omegaconf import DictConfig

MovingObject = Union[EgoStateStamped, DynamicObjectStamped]

def get_x_y_yaw_from_state(state: MovingObject) -> Tuple[float, float, float]:
    return state.state.pos.x, state.state.pos.y, state.state.yaw


def _ego_object_distance(
    ego_state: EgoStateStamped,
    obj: DynamicObjectStamped,
    ego_params: DictConfig
) -> float:
    """Calculate the distance from the ego vehicle to a dynamic object."""
    ego_x, ego_y, ego_yaw = get_x_y_yaw_from_state(ego_state)
    ego_x_center = ego_x + (ego_params.length / 2 - ego_params.rear_to_wheel) * cos(ego_yaw)
    ego_y_center = ego_y + (ego_params.length / 2 - ego_params.rear_to_wheel) * sin(ego_yaw)
    obj_x_center, obj_y_center, obj_yaw = get_x_y_yaw_from_state(obj)

    ego_corners = get_bbox_corners(ego_x_center, ego_y_center, ego_yaw, ego_params.length, ego_params.width)
    obj_corners = get_bbox_corners(obj_x_center, obj_y_center, obj_yaw, obj.state.length, obj.state.width)

    ego_polygon = Polygon([(corner.x, corner.y) for corner in ego_corners])
    obj_polygon = Polygon([(corner.x, corner.y) for corner in obj_corners])

    return ego_polygon.distance(obj_polygon)


@overload
def _create_interpolated_object(
    base: EgoStateStamped,
    new_timestamp: int,
    new_pos: Vector2D,
    new_yaw: float,
    new_velocity: float
) -> EgoStateStamped:
    ...


@overload
def _create_interpolated_object(
    base: DynamicObjectStamped,
    new_timestamp: int,
    new_pos: Vector2D,
    new_yaw: float,
    new_velocity: float
) -> DynamicObjectStamped:
    ...


def _create_interpolated_object(
    base: MovingObject,
    new_timestamp: int,
    new_pos: Vector2D,
    new_yaw: float,
    new_velocity: float
) -> MovingObject:

    vel_vector = get_vector(new_velocity, new_yaw)

    if isinstance(base, EgoStateStamped):
        return EgoStateStamped(
            timestamp=new_timestamp,
            state=EgoState(
                pos=new_pos,
                yaw=new_yaw,
                velocity=vel_vector,
                acceleration=Vector2D(0.0, 0.0),
                steering_angle=0.0,
            )
        )

    elif isinstance(base, DynamicObjectStamped):
        return DynamicObjectStamped(
            timestamp=new_timestamp,
            state=DynamicObject(
                id=base.state.id,
                obj_class=base.state.obj_class,
                pos=new_pos,
                yaw=new_yaw,
                velocity=vel_vector,
                acceleration=Vector2D(0.0, 0.0),
                width=base.state.width,
                length=base.state.length
            )
        )

    raise TypeError("Unsupported object type")


@overload
def _interpolate_states(
    start_state: EgoStateStamped,
    end_state: EgoStateStamped,
    new_resolution_ms: int
) -> Sequence[EgoStateStamped]:
    ...


@overload
def _interpolate_states(
    start_state: DynamicObjectStamped,
    end_state: DynamicObjectStamped,
    new_resolution_ms: int
) -> Sequence[DynamicObjectStamped]:
    ...


def _interpolate_states(
    start_state: MovingObject,
    end_state: MovingObject,
    new_resolution_ms: int
) -> Sequence[MovingObject]:
    """Interpolate the vehicle state from start to end state at the new resolution."""

    orig_resolution_ms = end_state.timestamp - start_state.timestamp
    if orig_resolution_ms <= new_resolution_ms:
        return [start_state, end_state]
    
    dx = (end_state.state.pos.x - start_state.state.pos.x)
    dy = (end_state.state.pos.y - start_state.state.pos.y)
    dvel = (get_magnitude(end_state.state.velocity) - get_magnitude(start_state.state.velocity))
    dyaw = (end_state.state.yaw - start_state.state.yaw + pi) % (2 * pi) - pi
    start_vel_magnitude = get_magnitude(start_state.state.velocity)

    interpolated_states = [start_state]
    t = 0

    while t < orig_resolution_ms:
        t += new_resolution_ms
        interp_x = start_state.state.pos.x + (t * dx / orig_resolution_ms)
        interp_y = start_state.state.pos.y + (t * dy / orig_resolution_ms)
        interp_yaw = start_state.state.yaw + (t * dyaw / orig_resolution_ms)
        interp_vel = start_vel_magnitude + (t * dvel / orig_resolution_ms)

        interpolated_state = _create_interpolated_object(
            base=start_state,
            new_timestamp=start_state.timestamp + t,
            new_pos=Vector2D(x=interp_x, y=interp_y),
            new_yaw=interp_yaw,
            new_velocity=interp_vel
        )
        interpolated_states.append(interpolated_state)

    interpolated_states.append(end_state)
    return interpolated_states


def get_distance_to_objects(
    current_ego: EgoStateStamped,
    previous_ego: EgoStateStamped,
    predicted_env: PredictedEnvironment,
    vehicle_params: DictConfig,
    resolution_ms: int
) -> Tuple[Optional[Sequence[Tuple[int, float]]], bool]:
    """Checks the interval between two planning steps for collisions and calculates aggregated distances to all objects."""
    
    interpolated_ego_states = _interpolate_states(previous_ego, current_ego, resolution_ms)

    # find dynamic objects from predicted environment that are relevant for collision checking in this interval
    relevant_objects = []
    for obj_id, obj_states in predicted_env.objects.items():
        relevant_objects.append([])
        for obj_state in obj_states:
            if previous_ego.timestamp <= obj_state.timestamp <= current_ego.timestamp:
                heapq.heappush(relevant_objects[-1], (obj_state.timestamp, obj_state))

    distance_results = []
    for obj_states in relevant_objects:
        if not obj_states:
            continue
        
        min_distance = float('inf')
        for ego_state in interpolated_ego_states:
            while obj_states and obj_states[0][0] < ego_state.timestamp:
                heapq.heappop(obj_states)
            if not obj_states:
                break
            
            obj_state = obj_states[0][1]
            distance = _ego_object_distance(ego_state, obj_state, vehicle_params)
            min_distance = min(min_distance, distance)

            if distance < 0.1:  # collision threshold
                return None, True
        
        distance_results.append((obj_state.state.id, min_distance))
    
    return distance_results, False



def _get_lane_occlusion(
    ego_state: EgoStateStamped,
    lane: Lane,
    vehicle_params: DictConfig
) -> float:
    """Calculate the area of the ego vehicle on the lane."""
    ego_x, ego_y, ego_yaw = get_x_y_yaw_from_state(ego_state)
    ego_center_offset = vehicle_params.length / 2 - vehicle_params.rear_to_wheel
    ego_x += ego_center_offset * cos(ego_yaw)
    ego_y += ego_center_offset * sin(ego_yaw)

    ego_corners = get_bbox_corners(ego_x, ego_y, ego_yaw, vehicle_params.length, vehicle_params.width)
    ego_polygon = Polygon([(corner.x, corner.y) for corner in ego_corners])

    lane_polygon_points = []
    for i in range(len(lane.centerline) - 1):
        p1, _ = lane.centerline[i]
        p2, _ = lane.centerline[i + 1]

        lane_vec = Vector2D(x = p2.x - p1.x, y = p2.y - p1.y)
        lane_len = (lane_vec.x ** 2 + lane_vec.y ** 2) ** 0.5
        if lane_len == 0.0:
            continue
        
        lane_unit = Vector2D(x = lane_vec.x / lane_len, y = lane_vec.y / lane_len)
        normal_vec = Vector2D(x = -lane_unit.y * lane.width / 2, y = lane_unit.x * lane.width / 2)

        lane_polygon_points.append((p1.x + normal_vec.x, p1.y + normal_vec.y))
        lane_polygon_points.append((p1.x - normal_vec.x, p1.y - normal_vec.y))
    
    lane_polygon = Polygon(lane_polygon_points)

    intersection_area = ego_polygon.intersection(lane_polygon).area
    return intersection_area


def _get_lane_offset(
    ego_state: EgoStateStamped,
    lane: Lane
) -> Tuple[float, float]:
    """Calculate the offset of the ego vehicle from the lane centerline."""
    ego_x, ego_y, ego_yaw = get_x_y_yaw_from_state(ego_state)

    min_distance = float('inf')
    for i in range(len(lane.centerline) - 1):
        p1, yaw1 = lane.centerline[i]
        p2, yaw2 = lane.centerline[i + 1]

        lane_vec = Vector2D(x = p2.x - p1.x, y = p2.y - p1.y)
        lane_len = (lane_vec.x ** 2 + lane_vec.y ** 2) ** 0.5
        if lane_len == 0.0:
            continue
        
        lane_unit = Vector2D(x = lane_vec.x / lane_len, y = lane_vec.y / lane_len)
        normal_vec = Vector2D(x = -lane_unit.y, y = lane_unit.x)

        distance_to_line = abs((ego_x - p1.x) * normal_vec.x + (ego_y - p1.y) * normal_vec.y)
        if distance_to_line < min_distance:
            min_distance = distance_to_line
            yaw_offset = (ego_yaw - (yaw1 + yaw2) / 2) % (2 * pi) - pi

    return min_distance, yaw_offset


def get_ego_lane_info(
    ego_state: EgoState,
    vehicle_cfg: DictConfig,
    lanes: List[Lane]
) -> Tuple[int, float, float, float, bool]:
    """
    Returns:
    lane_id: int,
    distance_to_lane_center: float [m]
    yaw_offset_to_lane_direction: float, [rad]
    lane_occlusion: float [e.g. 0.8]
    opposite_lane: bool
    """
    
    occlusion_sum = 0.0
    for lane in lanes:
        lane_occlusion = _get_lane_occlusion(EgoStateStamped(timestamp=0, state=ego_state), lane, vehicle_cfg)
        occlusion_sum += lane_occlusion
        if lane_occlusion > 0.5:
            distance_to_lane_center, yaw_offset = _get_lane_offset(EgoStateStamped(timestamp=0, state=ego_state), lane)
            opposite_lane = (yaw_offset > pi / 2 or yaw_offset < -pi / 2)
            return lane.id, distance_to_lane_center, yaw_offset, occlusion_sum, opposite_lane
    
    return -1, float('inf'), 0.0, occlusion_sum, False