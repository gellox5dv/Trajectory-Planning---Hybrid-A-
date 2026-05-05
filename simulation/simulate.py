from .environment import environment
from math import cos, sin
from typing import Tuple
from .helper import get_bbox_corners, check_line_intersection, global_to_ego_axis
from models.models import (
    EgoStateStamped, EgoState, EgoInput, Environment, Vector2D, DynamicObjectStamped, 
    DynamicObject, VehicleParameters, PredictedEnvironment, Lane
)


class Simulation:
    
    ego_state = EgoStateStamped(
        timestamp = 0.0,
        state = EgoState(
            pos = Vector2D(x = 200.0, y = 2.0),
            yaw = 0.0,
            velocity = 50.0
        )
    )

    curr_env = environment


    def _get_x_y_yaw_from_state(self, state: EgoStateStamped | DynamicObjectStamped) -> Tuple[float, float, float]:
        return state.state.pos.x, state.state.pos.y, state.state.yaw


    def _check_collision_object(
        self,
        ego_state: EgoStateStamped,
        obj: DynamicObjectStamped,
        ego_params: VehicleParameters
    ) -> bool:
        """Check if the ego vehicle collides with the given dynamic object."""

        ego_x, ego_y, ego_yaw = self._get_x_y_yaw_from_state(ego_state)
        obj_x_g, obj_y_g, obj_yaw_g = self._get_x_y_yaw_from_state(obj)
        
        obj_x_ego, obj_y_ego, obj_yaw_ego = global_to_ego_axis(obj_x_g, obj_y_g, ego_x, ego_y, ego_yaw, obj_yaw_g)
        obj_corners = get_bbox_corners(obj_x_ego, obj_y_ego, obj_yaw_ego, obj.state.length, obj.state.width)

        ego_center_x = ego_params.length / 2 - ego_params.rear_to_wheel
        ego_center_y = 0.0
        ego_corners = get_bbox_corners(ego_center_x, ego_center_y, 0.0, ego_params.length, ego_params.width)

        for i in range(4):
            for j in range(4):
                if check_line_intersection(ego_corners[i], ego_corners[(i + 1) % 4], obj_corners[j], obj_corners[(j + 1) % 4]):
                    return True
        
        return False


    def _check_collision_lane(
        self,
        ego_state: EgoStateStamped,
        lane: Lane,
        ego_params: VehicleParameters
    ) -> bool:
        """Check if the ego vehicle collides with the lane boundaries."""

        ego_center_offset = ego_params.length / 2 - ego_params.rear_to_wheel
        ego_x, ego_y, ego_yaw = self._get_x_y_yaw_from_state(ego_state)
        ego_x += ego_center_offset * cos(ego_yaw)
        ego_y += ego_center_offset * sin(ego_yaw)

        ego_corners = get_bbox_corners(ego_x, ego_y, ego_yaw, ego_params.length, ego_params.width)

        f1 = False  # flag to indicate if atleast one centerline point is within 10 m of ego position

        for i in range(len(lane.centerline) - 1):
            p1, _ = lane.centerline[i]
            p2, _ = lane.centerline[i + 1]

            if (ego_x - p1.x) ** 2 + (ego_y - p1.y) ** 2 > 100.0:   # 10 m radius
                continue

            f1 = True

            lane_vec = Vector2D(x = p2.x - p1.x, y = p2.y - p1.y)
            lane_len = (lane_vec.x ** 2 + lane_vec.y ** 2) ** 0.5
            lane_unit = Vector2D(x = lane_vec.x / lane_len, y = lane_vec.y / lane_len)
            
            if lane_len == 0.0:
                continue
            
            for j in range(4):
                corner_vec = Vector2D(x = ego_corners[j].x - p1.x, y = ego_corners[j].y - p1.y)
                cross_prod = lane_unit.x * corner_vec.y - lane_unit.y * corner_vec.x    # perpendicular distance from centerpoint to ego corner
                if cross_prod > lane.width / 2 or cross_prod < -lane.width / 2:
                    return True

        if not f1:
            return True # offroad

        return False


    def _is_collision_free(
        self,
        ego_state: EgoStateStamped,
        env: PredictedEnvironment,
        ego_params: VehicleParameters
    ) -> bool:
        """Check if the ego state is collision-free with objects and lanes."""

        for obj in env.dynamic_objects:
            if self._check_collision_object(ego_state, obj, ego_params):
                return False
        
        #TODO: implement current lane tracking
        for lane in env.lanes:
            if self._check_collision_lane(ego_state, lane, ego_params):
                return False

        return True

    
    def get_ego_state(self) -> EgoStateStamped:
        return self.ego_state
    

    def get_environment(self) -> Environment:
        return self.curr_env
    

    def apply_control(self, control: EgoInput) -> None:
        ...
    

    def step(self, dt: float) -> None:
        ...
