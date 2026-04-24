from .environment import environment
from math import cos, sin
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


    def _get_bbox_corners(self, x: float, y: float, yaw: float, length: float, width: float):
        """Compute the corners of a bounding box given its center position, orientation, length, and width."""

        return [
            Vector2D(
                x = x + (length / 2) * cos(yaw) - (width / 2) * sin(yaw),
                y = y + (length / 2) * sin(yaw) + (width / 2) * cos(yaw)
            ),
            Vector2D(
                x = x + (length / 2) * cos(yaw) + (width / 2) * sin(yaw),
                y = y + (length / 2) * sin(yaw) - (width / 2) * cos(yaw)
            ),
            Vector2D(
                x = x - (length / 2) * cos(yaw) + (width / 2) * sin(yaw),
                y = y - (length / 2) * sin(yaw) - (width / 2) * cos(yaw)
            ),
            Vector2D(
                x = x - (length / 2) * cos(yaw) - (width / 2) * sin(yaw),
                y = y - (length / 2) * sin(yaw) + (width / 2) * cos(yaw)
            )
        ]


    def _check_line_intersection(self, p1: Vector2D, p2: Vector2D, p3: Vector2D, p4: Vector2D):
        """Check if the line segments p1p2 and p3p4 intersect."""

        # direction vectors
        d1 = Vector2D(x = p2.x - p1.x, y = p2.y - p1.y)
        d2 = Vector2D(x = p4.x - p3.x, y = p4.y - p3.y)

        # determinant of the direction vectors
        det = d1.x * d2.y - d1.y * d2.x

        # no intersection
        if det == 0:
            return False

        # parameters of the intersection point
        t = ((p3.x - p1.x) * d2.y - (p3.y - p1.y) * d2.x) / det
        u = ((p3.x - p1.x) * d1.y - (p3.y - p1.y) * d1.x) / det

        return (0 <= t <= 1) and (0 <= u <= 1)


    def _check_collision_object(
        self,
        ego_state: EgoStateStamped,
        obj: DynamicObjectStamped
    ):
        """Check if the ego vehicle collides with the given dynamic object."""

        ego_width = VehicleParameters.WIDTH
        ego_length = VehicleParameters.LENGTH
        ego_x = ego_state.state.pos.x
        ego_y = ego_state.state.pos.y
        ego_yaw = ego_state.state.yaw

        ego_corners = self._get_bbox_corners(ego_x, ego_y, ego_yaw, ego_length, ego_width)

        obj_width = obj.state.width
        obj_length = obj.state.length
        obj_x = obj.state.pos.x
        obj_y = obj.state.pos.y
        obj_yaw = obj.state.yaw

        obj_corners = self._get_bbox_corners(obj_x, obj_y, obj_yaw, obj_length, obj_width)

        # check if any boundary of the ego bounding box intersects with any boundary of the object bounding box
        for i in range(4):
            for j in range(4):
                if self._check_line_intersection(
                    ego_corners[i],
                    ego_corners[(i + 1) % 4],
                    obj_corners[j],
                    obj_corners[(j + 1) % 4]
                ):
                    return True
        return False

    #TODO: implement lane collision checking
    def _check_collision_lane(
        self,
        ego_state: EgoStateStamped,
        lane: Lane
    ): 
        ...

    def _is_collision_free(
        self,
        ego_state: EgoStateStamped,
        env: PredictedEnvironment
    ):
        """Check if the ego state is collision-free with objects and lanes."""

        for obj in env.dynamic_objects:
            if self._check_collision_object(ego_state, obj):
                return False
        
        #TODO: implement current lane tracking
        for lane in env.lanes:
            if self._check_collision_lane(ego_state, lane):
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
