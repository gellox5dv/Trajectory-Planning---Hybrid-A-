from .environment import environment
from math import hypot, sin, cos, tan
from omegaconf import DictConfig
from models.models import EgoStateStamped, EgoState, Environment, Vector2D, GoalRegion
from utils.helper import get_vector, get_nearest_lane_center


class Simulation:
    
    ego_state = EgoStateStamped(
        timestamp = 0,
        state = EgoState(
            pos = Vector2D(x = 50.0, y = 2.0),
            yaw = 0.0,
            velocity = get_vector(13.0 + 8/9, 0.0),
            acceleration = get_vector(0.0, 0.0),
            steering_angle = 0.0
        )
    )

    curr_env = environment

    
    def get_ego_state(self) -> EgoStateStamped:
        return self.ego_state
    

    def get_environment(self) -> Environment:
        return self.curr_env
    

    def apply_control(self, acc: float, steer_rate: float, cfg: DictConfig) -> None:
        ...


    def step(self, dt: float) -> None:
        ...
    
    
    def get_goal_region(
            self,
            horizon: int,
            length: float,
            width: float,
        ) -> GoalRegion:
        """
        Get the goal region which is horizon seconds ahead in the same lane as the ego vehicle.

        Args:
            horizon (float): The time horizon to look ahead [ms].
            length (float): The length of the goal region [m].
            width (float): The width of the goal region [m].

        Returns:
            GoalRegion: The computed goal region.
        """

        nearest_lane_yaw, nearest_lane_center = get_nearest_lane_center(self.get_ego_state(), self.curr_env.lanes)

        curr_vel = self.get_ego_state().state.velocity
        curr_vel_magnitude = hypot(curr_vel.x, curr_vel.y)

        distance_ahead = curr_vel_magnitude * horizon / 1000.0

        goal_center_x = nearest_lane_center.x + distance_ahead * cos(nearest_lane_yaw)
        goal_center_y = nearest_lane_center.y + distance_ahead * sin(nearest_lane_yaw)

        return GoalRegion(
            center=Vector2D(x=goal_center_x, y=goal_center_y),
            length=length,
            width=width,
            yaw=nearest_lane_yaw
        )





