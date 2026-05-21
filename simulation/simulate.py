from .environment import environment
from typing import Tuple
from models.models import (
    EgoStateStamped, EgoState, EgoInput, Environment, Trajectory, Vector2D, DynamicObjectStamped, 
    DynamicObject, VehicleParameters, PredictedEnvironment, Lane
)


class Simulation:
    
    ego_state = EgoStateStamped(
        timestamp = 0,
        state = EgoState(
            pos = Vector2D(x = 50.0, y = 2.0),
            yaw = 0.0,
            velocity = 13.0 + 8/9
        )
    )

    curr_env = environment

    
    def get_ego_state(self) -> EgoStateStamped:
        return self.ego_state
    

    def get_environment(self) -> Environment:
        return self.curr_env
    

    def apply_steer_rate(self, steer_rate: float) -> None:
        ...
    

    def apply_acceleration(self, acc: float) -> None:
        ...
    

    def step(self, dt: float) -> None:
        ...