from .environment import environment
from models.models import EgoStateStamped, EgoState, EgoInput, Environment, Vector2D


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

    
    def get_ego_state(self) -> EgoStateStamped:
        return self.ego_state
    
    def get_environment(self) -> Environment:
        return self.curr_env
    
    def apply_control(self, control: EgoInput) -> None:
        ...
    
    def step(self, dt: float) -> None:
        ...
