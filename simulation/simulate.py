from .environment import create_environment
from omegaconf import DictConfig
from models.models import EgoStateStamped, EgoState, Environment, Vector2D
from utils.helper import get_vector
from motion.bicycle import DynamicBicycleModel
from motion.motion_prediction import predict_motion_constant_velocity


class Simulation:

    def __init__(self, cfg: DictConfig):
        self.ego_state = EgoStateStamped(
            timestamp = 0,
            state = EgoState(
                pos = Vector2D(x = 50.0, y = 2.0),
                yaw = 0.0,
                velocity = get_vector(13.0 + 8/9, 0.0),
                acceleration = get_vector(0.0, 0.0),
                steering_angle = 0.0
            )
        )

        self.curr_env = create_environment(cfg.scenario)
        self.bicycle_model = DynamicBicycleModel(self.ego_state.state, cfg.vehicle)

        self.ego_history = [self.ego_state]
        self.obj_history = [self.curr_env.objects]

    
    def get_ego_state(self) -> EgoStateStamped:
        return self.ego_state
    

    def get_environment(self) -> Environment:
        return self.curr_env
    

    def apply_control(self, acc: float, steer_rate: float) -> None:
        ...


    def step(self, acc: float, steer_rate: float, dt: int) -> None:
        self.ego_state.state = self.bicycle_model.step(acc, steer_rate, dt)
        self.ego_state.timestamp += dt
        self.ego_history.append(self.ego_state)

        self.curr_env.objects = predict_motion_constant_velocity(self.curr_env.objects, prediction_horizon=dt, dt=dt, last_only=True)
        self.obj_history.append(self.curr_env.objects)
