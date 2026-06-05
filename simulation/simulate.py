from .environment import create_environment
from math import atan2, cos, sin, pi
import copy
from omegaconf import DictConfig
from models.models import EgoStateStamped, EgoState, Environment, Vector2D
from utils.helper import get_vector
from motion.bicycle import DynamicBicycleModel
from motion.motion_prediction import predict_motion_constant_velocity
import numpy as np


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
        self.ego_noise_cfg = cfg.noise.ego
        self.bicycle_model = DynamicBicycleModel(self.ego_state.state, cfg.vehicle)

        self.ego_history = [self.ego_state]
        self.obj_history = [self.curr_env.objects]

    
    def get_ego_state(self, noise_level: float | None = None) -> EgoStateStamped:
        noisy_state = copy.deepcopy(self.ego_state)

        if noise_level is None:
            noise_level = float(self.ego_noise_cfg.noise_level)

        if noise_level <= 0.0 or self.ego_noise_cfg is None:
            return noisy_state

        pos_x_std = float(self.ego_noise_cfg.pos_x_std) * noise_level
        pos_y_std = float(self.ego_noise_cfg.pos_y_std) * noise_level
        yaw_std = float(self.ego_noise_cfg.yaw_std_deg) * noise_level * (pi / 180.0)
        velocity_std = float(self.ego_noise_cfg.velocity_std_kmh) * noise_level / 3.6
        acc_x_std = float(self.ego_noise_cfg.acceleration_x_std) * noise_level
        acc_y_std = float(self.ego_noise_cfg.acceleration_y_std) * noise_level

        noisy_state.state.pos.x += float(np.random.normal(0.0, pos_x_std))
        noisy_state.state.pos.y += float(np.random.normal(0.0, pos_y_std))
        noisy_state.state.yaw += float(np.random.normal(0.0, yaw_std))

        velocity = noisy_state.state.velocity
        velocity_magnitude = float(np.hypot(velocity.x, velocity.y))
        velocity_direction = atan2(velocity.y, velocity.x) if velocity_magnitude > 1e-12 else noisy_state.state.yaw
        velocity_magnitude = max(0.0, velocity_magnitude + float(np.random.normal(0.0, velocity_std)))
        noisy_state.state.velocity = get_vector(velocity_magnitude, velocity_direction)

        noisy_state.state.acceleration.x += float(np.random.normal(0.0, acc_x_std))
        noisy_state.state.acceleration.y += float(np.random.normal(0.0, acc_y_std))

        return noisy_state
    

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
