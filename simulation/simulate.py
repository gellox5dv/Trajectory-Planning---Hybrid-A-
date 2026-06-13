<<<<<<< HEAD
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
=======
from .environment import create_environment
from math import atan2, pi
import copy
from omegaconf import DictConfig
from models.models import EgoStateStamped, EgoState, Environment, Vector2D, DynamicObjectStamped
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
        self.env_noise_cfg = cfg.noise.environment
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
        acc_std = float(self.ego_noise_cfg.acceleration_std) * noise_level

        noisy_state.state.pos.x += float(np.random.normal(0.0, pos_x_std))
        noisy_state.state.pos.y += float(np.random.normal(0.0, pos_y_std))
        noisy_state.state.yaw += float(np.random.normal(0.0, yaw_std))

        velocity = noisy_state.state.velocity
        velocity_magnitude = float(np.hypot(velocity.x, velocity.y))
        velocity_direction = atan2(velocity.y, velocity.x) if velocity_magnitude > 1e-12 else noisy_state.state.yaw
        velocity_magnitude = max(0.0, velocity_magnitude + float(np.random.normal(0.0, velocity_std)))
        noisy_state.state.velocity = get_vector(velocity_magnitude, velocity_direction)

        acc = noisy_state.state.acceleration
        acc_magnitude = float(np.hypot(acc.x, acc.y))
        acc_direction = atan2(acc.y, acc.x) if acc_magnitude > 1e-12 else noisy_state.state.yaw
        acc_magnitude = max(0.0, acc_magnitude + float(np.random.normal(0.0, acc_std)))
        noisy_state.state.acceleration = get_vector(acc_magnitude, acc_direction)

        return noisy_state
    

    def get_environment(self, noise_level: float | None = None) -> Environment:
        return self.get_noisy_environment(noise_level)


    def get_noisy_environment(self, noise_level: float | None = None) -> Environment:
        noisy_env = copy.deepcopy(self.curr_env)

        if noise_level is None:
            noise_level = float(self.env_noise_cfg.noise_level)

        if noise_level <= 0.0 or self.env_noise_cfg is None:
            return noisy_env

        pos_x_std = float(self.env_noise_cfg.pos_x_std) * noise_level
        pos_y_std = float(self.env_noise_cfg.pos_y_std) * noise_level
        yaw_std = float(self.env_noise_cfg.yaw_std_deg) * noise_level * (pi / 180.0)
        velocity_std = float(self.env_noise_cfg.velocity_std_kmh) * noise_level / 3.6
        acc_std = float(self.env_noise_cfg.acceleration_std) * noise_level

        for obj in noisy_env.objects:
            self._apply_object_noise(obj, pos_x_std, pos_y_std, yaw_std, velocity_std, acc_std)

        return noisy_env


    @staticmethod
    def _apply_object_noise(
        obj: DynamicObjectStamped,
        pos_x_std: float,
        pos_y_std: float,
        yaw_std: float,
        velocity_std: float,
        acc_std: float,
    ) -> None:
        obj.state.pos.x += float(np.random.normal(0.0, pos_x_std))
        obj.state.pos.y += float(np.random.normal(0.0, pos_y_std))
        obj.state.yaw += float(np.random.normal(0.0, yaw_std))

        velocity = obj.state.velocity
        velocity_magnitude = float(np.hypot(velocity.x, velocity.y))
        velocity_direction = atan2(velocity.y, velocity.x) if velocity_magnitude > 1e-12 else obj.state.yaw
        velocity_magnitude = max(0.0, velocity_magnitude + float(np.random.normal(0.0, velocity_std)))
        obj.state.velocity = get_vector(velocity_magnitude, velocity_direction)

        acc = obj.state.acceleration
        acc_magnitude = float(np.hypot(acc.x, acc.y))
        acc_direction = atan2(acc.y, acc.x) if acc_magnitude > 1e-12 else obj.state.yaw
        acc_magnitude = max(0.0, acc_magnitude + float(np.random.normal(0.0, acc_std)))
        obj.state.acceleration = get_vector(acc_magnitude, acc_direction)


    def apply_control(self, acc: float, steer_rate: float) -> None:
        ...


    def step(self, acc: float, steer_rate: float, dt: int) -> None:
        self.ego_state.state = self.bicycle_model.step(acc, steer_rate, dt)
        self.ego_state.timestamp += dt
        self.ego_history.append(self.ego_state)

        self.curr_env.objects = predict_motion_constant_velocity(self.curr_env.objects, prediction_horizon=dt, dt=dt, last_only=True)
        self.obj_history.append(self.curr_env.objects)
>>>>>>> feature/bicycle-only
