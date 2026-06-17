from __future__ import annotations

import casadi as ca
import math
import numpy as np

from typing import Tuple
from omegaconf import DictConfig

from models.models import (
    EgoStateStamped,
    EgoState,
    Trajectory,
    Vector2D
)
from utils.helper import get_magnitude, get_signed_magnitude, get_vector


class MPCController:
    """
    Nonlinear MPC based on a kinematic bicycle model.

    State:
        [x, y, yaw, v, delta]

    Control:
        [acceleration, steering_rate]
    """

    def __init__(
        self,
        vehicle_params: DictConfig,
        controller_cfg: DictConfig,
    ):
        self.L = float(vehicle_params.wheel_base)

        self.dt = float(controller_cfg.dt) / 1000.0
        self.N = int(math.ceil(float(controller_cfg.horizon) / float(controller_cfg.dt)))

        self.dt = controller_cfg.dt / 1000.0
        self.N = int(math.ceil(controller_cfg.horizon / controller_cfg.dt))

        self.max_acc = vehicle_params.max_acceleration
        self.max_dec = -abs(vehicle_params.max_deceleration)

        self.max_delta = float(vehicle_params.max_steer)
        self.max_delta_rate = float(vehicle_params.max_steer_rate)

        self.Q_x = float(controller_cfg.get("Q_x", 5.0))
        self.Q_y = float(controller_cfg.get("Q_y", 5.0))
        self.Q_yaw = float(controller_cfg.get("Q_yaw", 10.0))
        self.Q_v = float(controller_cfg.get("Q_v", 1.0))
        self.Q_delta = float(controller_cfg.get("Q_delta", 0.1))

        # Tracking weights
        self.Qx = controller_cfg.get("Q_x", 5.0)
        self.Qy = controller_cfg.get("Q_y", 5.0)
        self.Qyaw = controller_cfg.get("Q_yaw", 10.0)
        self.Qv = controller_cfg.get("Q_v", 1.0)
        self.Qdelta = controller_cfg.get("Q_delta", 0.1)

        # Control effort penalties
        self.Ra = controller_cfg.get("R_a", 0.1)
        self.Rsr = controller_cfg.get("R_sr", 0.1)

        # Input rate penalties
        self.Rda = controller_cfg.get("R_da", 1.0)
        self.Rdsr = controller_cfg.get("R_dsr", 1.0)

        self._build_solver()

    def _build_solver(self) -> None:
        opti = ca.Opti()

        nx = 5
        nu = 2

        X = opti.variable(nx, self.N + 1)
        U = opti.variable(nu, self.N)

        X0 = opti.parameter(nx)
        REF = opti.parameter(nx, self.N + 1)

        cost = 0

        for k in range(self.N):
            x = X[:, k]
            u = U[:, k]

            yaw = x[2]
            v = x[3]
            delta = x[4]

            a = u[0]
            delta_rate = u[1]

            xdot = ca.vertcat(
                v * ca.cos(yaw),
                v * ca.sin(yaw),
                v / self.L * ca.tan(delta),
                a,
                delta_rate,
            )

            x_next = x + self.dt * xdot
            opti.subject_to(X[:, k + 1] == x_next)

            yaw_err = ca.atan2(
                ca.sin(X[2, k] - REF[2, k]),
                ca.cos(X[2, k] - REF[2, k]),
            )

            cost += (
                self.Q_x * (X[0, k] - REF[0, k]) ** 2
                + self.Q_y * (X[1, k] - REF[1, k]) ** 2
                + self.Q_yaw * yaw_err ** 2
                + self.Q_v * (X[3, k] - REF[3, k]) ** 2
                + self.Q_delta * (X[4, k] - REF[4, k]) ** 2
            )

            cost += (
                self.Ra * a ** 2
                + self.Rsr * delta_rate ** 2
            )

            if k > 0:
                du = U[:, k] - U[:, k - 1]
                cost += (
                    self.Rda * du[0] ** 2
                    + self.Rdsr * du[1] ** 2
                )

            opti.subject_to(a <= self.max_acc)
            opti.subject_to(a >= self.max_dec)

            opti.subject_to(delta_rate <= self.max_delta_rate)
            opti.subject_to(delta_rate >= -self.max_delta_rate)

            opti.subject_to(X[4, k] <= self.max_delta)
            opti.subject_to(X[4, k] >= -self.max_delta)

            opti.subject_to(X[3, k] >= 0.0)

        terminal_yaw_err = ca.atan2(
            ca.sin(X[2, self.N] - REF[2, self.N]),
            ca.cos(X[2, self.N] - REF[2, self.N]),
        )

        cost += (
            self.Q_x * (X[0, self.N] - REF[0, self.N]) ** 2
            + self.Q_y * (X[1, self.N] - REF[1, self.N]) ** 2
            + self.Q_yaw * terminal_yaw_err ** 2
            + self.Q_v * (X[3, self.N] - REF[3, self.N]) ** 2
            + self.Q_delta * (X[4, self.N] - REF[4, self.N]) ** 2
        )

        opti.subject_to(X[:, 0] == X0)
        opti.minimize(cost)

        options = {
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": 200,
        }

        opti.solver("ipopt", options)

        self.opti = opti
        self.X = X
        self.U = U
        self.X0 = X0
        self.REF = REF

    def _unwrap_reference_yaws(self, trajectory: Trajectory) -> list[float]:
        yaws = []
        prev_yaw = None

        for stamped in trajectory.states:
            yaw = float(stamped.state.yaw)

            if prev_yaw is not None:
                while yaw - prev_yaw > math.pi:
                    yaw -= 2.0 * math.pi
                while yaw - prev_yaw < -math.pi:
                    yaw += 2.0 * math.pi

            yaws.append(yaw)
            prev_yaw = yaw

        return yaws

    def _interpolate_trajectory(self, trajectory: Trajectory, target_time_ms: int) -> EgoState:
        """
        Interpolates the trajectory to return an exact EgoState for a given target timestamp.
        """
        states = trajectory.states
        
        # 1. Boundary check: Return first or last state if outside of trajectory timeframe
        if target_time_ms <= states[0].timestamp:
            return states[0].state
        if target_time_ms >= states[-1].timestamp:
            return states[-1].state
            
        # 2. Find the correct time slot for interpolation
        for i in range(len(states) - 1):
            s1 = states[i]
            s2 = states[i + 1]
            
            if s1.timestamp <= target_time_ms <= s2.timestamp:
                dt_total = s2.timestamp - s1.timestamp
                
                # Safety check to avoid division by zero
                if dt_total == 0:
                    return s1.state
                    
                ratio = (target_time_ms - s1.timestamp) / float(dt_total)
                
                # Spatial interpolation (x, y)
                x = s1.state.pos.x + ratio * (s2.state.pos.x - s1.state.pos.x)
                y = s1.state.pos.y + ratio * (s2.state.pos.y - s1.state.pos.y)
                
                # Yaw interpolation (with modulo to prevent 360-degree jumps)
                yaw_diff = (s2.state.yaw - s1.state.yaw + math.pi) % (2 * math.pi) - math.pi
                yaw = s1.state.yaw + ratio * yaw_diff
                
                # Velocity interpolation using signed magnitude
                v1 = get_signed_magnitude(s1.state.velocity, s1.state.yaw)
                v2 = get_signed_magnitude(s2.state.velocity, s2.state.yaw)
                v = v1 + ratio * (v2 - v1)
                
                return EgoState(
                    pos=Vector2D(x=x, y=y),
                    yaw=yaw,
                    velocity=get_vector(v, yaw),
                    acceleration=Vector2D(x=0.0, y=0.0),
                    steering_angle=0.0
                )
                
        return states[-1].state

    def compute_control(
        self,
        ego_state: EgoStateStamped,
        trajectory: Trajectory,
    ) -> Tuple[float, float]:
        if trajectory is None or len(trajectory.states) == 0:
            return 0.0, 0.0

        px = float(ego_state.state.pos.x)
        py = float(ego_state.state.pos.y)
        yaw = float(ego_state.state.yaw)
        v = float(get_magnitude(ego_state.state.velocity))
        delta = float(getattr(ego_state.state, "steering_angle", 0.0))

        yaw = ego_state.state.yaw
        v = get_signed_magnitude(
            ego_state.state.velocity, ego_state.state.yaw
        )

        delta = getattr(
            ego_state.state,
            "steering_angle",
            0.0,
        )

        x0 = np.array([
            px,
            py,
            yaw,
            v,
            delta,
        ])

        ref = np.zeros((5, self.N + 1))
        
        # Base time of the ego vehicle to start the prediction
        current_time_ms = ego_state.timestamp
        dt_ms = int(self.dt * 1000)

        for k in range(self.N + 1):
            # Calculate the exact target time for the current MPC node
            target_time_ms = current_time_ms + k * dt_ms
            
            # Interpolate the trajectory state for the target time
            s = self._interpolate_trajectory(trajectory, target_time_ms)

            ref[:, k] = [
                s.pos.x,
                s.pos.y,
                s.yaw,
                get_signed_magnitude(s.velocity, s.yaw),
                0.0,
            ]

        self.opti.set_value(self.X0, x0)
        self.opti.set_value(self.REF, ref)

        if self._prev_X is not None:
            self.opti.set_initial(self.X, self._prev_X)
        else:
            self.opti.set_initial(self.X, np.tile(x0.reshape(-1, 1), (1, self.N + 1)))

        if self._prev_U is not None:
            self.opti.set_initial(self.U, self._prev_U)
        else:
            self.opti.set_initial(self.U, np.zeros((2, self.N)))

        try:
            sol = self.opti.solve()

            x_sol = sol.value(self.X)
            u_sol = sol.value(self.U)

            self._prev_X = x_sol
            self._prev_U = np.hstack([u_sol[:, 1:], u_sol[:, -1:]])

            acceleration = float(u_sol[0, 0])
            steering_rate = float(u_sol[1, 0])

        except RuntimeError:
            acceleration = 0.0
            steering_rate = 0.0
            self._prev_X = None
            self._prev_U = None

        acceleration = max(self.max_dec, min(self.max_acc, acceleration))
        steering_rate = max(-self.max_delta_rate, min(self.max_delta_rate, steering_rate))

        return acceleration, steering_rate