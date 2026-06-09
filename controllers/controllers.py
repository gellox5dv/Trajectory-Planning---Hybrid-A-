from __future__ import annotations

import casadi as ca
import math
import numpy as np

from typing import Tuple
from omegaconf import DictConfig

from models.models import EgoStateStamped, Trajectory
from utils.helper import get_magnitude


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

        self.max_acc = float(vehicle_params.max_acceleration)
        self.max_dec = float(vehicle_params.max_deceleration)

        self.max_delta = float(vehicle_params.max_steer)
        self.max_delta_rate = float(vehicle_params.max_steer_rate)

        self.Q_x = float(controller_cfg.get("Q_x", 5.0))
        self.Q_y = float(controller_cfg.get("Q_y", 5.0))
        self.Q_yaw = float(controller_cfg.get("Q_yaw", 10.0))
        self.Q_v = float(controller_cfg.get("Q_v", 1.0))
        self.Q_delta = float(controller_cfg.get("Q_delta", 0.1))

        self.R_a = float(controller_cfg.get("R_a", 0.1))
        self.R_sr = float(controller_cfg.get("R_sr", 0.1))

        self.R_da = float(controller_cfg.get("R_da", 1.0))
        self.R_dsr = float(controller_cfg.get("R_dsr", 1.0))

        self._prev_X = None
        self._prev_U = None

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
                self.R_a * a ** 2
                + self.R_sr * delta_rate ** 2
            )

            if k > 0:
                du = U[:, k] - U[:, k - 1]
                cost += (
                    self.R_da * du[0] ** 2
                    + self.R_dsr * du[1] ** 2
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

        x0 = np.array([px, py, yaw, v, delta])

        ref = np.zeros((5, self.N + 1))
        unwrapped_yaws = self._unwrap_reference_yaws(trajectory)

        for k in range(self.N + 1):
            idx = min(k, len(trajectory.states) - 1)
            s = trajectory.states[idx].state

            ref[:, k] = [
                float(s.pos.x),
                float(s.pos.y),
                float(unwrapped_yaws[idx]),
                float(get_magnitude(s.velocity)),
                float(getattr(s, "steering_angle", 0.0)),
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