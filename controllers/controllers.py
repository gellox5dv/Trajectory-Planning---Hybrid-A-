from __future__ import annotations

import casadi as ca
import math
import numpy as np

from typing import Optional, Tuple
from omegaconf import DictConfig

from models.models import (
    EgoStateStamped,
    Trajectory,
)
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

        self.L = vehicle_params.wheel_base

        self.dt = controller_cfg.dt_sim / 1000.0
        self.N = int(math.ceil(controller_cfg.horizon / controller_cfg.dt_sim))

        self.max_acc = vehicle_params.max_acceleration
        self.max_dec = -abs(vehicle_params.max_deceleration)

        self.max_delta = vehicle_params.max_steer
        self.max_delta_rate = vehicle_params.max_steer_rate

        # tracking weights
        self.Qx = controller_cfg.get("Qx", 5.0)
        self.Qy = controller_cfg.get("Qy", 5.0)
        self.Qyaw = controller_cfg.get("Qyaw", 10.0)
        self.Qv = controller_cfg.get("Qv", 1.0)
        self.Qdelta = controller_cfg.get("Qdelta", 0.1)

        # control effort
        self.Ra = controller_cfg.get("Ra", 0.1)
        self.Rsr = controller_cfg.get("Rsr", 0.1)

        # input rate penalties
        self.Rda = controller_cfg.get("Rda", 1.0)
        self.Rdsr = controller_cfg.get("Rdsr", 1.0)

        self._build_solver()

    def _build_solver(self):

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

            px = x[0]
            py = x[1]
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

            e = X[:, k] - REF[:, k]

            cost += (
                self.Qx * e[0] ** 2
                + self.Qy * e[1] ** 2
                + self.Qyaw * e[2] ** 2
                + self.Qv * e[3] ** 2
                + self.Qdelta * e[4] ** 2
            )

            cost += (
                self.Ra * a**2
                + self.Rsr * delta_rate**2
            )

            if k > 0:

                du = U[:, k] - U[:, k - 1]

                cost += (
                    self.Rda * du[0] ** 2
                    + self.Rdsr * du[1] ** 2
                )

            opti.subject_to(a <= self.max_acc)
            opti.subject_to(a >= self.max_dec)

            opti.subject_to(
                delta_rate <= self.max_delta_rate
            )

            opti.subject_to(
                delta_rate >= -self.max_delta_rate
            )

            opti.subject_to(
                X[4, k] <= self.max_delta
            )

            opti.subject_to(
                X[4, k] >= -self.max_delta
            )

        terminal_error = X[:, self.N] - REF[:, self.N]

        cost += (
            self.Qx * terminal_error[0] ** 2
            + self.Qy * terminal_error[1] ** 2
            + self.Qyaw * terminal_error[2] ** 2
            + self.Qv * terminal_error[3] ** 2
        )

        opti.subject_to(X[:, 0] == X0)

        opti.minimize(cost)

        options = {
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.sb": "yes",
        }

        opti.solver("ipopt", options)

        self.opti = opti
        self.X0 = X0
        self.REF = REF
        self.U = U

    def compute_control(
        self,
        ego_state: EgoStateStamped,
        trajectory: Trajectory,
    ) -> Tuple[float, float]:

        px = ego_state.state.pos.x
        py = ego_state.state.pos.y

        yaw = ego_state.state.yaw
        v = get_magnitude(
            ego_state.state.velocity
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

        for k in range(self.N + 1):

            idx = min(
                k,
                len(trajectory.states) - 1,
            )

            s = trajectory.states[idx].state

            ref[:, k] = [
                s.pos.x,
                s.pos.y,
                s.yaw,
                get_magnitude(s.velocity),
                0.0,
            ]

        self.opti.set_value(
            self.X0,
            x0,
        )

        self.opti.set_value(
            self.REF,
            ref,
        )

        sol = self.opti.solve()

        u0 = sol.value(
            self.U[:, 0]
        )

        acceleration = float(u0[0])
        steering_rate = float(u0[1])

        return (
            acceleration,
            steering_rate,
        )