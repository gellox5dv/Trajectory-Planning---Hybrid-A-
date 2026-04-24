import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict


@dataclass
class VehicleParameters:
    max_steer: float            # maximum steering angle [rad]
    max_steer_rate: float       # maximum steering rate [rad/s]

    L: float                    # wheelbase [m]
    Lf: float                   # CoG to front axle [m]
    Lr: float                   # CoG to rear axle [m]

    width: float                # vehicle width [m]
    length: float               # vehicle length [m]

    m: float                    # mass [kg]
    Iz: float                   # yaw inertia [kg·m²]

    Cf: float                   # front cornering stiffness [N/rad]
    Cr: float                   # rear cornering stiffness [N/rad]

    max_acceleration: float     # max longitudinal acceleration [m/s²]
    max_deceleration: float     # max braking deceleration [m/s²]

    mu: float                   # tire-road friction coefficient [-]
@dataclass
class EgoStateStamped:
    x: float
    y: float
    yaw: float
    v: float
    steer: float
    timestamp: float = 0.0

@dataclass
class EgoInput:
    acceleration: float
    steer_rate: float

@dataclass
class DynamicState:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float
    steer: float
    timestamp: float = 0.0

def nonlinear_bicycle_model(
    state: EgoStateStamped,
    control: EgoInput,
    params: VehicleParameters,
    dt: float
) ->EgoStateStamped:
    """
    Computes the next state using a nonlinear kinematic bicycle model.

    Responsibilities:
    - propagate the ego state forward in time
    - respect vehicle geometry (wheelbase, steering limits)

    This model is a simplification of real vehicle dynamics and is used
    inside the planner due to its computational efficiency.
    """

    #Streering angle

    # Update steering angle with rate limiting
    steer_new = state.steer + control.steer_rate * dt
    steer_new = np.clip(steer_new, -params.max_steer, params.max_steer)
    # Update velocity with acceleration limits
    acc_clamped = np.clip(control.acceleration, -params.max_deceleration, params.max_acceleration)
    v_new = state.v + acc_clamped * dt
    v_new = max(v_new, 0.0)  # Ensure velocity doesn't go negative

    # Tire Slip angle. Fomurlas used from "https://arxiv.org/pdf/2306.04857"

        # Formulas:
        # alpha_f ≈ steer - arctan(Lf * yaw_rate / vx)   [front tire]
        # alpha_r ≈ -arctan(Lr * yaw_rate / vx)         [rear tire]
    if abs(v_new) > 1e-6:
        # Update yaw angle using kinematic bicycle model
        yaw_rate_approx = (v_new / params.L) * np.tan(steer_new)
    
        alpha_f = steer_new - np.arctan2 (
            params.Lf * yaw_rate_approx, v_new
        )
        alpha_r = -np.arctan2 (
            params.Lr * yaw_rate_approx, v_new
        )
    else:
        alpha_f = 0.0
        alpha_r = 0.0
    beta = np.arctan(
        (-params.Lf * np.tan(alpha_r) + params.Lr * np.tan(steer_new))
        / (params.Lf + params.Lr)
    )

    # Side Slip angle beta

        # Formula:
        # β = arctan((-Lf*tan(αr) + Lr*tan(δ - αf)) / (Lf + Lr))
    beta = np.arctan(
        (-params.Lf * np.tan(alpha_r) + params.Lr * np.tan(steer_new - alpha_f))
        / (params.Lf + params.Lr)
    )

    # Yaw rate

        # Fomula:
        # ψ_ = V*cos(β) * (tan(δ - αf) + tan(αr)) / (Lf + Lr)
    yaw_rate = (
        v_new * np.cos(beta) * (np.tan(steer_new - alpha_f) + np.tan(alpha_r))
        / (params.Lf + params.Lr)
    )
    yaw_new = state.yaw + yaw_rate * dt

    # Position

        # Formulas:
        # X_ = V * cos(β + ψ)
        # Y_ = V * sin(β + ψ)
    x_new = state.x + v_new * np.cos(beta + state.yaw) * dt
    y_new = state.y + v_new * np.sin(beta + state.yaw) * dt
    # Update timestamp
    timestamp_new = state.timestamp + dt
    return EgoStateStamped(
        x=x_new,
        y=y_new,
        yaw=yaw_new,
        v=v_new,
        steer=steer_new,
        timestamp=timestamp_new
    )

class DynamicBicycleModel:

    def __init__(self, params: VehicleParameters) -> None:
        """
        Initializes the dynamic bicycle model.

        Responsibilities:
        - store vehicle parameters (mass, inertia, tire stiffness, etc.)
        - prepare internal variables required for dynamic simulation

        This model represents the physical behavior of the vehicle more accurately
        than the kinematic model and is used for control and tracking.
        """
        self.params = params

    def step(
        self,
        state: DynamicState,
        control: EgoInput,
        dt: float
    ) -> DynamicState:
        """
        Propagates the vehicle state using a dynamic bicycle model.

        Responsibilities:
        - compute longitudinal and lateral motion
        - update yaw rate and vehicle orientation
        - account for tire forces and vehicle inertia

        Inputs:
        - state: current dynamic state (includes velocities and yaw rate)
        - control: steering and acceleration input
        - dt: time step in seconds

        Output:
        - next dynamic state after dt

        This model is used in the control module to accurately
        follow the planned trajectory and capture real vehicle behavior.
        """
        # Update steering angle
        steer_new = state.steer + control.steer_rate * dt
        
        # Compute slip angles
        if abs(state.vx) > 1e-6:  # Avoid division by zero
            alpha_f = steer_new - np.arctan2(state.vy + self.params.Lf * state.yaw_rate, state.vx)
            alpha_r = -np.arctan2(state.vy - self.params.Lr * state.yaw_rate, state.vx)
        else:
            alpha_f = 0.0
            alpha_r = 0.0
        
        # Compute tire lateral forces
        F_yf = self.params.Cf * alpha_f
        F_yr = self.params.Cr * alpha_r
        
        # Longitudinal acceleration from control input
        ax = control.acceleration
        
        # Lateral acceleration from tire forces
        ay = (F_yf + F_yr) / self.params.m
        
        # Yaw acceleration from tire forces and geometry
        yaw_ddot = (self.params.Lf * F_yf - self.params.Lr * F_yr) / self.params.Iz
        
        # Update velocities
        vx_new = state.vx + (ax - state.vy * state.yaw_rate) * dt
        vy_new = state.vy + (ay + state.vx * state.yaw_rate) * dt
        yaw_rate_new = state.yaw_rate + yaw_ddot * dt
        
        # Update position and orientation
        x_new = state.x + (state.vx * np.cos(state.yaw) - state.vy * np.sin(state.yaw)) * dt
        y_new = state.y + (state.vx * np.sin(state.yaw) + state.vy * np.cos(state.yaw)) * dt
        yaw_new = state.yaw + state.yaw_rate * dt
        
        # Update timestamp
        timestamp_new = state.timestamp + dt
        
        return DynamicState(
            x=x_new,
            y=y_new,
            yaw=yaw_new,
            vx=vx_new,
            vy=vy_new,
            yaw_rate=yaw_rate_new,
            steer=steer_new,
            timestamp=timestamp_new
        )
