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
    beta: float = 0.0
    yaw_rate: float
    steer: float
    timestamp: float = 0.0

def bicycle_model(
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
    v_new = max(state.v + acc_clamped * dt, 0.0) # Ensure velocity doesn't go negative

    if v_new > 1e-3:
        Cf, Cr   = params.Cf, params.Cr
        lf, lr   = params.Lf, params.Lr
        m, Iz, v = params.m, params.Iz, v_new

        #Linear state space 
        A11 = -(Cf + Cr) / (m * v)
        A12 = (-lf * Cf + lr * Cr) / (m * v**2) - 1.0
        A21 = (-lf * Cf + lr * Cr) / Iz
        A22 = -(lf**2 * Cf + lr**2 * Cr) / (Iz * v)

        # B vector
        B1 = Cf / (m * v)
        B2 = lf * Cf / Iz

        psi_dot = (v / params.L) * np.tan(steer_new)
        beta    = 0.0

        # Derivates
        beta_dot = A11 * beta + A12 * psi_dot + B1 * steer_new
        psi_ddot = A21 * beta + A22 * psi_dot + B2 * steer_new

        # Integrate
        beta_new = beta + beta_dot * dt
        psi_dot_new = psi_dot + psi_ddot * dt
    else:
        beta_new = 0.0
        psi_dot_new = 0.0
    
    yaw_new = state.yaw + psi_dot_new * dt
    x_new   = state.x + v_new * np.cos(beta_new + state.yaw) * dt
    y_new   = state.y + v_new * np.sin(beta_new + state.yaw) * dt

    return EgoStateStamped(
        x=x_new,
        y=y_new,
        yaw=yaw_new,
        v=v_new,
        steer=steer_new,
        timestamp=state.timestamp + dt
    )

class Vehicle:
    def __init__(self, x, y, yaw, v):

        self.length        = 2.338
        self.width         = 1.381
        self.rear_to_wheel = 0.339
        self.wheel_legth   = 0.531
        self.wheel_width   = 0.125
        self.track         = 1.094
        self.wheel_base    = 1.686

        # State
        self.x    = x
        self.y    = y
        self.yaw  = yaw
        self.v    = v
        self.steer = 0.0
        self.lat_acc = 0.0

        # Dynamics
        self.Caf  = 2 * 32857.5
        self.Car  = 2 * 32857.5
        self.mass = 1500.0
        self.lf   = 0.9442
        self.lr   = 0.7417
        self.Iz   = 430.166

        # Build Vehicle Parameters from the above
        self.params = VehicleParameters(
            max_steer         = 0.6,    # ~34 degrees like a typical car
            max_steer_rate    = 0.5,    # rad/s
            L                 = self.wheel_base,
            Lf                = self.lf,
            Lr                = self.lr,
            width             = self.width,
            length            = self.length,
            m                 = self.mass,
            Iz                = self.Iz,
            Cf                = self.Caf,
            Cr                = self.Car,
            max_acceleration  = 3.0,    #measured m/s²
            max_deceleration  = 5.0,    #measured m/s²
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
