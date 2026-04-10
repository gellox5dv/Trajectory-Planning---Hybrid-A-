from dataclasses import dataclass
from typing import Tuple

@dataclass
class EgoGeometry:
        max_steer: float            # maximum steering angle [rad]
        L: float                    # wheelbase [m]
        Lf: float                   # center of gravity to front axle [m]
        Lr: float                   # center of gravity to rear axle [m]
        width: float                # vehicle width [m]
        length: float               # vehicle length [m]
        dt: float                   # time step [s]
        m: float                    # mass [kg]
        Iz: float                   # yaw inertia [kg·m²]
        Cf: float                   # front cornering stiffness [N/rad]
        Cr: float                   # rear cornering stiffness [N/rad]

        #h                           #height of mass point

@dataclass
class EgoState:
    pos: Tuple[float, float]            # position (x, y) [m]
    yaw: float                          # heading angle [rad]
    velocity: Tuple[float, float]       # vehicle velocity [m/s]

@dataclass
class EgoStateStamped:
    timestamp: int                  # discrete timestamp in [ms]
    vehicle_state: EgoState         # associated vehicle state


@dataclass
class EgoInput:
    steering_angle: float           # steering angle input [rad]
    acceleration: float             # longitudinal acceleration [m/s²]


###################


@dataclass
class VehicleState:
    pos: Tuple[float, float]            # position (x, y) [m]
    yaw: float                          # heading angle [rad]
    velocity: Tuple[float, float]       # vehicle velocity [m/s]
    acceleration: Tuple[float, float]   # vehicle acceleration [m/s]


@dataclass
class VehicleStateStamped:
    timestamp: int                  # discrete timestamp in [ms]
    vehicle_state: VehicleState     # associated vehicle state



