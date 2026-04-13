from dataclasses import dataclass
from typing import Tuple
from enum import Enum
from typing import List


@dataclass
class VehicleParameters:
        max_steer: float            # maximum steering angle [rad]
        max_steer_rate: float       # maximum steering rate [rad/s]

        L: float                    # wheelbase [m]
        Lf: float                   # center of gravity to front axle [m]
        Lr: float                   # center of gravity to rear axle [m]

        width: float                # vehicle width [m]
        length: float               # vehicle length [m]

        m: float                    # mass [kg]
        Iz: float                   # yaw inertia [kg·m²]

        Cf: float                   # front cornering stiffness [N/rad]
        Cr: float                   # rear cornering stiffness [N/rad]

        max_acceleration: float     # maximum longitudinal acceleration [m/s²]
        max_deceleration: float     # maximum braking deceleration [m/s²]

        mu: float                   # tire-road friction coefficient [-]


@dataclass
class EgoState:
    pos: Vector2D                       # position (x, y) [m]
    yaw: float                          # heading angle [rad]
    velocity: float                     # vehicle velocity [m/s]

@dataclass
class EgoStateStamped:
    timestamp: int                  # discrete timestamp in [ms]
    vehicle_state: EgoState         # associated vehicle state

@dataclass
class EgoInput:
    steering_angle: float           # steering angle input [rad]
    acceleration: float             # longitudinal acceleration [m/s²]


@dataclass
class DynamicObject:
    id: int
    obj_class: ObjectType               # class of the Object
    pos: Vector2D                       # position (x, y) [m]
    yaw: float                          # heading angle [rad]
    velocity: Vector2D                  # vehicle velocity [m/s]
    acceleration: Tuple[float, float]   # vehicle acceleration [m/s]
    width: float                        # vehicle width [m]
    length: float                       # vehicle length [m]

@dataclass
class DynamicObjectStamped:
    timestamp: int                      # discrete timestamp in [ms]
    vehicle_state: DynamicObject        # associated vehicle state

@dataclass
class DynamicState:
    pos: Vector2D                       # position (x, y) [m]
    yaw: float                          # heading angle [rad]
    velocity: Vector2D                  # object velocity [m/s]
    yaw_rate: float                     # [rad/s]

class ObjectType(Enum):
    VEHICLE = 1
    PEDESTRIAN = 2
    CYCLIST = 3
    STATIC = 4


@dataclass
class PlanningRequest:
    start_state: EgoState               # initial state for planning (latency-compensated)
    goal_region: GoalRegion             # target region the planner should reach

    vehicle_params: VehicleParameters   # physical vehicle properties
    environment: Environment            # predicted dynamic objects

    horizon: int                        # planning horizon [ms]

    dt: int                             # internal integration step for motion primitives [ms]
    output_dt: float                    # trajectory output resolution (must be multiple of dt) [ms]

    max_compute_time: int               # max allowed planning time [ms]


@dataclass
class Environment:
    objects: dict[int, list[DynamicObjectStamped]]
    # mapping: object_id → predicted states over time

    lanes: list[Lane]           # static lane geometry

    dt: int                     # time step of predictions [ms]
    horizon: int                # prediction horizon [ms]

@dataclass
class Lane:
    id: int                     # unique lane identifier

    centerline: list[Vector2D]  # ordered centerline points [m]
    width: float                # total lane width [m] (± width/2 from centerline)

    speed_limit: float          # speed limit [m/s]

    cumulative_lengths: list[float]  # arc length s at each centerline point [m]

@dataclass
class GoalRegion:
    center: Vector2D          # center position of goal region [m]

    length: float             # longitudinal size along heading [m]
    width: float              # lateral size across heading [m]

    yaw: float                # desired orientation [rad]
    yaw_tolerance: float      # allowed yaw deviation [rad]

    target_velocity: float    # desired speed [m/s]
    velocity_tolerance: float # allowed speed deviation [m/s]

@dataclass
class PlanResult:
    success: bool                    # indicates if a valid plan was found
    trajectory: Trajectory           # resulting trajectory (None if failed)
    cost: float                      # total cost of the trajectory
    status_message: str              # diagnostic message 

@dataclass
class Trajectory:
    states: List[EgoStateStamped]

@dataclass
class Vector2D:
    x: float
    y: float



