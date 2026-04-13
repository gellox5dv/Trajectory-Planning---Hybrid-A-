from dataclasses import dataclass
from typing import List, Dict, Optional
from enum import Enum


# ============================================================
# Basic Geometry
# ============================================================

@dataclass
class Vector2D:
    x: float                    # x-coordinate [m]
    y: float                    # y-coordinate [m]


# ============================================================
# Vehicle Model
# ============================================================

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


# ============================================================
# Ego Vehicle State (Planning)
# ============================================================

@dataclass
class EgoState:
    pos: Vector2D               # position (x, y) [m]
    yaw: float                  # heading angle [rad]
    velocity: float             # forward velocity [m/s]


@dataclass
class EgoStateStamped:
    timestamp: int              # time [ms]
    state: EgoState             # ego state at this time


# ============================================================
# Control Input 
# ============================================================

@dataclass
class EgoInput:
    steering_angle: float       # steering input δ [rad]
    acceleration: float         # longitudinal acceleration [m/s²]


# ============================================================
# Dynamic Objects 
# ============================================================

class ObjectType(Enum):
    VEHICLE = 1
    PEDESTRIAN = 2
    CYCLIST = 3
    STATIC = 4


@dataclass
class DynamicObject:
    id: int
    obj_class: ObjectType       # semantic class of object

    pos: Vector2D               # position [m]
    yaw: float                  # orientation [rad]

    velocity: Vector2D          # velocity (vx, vy) [m/s]
    acceleration: Vector2D      # acceleration (ax, ay) [m/s²]

    width: float                # bounding box width [m]
    length: float               # bounding box length [m]


@dataclass
class DynamicObjectStamped:
    timestamp: int              # time [ms]
    state: DynamicObject        # object state at this time


# ============================================================
# Dynamic State (for Tracking)
# ============================================================

@dataclass
class DynamicState:
    pos: Vector2D               # position [m]
    yaw: float                  # heading [rad]
    velocity: Vector2D          # (vx, vy) [m/s]
    yaw_rate: float             # yaw rate [rad/s]


# ============================================================
# Lane Representation
# ============================================================

@dataclass
class Lane:
    id: int                     # unique lane identifier

    centerline: List[Vector2D]  # ordered centerline points [m]

    width: float                # total lane width [m] (± width/2 from centerline)

    speed_limit: float          # speed limit [m/s]

    cumulative_lengths: List[float]
    # arc length s at each centerline point [m]
    # used for fast longitudinal position computation


# ============================================================
# Environment (Prediction Output)
# ============================================================

@dataclass
class Environment:
    objects: Dict[int, List[DynamicObjectStamped]]
    # mapping: object_id → predicted states over time

    lanes: List[Lane]           # static road geometry

    dt: int                     # time resolution of prediction [ms]
    horizon: int                # prediction horizon [ms]


# ============================================================
# Goal Definition
# ============================================================

@dataclass
class GoalRegion:
    center: Vector2D            # center position [m]

    length: float               # longitudinal size [m]
    width: float                # lateral size [m]

    yaw: float                  # desired heading [rad]
    yaw_tolerance: float        # allowed deviation [rad]

    target_velocity: float      # desired speed [m/s]
    velocity_tolerance: float   # allowed speed deviation [m/s]


# ============================================================
# Planner Interface
# ============================================================

@dataclass
class PlanningRequest:
    start_state: EgoState       # initial state for planning 

    goal_region: GoalRegion     # target region

    vehicle_params: VehicleParameters
    environment: Environment

    horizon: int                # planning horizon [ms]

    dt: int                     # internal integration step [ms]
    output_dt: int              # output resolution (multiple of dt) [ms]

    max_compute_time: int       # time budget [ms]


# ============================================================
# Planner Output
# ============================================================

@dataclass
class Trajectory:
    states: List[EgoStateStamped]   # time-parameterized ego states


@dataclass
class PlanResult:
    success: bool                   # True if valid plan found

    trajectory: Optional[Trajectory]  # resulting trajectory (None if failed)

    cost: float                     # total trajectory cost

    status_message: str             # diagnostic info 