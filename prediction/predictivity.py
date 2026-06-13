import math
import sys
from pathlib import Path
<<<<<<< HEAD
from typing import Dict, List, Optional
=======
from typing import Dict, List, Literal, Optional
>>>>>>> feature/bicycle-only

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from models.models import (
    DynamicObject,
    DynamicObjectStamped,
<<<<<<< HEAD
    EgoState,
=======
>>>>>>> feature/bicycle-only
    EgoStateStamped,
    Environment,
    PredictedEnvironment,
    Vector2D,
)

<<<<<<< HEAD
_YAW_RATE_MIN = 1e-9  # below this, treat turn rate as straight-line motion

"""
Scenario
  - Ego vehicle drives east at 10 m/s.
  - Lead vehicle is 40 m ahead of ego, also going east, but braking hard
    at -4 m/s² (it will stop before the prediction horizon ends).
  - Oncoming vehicle is 120 m ahead of ego, heading west at 12 m/s.
We run two sub-scenarios:
  A. Oncoming vehicle is far enough → overtake is SAFE.
  B. Oncoming vehicle is close      → overtake is NOT SAFE.
"""
=======
PredictionModel = Literal["constant_acceleration", "constant_turn_rate"]

# Guard for near-zero yaw rate in CTRV: below this, use straight-line fallback.
_YAW_RATE_MIN: float = 1e-4  # [rad/s]

# Guard for near-zero acceleration norm in braking clamp.
_ACCEL_NORM_SQ_MIN: float = 1e-9
>>>>>>> feature/bicycle-only

# Main role: check that the prediction horizon and time step can produce a valid timestamp grid.
def _validate_prediction_inputs(prediction_horizon: int, dt: int) -> None:
    """Validate prediction timing inputs, both expressed in milliseconds."""
    if prediction_horizon < 0:
        raise ValueError("prediction_horizon must be >= 0")
    if dt <= 0:
        raise ValueError("dt must be > 0")
    if prediction_horizon % dt != 0:
        raise ValueError(
            f"prediction_horizon ({prediction_horizon}) must be divisible by dt ({dt})"
        )

<<<<<<< HEAD

=======
>>>>>>> feature/bicycle-only
# Main role: convert scalar speed/acceleration into x and y components using the object's yaw.
def _vector_from_motion(value, yaw: float) -> Vector2D:
    """
    Convert scalar longitudinal motion or Vector2D motion into x/y components.

    Equations:
        vx = v * cos(yaw)
        vy = v * sin(yaw)
    """
    if isinstance(value, Vector2D):
        return value
    return Vector2D(x=float(value) * math.cos(yaw), y=float(value) * math.sin(yaw))

<<<<<<< HEAD

=======
>>>>>>> feature/bicycle-only
# Main role: convert vector velocity back into one scalar speed value.
def _speed_from_velocity(velocity) -> float:
    """
    Return scalar speed from either Vector2D velocity or scalar velocity.

    Equation:
        speed = sqrt(vx^2 + vy^2)
    """
    if isinstance(velocity, Vector2D):
        return math.hypot(velocity.x, velocity.y)
    return abs(float(velocity))

<<<<<<< HEAD

=======
>>>>>>> feature/bicycle-only
# Main role: predict one future position/velocity using constant acceleration and stop clamping.
def _predict_constant_acceleration_step(
    position: Vector2D,
    velocity: Vector2D,
    acceleration: Vector2D,
    time_s: float,
) -> tuple[Vector2D, Vector2D]:
    """
<<<<<<< HEAD
    Predict one step and clamp braking objects at the stop point.

    Constant acceleration equations:
        x(t)  = x0 + vx0 * t + 0.5 * ax * t^2
        y(t)  = y0 + vy0 * t + 0.5 * ay * t^2
        vx(t) = vx0 + ax * t
        vy(t) = vy0 + ay * t

    Stop-time equation for braking:
        t_stop = -dot(v, a) / dot(a, a)
=======
    Predict one step from t=0 and clamp braking objects exactly at their stop point.

    Constant acceleration kinematic equations:
        x(t)  = x0 + vx0*t + 0.5*ax*t^2
        y(t)  = y0 + vy0*t + 0.5*ay*t^2
        vx(t) = vx0 + ax*t
        vy(t) = vy0 + ay*t

    Stop-time derivation (dot product form):
        The object decelerates when dot(v, a) < 0.
        Setting |v(t)| = 0 along the motion axis gives:
            t_stop = -dot(v0, a) / dot(a, a)
        If t_stop falls within [0, time_s], the object has stopped.
        Position is evaluated at t_stop and velocity is clamped to zero.
        This prevents the model from reversing a braking vehicle.
>>>>>>> feature/bicycle-only
    """
    velocity_dot_accel = velocity.x * acceleration.x + velocity.y * acceleration.y
    acceleration_norm_sq = acceleration.x**2 + acceleration.y**2

<<<<<<< HEAD
    if velocity_dot_accel < 0.0 and acceleration_norm_sq > 1e-9:
=======
    if velocity_dot_accel < 0.0 and acceleration_norm_sq > _ACCEL_NORM_SQ_MIN:
>>>>>>> feature/bicycle-only
        stop_time_s = -velocity_dot_accel / acceleration_norm_sq
        if 0.0 <= stop_time_s <= time_s:
            stop_pos = Vector2D(
                x=position.x + velocity.x * stop_time_s + 0.5 * acceleration.x * stop_time_s**2,
                y=position.y + velocity.y * stop_time_s + 0.5 * acceleration.y * stop_time_s**2,
            )
            return stop_pos, Vector2D(x=0.0, y=0.0)

    new_pos = Vector2D(
        x=position.x + velocity.x * time_s + 0.5 * acceleration.x * time_s**2,
        y=position.y + velocity.y * time_s + 0.5 * acceleration.y * time_s**2,
    )
    new_velocity = Vector2D(
        x=velocity.x + acceleration.x * time_s,
        y=velocity.y + acceleration.y * time_s,
    )
    return new_pos, new_velocity

<<<<<<< HEAD

# Main role: predict one future position/velocity using constant speed and constant yaw rate.
def _predict_constant_turn_step(
    position: Vector2D,
    speed: float,
    yaw: float,
    yaw_rate: float,
    time_s: float,
) -> tuple[Vector2D, float, Vector2D]:
    """
    Predict one step using a constant turn-rate and velocity model.

    Constant turn equations:
        yaw(t) = yaw0 + yaw_rate * t
        x(t)   = x0 + v/yaw_rate * (sin(yaw(t)) - sin(yaw0))
        y(t)   = y0 - v/yaw_rate * (cos(yaw(t)) - cos(yaw0))

    Straight-line fallback when yaw_rate is close to zero:
        x(t) = x0 + v*cos(yaw0)*t
        y(t) = y0 + v*sin(yaw0)*t
    """
    if abs(yaw_rate) < _YAW_RATE_MIN:
=======
# Main role: predict one future position/yaw using the CTRV model.
def _predict_ctrv_step(
    position: Vector2D,
    yaw: float,
    speed: float,
    yaw_rate: float,
    time_s: float,
) -> tuple[Vector2D, float]:
    """
    Predict one step using the Constant Turn Rate and Velocity (CTRV) model.

    CTRV kinematic equations (exact closed-form integration of circular motion):
        When |yaw_rate| >= _YAW_RATE_MIN:
            x(t) = x0 + (v / ω) · ( sin(yaw0 + ω·t) - sin(yaw0) )
            y(t) = y0 + (v / ω) · ( cos(yaw0) - cos(yaw0 + ω·t) )
            yaw(t) = yaw0 + ω·t

        When |yaw_rate| < _YAW_RATE_MIN  (straight-line Taylor limit):
            x(t) = x0 + v·cos(yaw0)·t
            y(t) = y0 + v·sin(yaw0)·t
            yaw(t) = yaw0

    Parameters

    position  : current position in global frame [m]
    yaw       : current heading [rad], 0 = east, counter-clockwise positive
    speed     : longitudinal speed [m/s], always positive
    yaw_rate  : ω, rate of heading change [rad/s],
                positive = left turn, negative = right turn
    time_s    : prediction time from t=0 [s]

    Returns

    new_pos   : predicted position in global frame [m]
    new_yaw   : predicted heading [rad]

    Note: speed is held constant (no longitudinal acceleration). For a turning + braking vehicle, combine this with
                                                                 a longitudinal speed profile (future extension).
    """
    if abs(yaw_rate) >= _YAW_RATE_MIN:
        new_yaw = yaw + yaw_rate * time_s
        radius = speed / yaw_rate  # signed turning radius [m]
        new_pos = Vector2D(
            x=position.x + radius * (math.sin(new_yaw) - math.sin(yaw)),
            y=position.y + radius * (math.cos(yaw) - math.cos(new_yaw)),
        )
    else:
        # Straight-line fallback: Taylor expansion of CTRV as ω → 0
>>>>>>> feature/bicycle-only
        new_yaw = yaw
        new_pos = Vector2D(
            x=position.x + speed * math.cos(yaw) * time_s,
            y=position.y + speed * math.sin(yaw) * time_s,
        )
<<<<<<< HEAD
    else:
        new_yaw = yaw + yaw_rate * time_s
        new_pos = Vector2D(
            x=position.x + speed / yaw_rate * (math.sin(new_yaw) - math.sin(yaw)),
            y=position.y - speed / yaw_rate * (math.cos(new_yaw) - math.cos(yaw)),
        )

    new_velocity = Vector2D(
        x=speed * math.cos(new_yaw),
        y=speed * math.sin(new_yaw),
    )
    return new_pos, new_yaw, new_velocity

=======

    return new_pos, new_yaw
>>>>>>> feature/bicycle-only

# Main role: compare two headings correctly even when angles wrap around +pi/-pi.
def _angle_difference(a: float, b: float) -> float:
    """
    Return signed angle difference a - b wrapped to [-pi, pi].

    Equation:
<<<<<<< HEAD
        delta = (a - b + pi) mod (2*pi) - pi
    """
    return (a - b + math.pi) % (2 * math.pi) - math.pi


=======
        delta = (a - b + pi) mod (2·pi) - pi
    """
    return (a - b + math.pi) % (2 * math.pi) - math.pi

>>>>>>> feature/bicycle-only
# Main role: express an object's position in the ego vehicle's local coordinate frame.
def _object_relative_to_ego(obj: DynamicObjectStamped, ego_state: EgoStateStamped) -> Vector2D:
    """
    Transform object position from global frame into ego-local frame.

    Equations:
        dx = obj_x - ego_x
        dy = obj_y - ego_y
<<<<<<< HEAD
        x_ego =  dx*cos(yaw) + dy*sin(yaw)
        y_ego = -dx*sin(yaw) + dy*cos(yaw)
=======
        x_ego =  dx·cos(yaw) + dy·sin(yaw)    (forward axis)
        y_ego = −dx·sin(yaw) + dy·cos(yaw)    (left axis)
>>>>>>> feature/bicycle-only
    """
    dx = obj.state.pos.x - ego_state.state.pos.x
    dy = obj.state.pos.y - ego_state.state.pos.y
    yaw = ego_state.state.yaw
    return Vector2D(
        x=dx * math.cos(yaw) + dy * math.sin(yaw),
        y=-dx * math.sin(yaw) + dy * math.cos(yaw),
    )

<<<<<<< HEAD
=======
# Core prediction models
>>>>>>> feature/bicycle-only

# Main role: generate future states for every object over the prediction horizon.
def predict_motion_constant_acceleration(
    objects: List[DynamicObjectStamped],  # current dynamic objects from Environment.objects
    prediction_horizon: int,              # total prediction time [ms]
    dt: int,                              # prediction step size [ms]
<<<<<<< HEAD
    last_only: bool = False,              # return only final state per object if True
) -> List[DynamicObjectStamped]:
    """Predict object motion over the horizon using constant acceleration."""
=======
) -> List[DynamicObjectStamped]:
    """
    Predict object motion over the horizon using constant acceleration.

    Yaw is held constant (straight-line motion). For curved trajectories,
    use predict_motion_constant_turn_rate instead.
    """
>>>>>>> feature/bicycle-only
    _validate_prediction_inputs(prediction_horizon, dt)
    predicted_objects: List[DynamicObjectStamped] = []

    for obj in objects:
        cs = obj.state
        velocity = _vector_from_motion(cs.velocity, cs.yaw)          # initial velocity [m/s]
<<<<<<< HEAD
        acceleration = _vector_from_motion(cs.acceleration, cs.yaw)  # constant acceleration [m/s^2]
        predicted_states = []
=======
        acceleration = _vector_from_motion(cs.acceleration, cs.yaw)  # constant acceleration [m/s²]
>>>>>>> feature/bicycle-only

        for t in range(0, prediction_horizon + dt, dt):
            time_s = t / 1000.0
            new_pos, new_velocity = _predict_constant_acceleration_step(
                position=cs.pos,
                velocity=velocity,
                acceleration=acceleration,
                time_s=time_s,
            )
<<<<<<< HEAD

            predicted_states.append(
=======
            predicted_objects.append(
>>>>>>> feature/bicycle-only
                DynamicObjectStamped(
                    timestamp=obj.timestamp + t,
                    state=DynamicObject(
                        id=cs.id, obj_class=cs.obj_class,
                        pos=new_pos, yaw=cs.yaw,
                        velocity=new_velocity, acceleration=acceleration,
                        width=cs.width, length=cs.length,
                    ),
                )
            )

<<<<<<< HEAD
        if last_only:
            predicted_objects.append(predicted_states[-1])
        else:
            predicted_objects.extend(predicted_states)

    return predicted_objects

# Main role: generate future curved states for every object over the prediction horizon.
def predict_motion_constant_turn(
    objects: List[DynamicObjectStamped],       # current dynamic objects from Environment.objects
    prediction_horizon: int,                   # total prediction time [ms]
    dt: int,                                   # prediction step size [ms]
    yaw_rate: float = 0.0,                     # constant turn rate [rad/s]
    object_yaw_rates: Optional[Dict[int, float]] = None,  # optional per-object turn rates [rad/s]
    last_only: bool = False,                   # return only final state per object if True
) -> List[DynamicObjectStamped]:
    """Predict object motion over the horizon using constant speed and constant turn rate."""
    _validate_prediction_inputs(prediction_horizon, dt)
    predicted_objects: List[DynamicObjectStamped] = []
    object_yaw_rates = object_yaw_rates or {}
=======
    return predicted_objects

# Main role: generate future states using CTRV for objects that are turning.
def predict_constant_turn(
    objects: List[DynamicObjectStamped],  # current dynamic objects from Environment.objects
    prediction_horizon: int,              # total prediction time [ms]
    dt: int,                              # prediction step size [ms]
    yaw_rates: Optional[Dict[int, float]] = None,  # object_id -> yaw_rate [rad/s]; 0 if absent
) -> List[DynamicObjectStamped]:
    """
    Predict object motion over the horizon using the CTRV model.

    CTRV (Constant Turn Rate and Velocity) tracks circular arc motion exactly.
    It is more accurate than constant acceleration whenever an object is turning,
    such as the oncoming vehicle negotiating a curve before reaching ego.

    yaw_rates maps each object id to its measured yaw rate in rad/s.
    A positive value means a left turn, negative means a right turn.
    If an object id is not in yaw_rates, yaw_rate = 0 is assumed,
    which degenerates to straight-line motion (same as constant velocity).

    Speed is held constant (no longitudinal acceleration in this model).
    For a turning + braking vehicle, extend by applying a speed profile
    before calling this function.

    Parameters

    objects           : list of stamped dynamic objects at t=0
    prediction_horizon: total time to predict [ms]
    dt                : step size [ms]
    yaw_rates         : optional dict mapping object id to yaw_rate [rad/s]
    """
    _validate_prediction_inputs(prediction_horizon, dt)

    if yaw_rates is None:
        yaw_rates = {}

    predicted_objects: List[DynamicObjectStamped] = []
>>>>>>> feature/bicycle-only

    for obj in objects:
        cs = obj.state
        speed = _speed_from_velocity(cs.velocity)
<<<<<<< HEAD
        obj_yaw_rate = object_yaw_rates.get(cs.id, yaw_rate)
        predicted_states = []

        for t in range(0, prediction_horizon + dt, dt):
            time_s = t / 1000.0
            new_pos, new_yaw, new_velocity = _predict_constant_turn_step(
                position=cs.pos,
                speed=speed,
                yaw=cs.yaw,
                yaw_rate=obj_yaw_rate,
                time_s=time_s,
            )

            predicted_states.append(
=======
        yaw_rate = yaw_rates.get(cs.id, 0.0)

        for t in range(0, prediction_horizon + dt, dt):
            time_s = t / 1000.0
            new_pos, new_yaw = _predict_ctrv_step(
                position=cs.pos,
                yaw=cs.yaw,
                speed=speed,
                yaw_rate=yaw_rate,
                time_s=time_s,
            )
            # Reconstruct velocity vector from new heading and constant speed
            new_velocity = Vector2D(
                x=speed * math.cos(new_yaw),
                y=speed * math.sin(new_yaw),
            )
            predicted_objects.append(
>>>>>>> feature/bicycle-only
                DynamicObjectStamped(
                    timestamp=obj.timestamp + t,
                    state=DynamicObject(
                        id=cs.id, obj_class=cs.obj_class,
                        pos=new_pos, yaw=new_yaw,
<<<<<<< HEAD
                        velocity=new_velocity, acceleration=cs.acceleration,
=======
                        velocity=new_velocity,
                        acceleration=Vector2D(x=0.0, y=0.0),
>>>>>>> feature/bicycle-only
                        width=cs.width, length=cs.length,
                    ),
                )
            )

<<<<<<< HEAD
        if last_only:
            predicted_objects.append(predicted_states[-1])
        else:
            predicted_objects.extend(predicted_states)

    return predicted_objects

# Main role: reorganize flat predictions into object_id -> list of predicted states.
def group_predictions_by_object(
    predicted_objects: List[DynamicObjectStamped],  # flat list of predictions for all objects
=======
    return predicted_objects

# Environment predictions

# Main role: reorganize flat predictions into object_id -> list of predicted states.
def group_predictions_by_object(
    predicted_objects: List[DynamicObjectStamped],
>>>>>>> feature/bicycle-only
) -> Dict[int, List[DynamicObjectStamped]]:
    """Convert flat predictions into object_id -> predicted states."""
    grouped: Dict[int, List[DynamicObjectStamped]] = {}
    for obj in predicted_objects:
        grouped.setdefault(obj.state.id, []).append(obj)
    return grouped

# Main role: build a PredictedEnvironment from the current Environment.
def predict_environment(
<<<<<<< HEAD
    environment: Environment,                 # current scene with dynamic objects and lanes
    prediction_horizon: int,                  # total prediction time [ms]
    dt: int,                                  # prediction step size [ms]
    model: str = "constant_acceleration",     # prediction model name
    yaw_rate: float = 0.0,                    # used only by constant_turn [rad/s]
    object_yaw_rates: Optional[Dict[int, float]] = None,  # per-object yaw rates for constant_turn
) -> PredictedEnvironment:
    """Predict every object in the environment and return PredictedEnvironment."""
    if not environment.objects:
        raise ValueError("Empty object list — prediction would be vacuously safe.")
    if model == "constant_acceleration":
        raw = predict_motion_constant_acceleration(environment.objects, prediction_horizon, dt)
    elif model == "constant_turn":
        raw = predict_motion_constant_turn(
            environment.objects,
            prediction_horizon,
            dt,
            yaw_rate=yaw_rate,
            object_yaw_rates=object_yaw_rates,
        )
    else:
        raise ValueError(f"Unknown prediction model: {model}")
=======
    environment: Environment,
    prediction_horizon: int,
    dt: int,
    model: PredictionModel = "constant_acceleration",
    yaw_rates: Optional[Dict[int, float]] = None,
) -> PredictedEnvironment:
    """
    Predict every object in the environment and return a PredictedEnvironment.

    model = "constant_acceleration" : straight-line, handles braking correctly.
    model = "constant_turn_rate"    : CTRV, handles curved trajectories correctly.

    yaw_rates is only used when model = "constant_turn_rate".
    Pass a dict mapping object_id -> measured yaw_rate [rad/s].

    Raises ValueError if the environment contains no objects, because an empty
    prediction would cause is_overtake_gap_safe to return True vacuously.
    """
    if not environment.objects:
        raise ValueError("Empty object list — prediction would be vacuously safe.")

    if model == "constant_acceleration":
        raw = predict_motion_constant_acceleration(
            environment.objects, prediction_horizon, dt
        )
    elif model == "constant_turn_rate":
        raw = predict_constant_turn(
            environment.objects, prediction_horizon, dt, yaw_rates=yaw_rates
        )
    else:
        raise ValueError(f"Unknown prediction model: {model!r}")

>>>>>>> feature/bicycle-only
    return PredictedEnvironment(
        objects=group_predictions_by_object(raw),
        lanes=environment.lanes,
        dt=dt,
        horizon=prediction_horizon,
    )

# Main role: retrieve all predicted objects at one exact timestamp.
def objects_at_time(
    predicted_environment: PredictedEnvironment,
<<<<<<< HEAD
    timestamp: int,  # timestamp to query [ms]
) -> List[DynamicObjectStamped]:
    """Return predicted objects that exist exactly at timestamp."""
=======
    timestamp: int,
) -> List[DynamicObjectStamped]:
    """Return predicted objects that exist exactly at timestamp [ms]."""
>>>>>>> feature/bicycle-only
    result = []
    for preds in predicted_environment.objects.values():
        result.extend(obj for obj in preds if obj.timestamp == timestamp)
    return result

# Main role: compute straight-line distance between two 2D points.
def point_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two points."""
    return math.hypot(x2 - x1, y2 - y1)

<<<<<<< HEAD
# Main role: return the scalar speed of one predicted object.
def object_speed(obj: DynamicObjectStamped) -> float:
    """Return predicted object speed [m/s]."""
    return _speed_from_velocity(obj.state.velocity)

# Main role: find the closest vehicle ahead of ego in the same lane.
def find_lead_vehicle(
    environment: Environment,             # current scene
    ego_state: EgoStateStamped,           # ego pose used as reference
    lane_y_tolerance: float = 2.0,        # max lateral offset to count as same lane [m]
) -> Optional[DynamicObjectStamped]:
    """Find closest object ahead of ego in the same lane."""
    candidates = []
    for obj in environment.objects:
        rel = _object_relative_to_ego(obj, ego_state)
        if rel.x > 0.0 and abs(rel.y) <= lane_y_tolerance:
=======

# Main role: find the closest vehicle ahead of ego in the same lane.
def find_lead_vehicle(
    environment: Environment,
    ego_state: EgoStateStamped,
    lane_y_tolerance: float = 2.0,
    max_lookahead_m: float = 150.0,
) -> Optional[DynamicObjectStamped]:
    """
    Find the closest object ahead of ego in the same lane.

    lane_y_tolerance  : max lateral offset to count as same lane [m]
    max_lookahead_m   : ignore objects further than this distance ahead [m]
                        Prevents reacting to irrelevant distant traffic.
    """
    candidates = []
    for obj in environment.objects:
        rel = _object_relative_to_ego(obj, ego_state)
        if 0.0 < rel.x <= max_lookahead_m and abs(rel.y) <= lane_y_tolerance:
>>>>>>> feature/bicycle-only
            candidates.append((rel.x, obj))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None

# Main role: find the closest vehicle ahead of ego that is driving in the opposite direction.
def find_oncoming_vehicle(
<<<<<<< HEAD
    environment: Environment,                  # current scene
    ego_state: EgoStateStamped,                # ego pose used as reference
    opposite_lane_y_tolerance: float = 4.0,    # lateral search band for opposite lane [m]
) -> Optional[DynamicObjectStamped]:
    """Find closest object ahead of ego that is moving in the opposite direction."""
=======
    environment: Environment,
    ego_state: EgoStateStamped,
    opposite_lane_y_tolerance: float = 4.0,
    max_lookahead_m: float = 150.0,
) -> Optional[DynamicObjectStamped]:
    """
    Find the closest object ahead of ego moving in the opposite direction.

    opposite_lane_y_tolerance : lateral search band covering the oncoming lane [m]
    max_lookahead_m           : ignore objects further than this distance ahead [m]

    Heading check: |angle_difference(obj_yaw, ego_yaw)| > pi/2 means the object
    is travelling more than 90° away from ego — i.e. oncoming.
    """
>>>>>>> feature/bicycle-only
    candidates = []
    for obj in environment.objects:
        rel = _object_relative_to_ego(obj, ego_state)
        heading_delta = abs(_angle_difference(obj.state.yaw, ego_state.state.yaw))
<<<<<<< HEAD
        if rel.x > 0.0 and abs(rel.y) <= opposite_lane_y_tolerance and heading_delta > math.pi / 2:
            candidates.append((rel.x, obj))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None

# Main role: predict only the lead and oncoming vehicles needed for an overtake decision.
def predict_overtake_environment(
    environment: Environment,        # current scene
    ego_state: EgoStateStamped,      # ego pose used to select relevant vehicles
    prediction_horizon: int,         # total prediction time [ms]
    dt: int,                         # prediction step size [ms]
    model: str = "constant_acceleration",  # prediction model name
    yaw_rate: float = 0.0,           # used only by constant_turn [rad/s]
    object_yaw_rates: Optional[Dict[int, float]] = None,  # per-object yaw rates for constant_turn
) -> PredictedEnvironment:
    """Predict only the lead and oncoming vehicles relevant to overtaking."""
=======
        if (
            0.0 < rel.x <= max_lookahead_m
            and abs(rel.y) <= opposite_lane_y_tolerance
            and heading_delta > math.pi / 2
        ):
            candidates.append((rel.x, obj))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None

# Overtake specific prediction

# Main role: predict only the lead and oncoming vehicles needed for an overtake decision.
def predict_overtake_environment(
    environment: Environment,
    ego_state: EgoStateStamped,
    prediction_horizon: int,
    dt: int,
    model: PredictionModel = "constant_acceleration",
    yaw_rates: Optional[Dict[int, float]] = None,
) -> PredictedEnvironment:
    """
    Predict only the lead and oncoming vehicles relevant to overtaking.

    Use model="constant_turn_rate" when either vehicle is on a curved road
    and a measured yaw_rate is available from the sensor pipeline.
    Pass yaw_rates as {object_id: measured_yaw_rate_rad_per_s}.
    """
>>>>>>> feature/bicycle-only
    lead = find_lead_vehicle(environment, ego_state)
    oncoming = find_oncoming_vehicle(environment, ego_state)
    relevant = [obj for obj in (lead, oncoming) if obj is not None]
    if not relevant:
        raise ValueError("No lead or oncoming vehicle found.")
    return predict_environment(
        Environment(objects=relevant, lanes=environment.lanes),
        prediction_horizon=prediction_horizon,
        dt=dt,
        model=model,
<<<<<<< HEAD
        yaw_rate=yaw_rate,
        object_yaw_rates=object_yaw_rates,
    )

# Main role: decide if non-lead traffic has passed behind ego by the required safety distance.
def is_overtake_gap_safe(
    predicted_environment: PredictedEnvironment,  # predicted lead/oncoming vehicles
    ego_trajectory: List[EgoStateStamped],        # planned ego states over time
    lead_vehicle_id: int,                         # ignored in oncoming gap check
    overtake_start_ms: int,                       # start of planned overtake [ms]
    overtake_duration_ms: int,                    # duration of planned overtake [ms]
    safety_distance: float,                       # required distance after oncoming passes [m]
) -> bool:
    """
    Return True only if non-lead vehicles have already passed ego.

    For an overtake to be considered safe, the oncoming vehicle must be behind
    the ego vehicle by at least safety_distance at every checked timestamp in
    the overtake window. This prevents starting the overtake before the
    oncoming vehicle has passed.
=======
        yaw_rates=yaw_rates,
    )

# Main role: verify if the opposite vehicle has passed behind ego by the required safety distance.
def is_overtake_gap_safe(
    predicted_environment: PredictedEnvironment,
    ego_trajectory: List[EgoStateStamped],
    lead_vehicle_id: int,
    overtake_start_ms: int,
    overtake_duration_ms: int,
    safety_distance: float,
) -> bool:
    """
    Return True only if oncoming vehicles stay farther than safety_distance
    from ego throughout the entire overtake window.

    The lead vehicle is excluded by id. Proximity to it is expected
    during overtake and is managed by the lateral planner.

    ego_trajectory must contain a state for every timestamp in the window
    at the same dt as predicted_environment, otherwise a ValueError is raised.

    This is a prediction level gap check. The planner must still produce
    and collision-check the actual ego trajectory.
>>>>>>> feature/bicycle-only
    """
    end_ms = overtake_start_ms + overtake_duration_ms
    ego_by_ts: Dict[int, EgoStateStamped] = {es.timestamp: es for es in ego_trajectory}

    for ts in range(overtake_start_ms, end_ms + predicted_environment.dt, predicted_environment.dt):
        ego_state = ego_by_ts.get(ts)
        if ego_state is None:
<<<<<<< HEAD
            raise ValueError(f"ego_trajectory missing timestamp {ts} ms.")
        for obj in objects_at_time(predicted_environment, ts):
            if obj.state.id == lead_vehicle_id:
                continue
            rel = _object_relative_to_ego(obj, ego_state)
            # Oncoming vehicle must be behind ego by at least safety_distance.
            if rel.x > -safety_distance:
                return False

    return True

# ---------------------------------------------------------------------------
 # helpers
# ---------------------------------------------------------------------------

HEADER = "=" * 70
SUBHEADER = "-" * 70

# Main role: format a Vector2D for readable demo output.
def fmt_vec(v: Vector2D) -> str:
    return f"({v.x:+.2f}, {v.y:+.2f})"


# Main role: print predicted object states in a table for the demo.
def print_trajectory(label: str, predictions: List[DynamicObjectStamped]) -> None:
    print(f"\n  {label}")
    print(f"  {'Time(ms)':>10}  {'x (m)':>10}  {'y (m)':>10}  {'speed (m/s)':>12}  {'stopped':>8}")
    print(f"  {'-'*10}  {'-'*10}  {'-'*10}  {'-'*12}  {'-'*8}")
    for obj in predictions:
        speed = object_speed(obj)
        stopped = "YES" if speed < 0.01 else ""
        print(
            f"  {obj.timestamp:>10}  "
            f"{obj.state.pos.x:>10.2f}  "
            f"{obj.state.pos.y:>10.2f}  "
            f"{speed:>12.3f}  "
            f"{stopped:>8}"
        )


# Main role: create a simple straight-line ego trajectory for the demo safety check.
def build_ego_trajectory(
    start_x: float,
    start_y: float,
    speed_mps: float,
    start_ms: int,
    end_ms: int,
    dt: int,
) -> List[EgoStateStamped]:
    """
    Simple straight-line ego trajectory at constant speed.
    In a real planner this comes from the trajectory generator.
    """
    trajectory = []
    for t in range(start_ms, end_ms + dt, dt):
        elapsed_s = (t - start_ms) / 1000.0
        trajectory.append(
            EgoStateStamped(
                timestamp=t,
                state=EgoState(
                    pos=Vector2D(x=start_x + speed_mps * elapsed_s, y=start_y),
                    yaw=0.0,
                    velocity=speed_mps,
                    acceleration=0.0,
                ),
            )
        )
    return trajectory


# ---------------------------------------------------------------------------
# Demo scenarios
# ---------------------------------------------------------------------------

# Main role: run a complete example showing prediction, detection, and overtake safety checks.
def run_demo() -> None:
    print(HEADER)
    print(" PREDICTION OVERTAKE SCENARIO ")
    print(HEADER)

    DT_MS = 500           # prediction step: 500 ms
    HORIZON_MS = 4000     # predict 4 seconds ahead
    T0 = 0                # all objects start at t = 0 ms

    EGO_X = 0.0
    EGO_Y = 0.0
    EGO_SPEED = 10.0      # m/s (~36 km/h)
    EGO_YAW = 0.0         # heading east

    LEAD_X = 40.0         # 40 m ahead of ego
    LEAD_Y = 0.0          # same lane
    LEAD_SPEED = 8.0      # m/s, slower than ego
    LEAD_ACCEL = -4.0     # m/s² — hard braking, will stop

    # Oncoming: heading west → yaw = pi
    ONCOMING_Y = 3.5      # opposite lane (3.5 m lateral offset)
    ONCOMING_SPEED = 12.0 # m/s (~43 km/h) heading toward ego
    ONCOMING_YAW = math.pi

    ego_state_t0 = EgoStateStamped(
        timestamp=T0,
        state=EgoState(
            pos=Vector2D(x=EGO_X, y=EGO_Y),
            yaw=EGO_YAW,
            velocity=EGO_SPEED,
            acceleration=0.0,
        ),
    )

    lead_vehicle = DynamicObjectStamped(
        timestamp=T0,
        state=DynamicObject(
            id=1,
            obj_class="car",
            pos=Vector2D(x=LEAD_X, y=LEAD_Y),
            yaw=EGO_YAW,
            velocity=Vector2D(x=LEAD_SPEED, y=0.0),
            acceleration=Vector2D(x=LEAD_ACCEL, y=0.0),
            width=2.0,
            length=4.5,
        ),
    )

    # ------------------------------------------------------------------
    # Section 1: Lead vehicle trajectory (braking to a stop)
    # ------------------------------------------------------------------
    print("\n[1] LEAD VEHICLE PREDICTION (braking hard at -4 m/s²)")
    print(SUBHEADER)
    print(f"  Initial pos : x={LEAD_X:.1f} m, y={LEAD_Y:.1f} m")
    print(f"  Initial speed: {LEAD_SPEED:.1f} m/s")
    print(f"  Acceleration: {LEAD_ACCEL:.1f} m/s²")

    lead_preds = predict_motion_constant_acceleration(
        objects=[lead_vehicle],
        prediction_horizon=HORIZON_MS,
        dt=DT_MS,
    )
    print_trajectory("Lead vehicle states:", lead_preds)

    # Verify velocity clamping — speed should not go below zero
    min_speed = min(object_speed(obj) for obj in lead_preds)
    print(f"\n  ✓ Minimum predicted speed = {min_speed:.4f} m/s  (should be 0, never negative)")

    # ------------------------------------------------------------------
    # Section 2: Oncoming vehicle trajectory (constant approach)
    # ------------------------------------------------------------------
    print(f"\n[2] ONCOMING VEHICLE PREDICTION (constant velocity, heading west)")
    print(SUBHEADER)

    def make_oncoming(x_start: float, label: str) -> DynamicObjectStamped:
        return DynamicObjectStamped(
            timestamp=T0,
            state=DynamicObject(
                id=2,
                obj_class="car",
                pos=Vector2D(x=x_start, y=ONCOMING_Y),
                yaw=ONCOMING_YAW,
                velocity=Vector2D(x=-ONCOMING_SPEED, y=0.0),  # negative x = westbound
                acceleration=Vector2D(x=0.0, y=0.0),
                width=2.0,
                length=4.5,
            ),
        )

    oncoming_far = make_oncoming(x_start=120.0, label="FAR")
    oncoming_near = make_oncoming(x_start=60.0, label="NEAR")

    for label, oncoming_obj in [("FAR (120 m away)", oncoming_far), ("NEAR (60 m away)", oncoming_near)]:
        preds = predict_motion_constant_acceleration(
            objects=[oncoming_obj],
            prediction_horizon=HORIZON_MS,
            dt=DT_MS,
        )
        print_trajectory(f"Oncoming vehicle — {label}:", preds)

    # ------------------------------------------------------------------
    # Section 3: Scene detection (find_lead_vehicle / find_oncoming_vehicle)
    # ------------------------------------------------------------------
    print(f"\n[3] SCENE DETECTION")
    print(SUBHEADER)

    env_far = Environment(objects=[lead_vehicle, oncoming_far], lanes=[])
    env_near = Environment(objects=[lead_vehicle, oncoming_near], lanes=[])

    detected_lead = find_lead_vehicle(env_far, ego_state_t0)
    detected_oncoming = find_oncoming_vehicle(env_far, ego_state_t0)

    print(f"  Ego at x={EGO_X:.1f} m, heading east (yaw=0)")
    print(
        f"  Lead vehicle detected    : "
        f"{'YES — id=' + str(detected_lead.state.id) + ' at x=' + str(detected_lead.state.pos.x) if detected_lead else 'NO'}"
    )
    print(
        f"  Oncoming vehicle detected: "
        f"{'YES — id=' + str(detected_oncoming.state.id) + ' at x=' + str(detected_oncoming.state.pos.x) if detected_oncoming else 'NO'}"
    )

    # ------------------------------------------------------------------
    # Section 4: Full predict_overtake_environment output
    # ------------------------------------------------------------------
    print(f"\n[4] PREDICTED OVERTAKE ENVIRONMENT (far oncoming)")
    print(SUBHEADER)

    pred_env_far = predict_overtake_environment(
        environment=env_far,
        ego_state=ego_state_t0,
        prediction_horizon=HORIZON_MS,
        dt=DT_MS,
    )

    for obj_id, preds in pred_env_far.objects.items():
        role = "lead" if obj_id == 1 else "oncoming"
        print_trajectory(f"Object id={obj_id} ({role}):", preds)

    # ------------------------------------------------------------------
    # Section 5: objects_at_time spot check
    # ------------------------------------------------------------------
    print(f"\n[5] OBJECTS AT t=2000 ms (mid-overtake snapshot)")
    print(SUBHEADER)

    snapshot = objects_at_time(pred_env_far, timestamp=2000)
    for obj in snapshot:
        role = "lead" if obj.state.id == 1 else "oncoming"
        print(
            f"  id={obj.state.id} ({role:8s})  "
            f"x={obj.state.pos.x:7.2f} m  "
            f"y={obj.state.pos.y:6.2f} m  "
            f"speed={object_speed(obj):.2f} m/s"
        )

    # ------------------------------------------------------------------
    # Section 6: Safety check — Scenario A (FAR oncoming → SAFE)
    # ------------------------------------------------------------------
    print(f"\n[6] OVERTAKE SAFETY CHECK")
    print(SUBHEADER)

    # Simple straight-line ego trajectory at constant speed during overtake
    OVERTAKE_START_MS = 0
    OVERTAKE_DURATION_MS = 4000
    SAFETY_DISTANCE_M = 30.0

    ego_traj = build_ego_trajectory(
        start_x=EGO_X,
        start_y=EGO_Y,
        speed_mps=EGO_SPEED,
        start_ms=OVERTAKE_START_MS,
        end_ms=OVERTAKE_START_MS + OVERTAKE_DURATION_MS,
        dt=DT_MS,
    )

    print(f"  Safety distance threshold : {SAFETY_DISTANCE_M:.1f} m")
    print(f"  Overtake window           : {OVERTAKE_START_MS}–{OVERTAKE_START_MS + OVERTAKE_DURATION_MS} ms")
    print(f"  Ego speed during overtake : {EGO_SPEED:.1f} m/s (constant, straight-line)")

    # Scenario A — far oncoming
    pred_env_far_check = predict_overtake_environment(
        environment=env_far,
        ego_state=ego_state_t0,
        prediction_horizon=HORIZON_MS,
        dt=DT_MS,
    )
    safe_a = is_overtake_gap_safe(
        predicted_environment=pred_env_far_check,
        ego_trajectory=ego_traj,
        lead_vehicle_id=1,
        overtake_start_ms=OVERTAKE_START_MS,
        overtake_duration_ms=OVERTAKE_DURATION_MS,
        safety_distance=SAFETY_DISTANCE_M,
    )
    print(f"\n  Scenario A — Oncoming starts at 120 m: {'✓ SAFE to overtake' if safe_a else '✗ NOT safe to overtake'}")

    # Scenario B — near oncoming
    pred_env_near_check = predict_overtake_environment(
        environment=env_near,
        ego_state=ego_state_t0,
        prediction_horizon=HORIZON_MS,
        dt=DT_MS,
    )
    safe_b = is_overtake_gap_safe(
        predicted_environment=pred_env_near_check,
        ego_trajectory=ego_traj,
        lead_vehicle_id=1,
        overtake_start_ms=OVERTAKE_START_MS,
        overtake_duration_ms=OVERTAKE_DURATION_MS,
        safety_distance=SAFETY_DISTANCE_M,
    )
    print(f"  Scenario B — Oncoming starts at  60 m: {'✓ SAFE to overtake' if safe_b else '✗ NOT safe to overtake'}")

    # Show at which timestamp Scenario B fails
    if not safe_b:
        print(f"\n  Breakdown of closing distance (Scenario B):")
        print(f"  {'Time(ms)':>10}  {'ego_x':>8}  {'oncoming_x':>12}  {'distance':>10}  {'< {:.0f}m?'.format(SAFETY_DISTANCE_M):>10}")
        print(f"  {'-'*10}  {'-'*8}  {'-'*12}  {'-'*10}  {'-'*10}")
        ego_by_ts = {es.timestamp: es for es in ego_traj}
        for ts in range(OVERTAKE_START_MS, OVERTAKE_START_MS + OVERTAKE_DURATION_MS + DT_MS, DT_MS):
            es = ego_by_ts.get(ts)
            if es is None:
                continue
            for obj in objects_at_time(pred_env_near_check, ts):
                if obj.state.id == 1:
                    continue
                dist = point_distance(
                    es.state.pos.x, es.state.pos.y,
                    obj.state.pos.x, obj.state.pos.y,
                )
                flag = "✗ UNSAFE" if dist < SAFETY_DISTANCE_M else ""
                print(
                    f"  {ts:>10}  {es.state.pos.x:>8.1f}  "
                    f"{obj.state.pos.x:>12.1f}  {dist:>10.1f}  {flag:>10}"
                )

    # ------------------------------------------------------------------
    # Printed comments
    # ------------------------------------------------------------------
    print(f"\n{HEADER}")
    print("  SUMMARY")
    print(HEADER)
    print(f"  Lead vehicle (id=1):  braking at {LEAD_ACCEL} m/s², velocity clamped to 0 ✓")
    print(f"  Oncoming far  (120 m): overtake {'SAFE ✓' if safe_a else 'NOT SAFE ✗'}")
    print(f"  Oncoming near  (60 m): overtake {'SAFE ✓' if safe_b else 'NOT SAFE ✗'}")
    print(f"\n  If both lines above match expectations, predictivity.py is working correctly.")
    print(HEADER)


if __name__ == "__main__":
    run_demo()
=======
            raise ValueError(
                f"ego_trajectory missing timestamp {ts} ms. "
                f"Ensure it covers the full overtake window at dt={predicted_environment.dt} ms."
            )
        for obj in objects_at_time(predicted_environment, ts):
            if obj.state.id == lead_vehicle_id:
                continue
            dist = point_distance(
                ego_state.state.pos.x, ego_state.state.pos.y,
                obj.state.pos.x, obj.state.pos.y,
            )
            if dist < safety_distance:
                return False
    return True
>>>>>>> feature/bicycle-only
