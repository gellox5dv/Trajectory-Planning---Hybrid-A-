import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, List, Literal, Optional
import numpy as np

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from models.models import (
    DynamicObject,
    DynamicObjectStamped,
     EgoState,
    EgoStateStamped,
    Environment,
    PredictedEnvironment,
    Vector2D,
)
from utils.helper import get_magnitude, get_vector, global_to_ego_axis, get_signed_magnitude

PredictionModel = Literal["constant_acceleration", "constant_turn_rate"]

# Guard for near-zero yaw rate in CTRV: below this, use straight-line fallback.
_YAW_RATE_MIN: float = 1e-4  # [rad/s]

# Guard for near-zero acceleration norm in braking clamp.
_ACCEL_NORM_SQ_MIN: float = 1e-9

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

# Main role: predict one future position/velocity using constant acceleration and stop clamping.
def _predict_constant_acceleration_step(
    position: Vector2D,
    velocity: Vector2D,
    acceleration: Vector2D,
    time_s: float,
) -> tuple[Vector2D, Vector2D]:
    """
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
    """
    velocity_dot_accel = velocity.x * acceleration.x + velocity.y * acceleration.y
    acceleration_norm_sq = acceleration.x**2 + acceleration.y**2

    if velocity_dot_accel < 0.0 and acceleration_norm_sq > _ACCEL_NORM_SQ_MIN:
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
    #circle motion equations, handles the turning
    if abs(yaw_rate) >= _YAW_RATE_MIN:
        new_yaw = yaw + yaw_rate * time_s
        radius = speed / yaw_rate  # signed turning radius [m]
        new_pos = Vector2D(
            x=position.x + radius * (math.sin(new_yaw) - math.sin(yaw)),
            y=position.y + radius * (math.cos(yaw) - math.cos(new_yaw)),
        )
    else:
        # Straight-line fallback: Taylor expansion of CTRV as ω → 0
        new_yaw = yaw
        new_pos = Vector2D(
            x=position.x + speed * math.cos(yaw) * time_s,
            y=position.y + speed * math.sin(yaw) * time_s,
        )

    return new_pos, new_yaw

# Core prediction models

# Main role: generate future states for every object over the prediction horizon.
def predict_motion_constant_acceleration(
    objects: List[DynamicObjectStamped],  # current dynamic objects from Environment.objects
    prediction_horizon: int,              # total prediction time [ms]
    dt: int,                              # prediction step size [ms]
    accelerations: Optional[Dict[int, Vector2D | float]] = None,  # object_id -> acceleration override
) -> List[DynamicObjectStamped]:
    """
    Predict object motion over the horizon using constant acceleration.

    If accelerations is provided, each object uses the acceleration from
    accelerations[object_id] instead of obj.state.acceleration.

    Yaw is held constant (straight-line motion). For curved trajectories,
    use predict_motion_constant_turn_rate instead.
    """
    _validate_prediction_inputs(prediction_horizon, dt)
    predicted_objects: List[DynamicObjectStamped] = []

    for obj in objects:
        cs = obj.state
        velocity = (
            cs.velocity
            if isinstance(cs.velocity, Vector2D)
            else get_vector(float(cs.velocity), cs.yaw)
        )
        acceleration_value = cs.acceleration
        if accelerations is not None:
            if cs.id not in accelerations:
                raise ValueError(f"Missing acceleration for object id {cs.id}")
            acceleration_value = accelerations[cs.id]

        acceleration = (
            acceleration_value
            if isinstance(acceleration_value, Vector2D)
            else get_vector(float(acceleration_value), cs.yaw)
        )
        # create a predicted object at every timestamp:
        for t in range(0, prediction_horizon + dt, dt):
            time_s = t / 1000.0
            new_pos, new_velocity = _predict_constant_acceleration_step(
                position=cs.pos,
                velocity=velocity,
                acceleration=acceleration,
                time_s=time_s,
            )
            predicted_objects.append(
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

    for obj in objects:
        cs = obj.state
        speed = (
            get_signed_magnitude(cs.velocity, cs.yaw)
            if isinstance(cs.velocity, Vector2D)
            else abs(float(cs.velocity))
        )
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
            # Reconstruct velocity vector from new heading and constant speed.
            new_velocity = get_vector(speed, new_yaw)
            predicted_objects.append(
                DynamicObjectStamped(
                    timestamp=obj.timestamp + t,
                    state=DynamicObject(
                        id=cs.id, obj_class=cs.obj_class,
                        pos=new_pos, yaw=new_yaw,
                        velocity=new_velocity,
                        acceleration=get_vector(0.0, new_yaw),
                        width=cs.width, length=cs.length,
                    ),
                )
            )

    return predicted_objects

# Environment predictions

# Main role: reorganize flat predictions into object_id -> list of predicted states.
def group_predictions_by_object(
    predicted_objects: List[DynamicObjectStamped],
) -> Dict[int, List[DynamicObjectStamped]]:
    """Convert flat predictions into object_id -> predicted states."""
    grouped: Dict[int, List[DynamicObjectStamped]] = {}
    for obj in predicted_objects:
        grouped.setdefault(obj.state.id, []).append(obj)
    return grouped

# Main role: build a PredictedEnvironment from the current Environment.
def predict_environment(
    environment: Environment,
    prediction_horizon: int,
    dt: int,
    model: PredictionModel = "constant_acceleration",
    accelerations: Optional[Dict[int, Vector2D | float]] = None,
    yaw_rates: Optional[Dict[int, float]] = None,
) -> PredictedEnvironment:
    """
    Predict every object in the environment and return a PredictedEnvironment.

    model = "constant_acceleration" : straight-line, handles braking correctly.
    model = "constant_turn_rate"    : CTRV, handles curved trajectories correctly.

    accelerations is only used when model = "constant_acceleration".
    Pass a dict mapping object_id -> acceleration, either Vector2D or scalar
    longitudinal acceleration [m/s²].

    yaw_rates is only used when model = "constant_turn_rate".
    Pass a dict mapping object_id -> measured yaw_rate [rad/s].

    Raises ValueError if the environment contains no objects, because an empty
    prediction would cause is_overtake_gap_safe to return True vacuously.
    """
    if not environment.objects:
        raise ValueError("Empty object list — prediction would be vacuously safe.")

    if model == "constant_acceleration":
        raw = predict_motion_constant_acceleration(
            environment.objects,
            prediction_horizon,
            dt,
            accelerations=accelerations,
        )
    elif model == "constant_turn_rate":
        raw = predict_constant_turn(
            environment.objects, prediction_horizon, dt, yaw_rates=yaw_rates
        )
    else:
        raise ValueError(f"Unknown prediction model: {model!r}")

    return PredictedEnvironment(
        objects=group_predictions_by_object(raw),
        lanes=environment.lanes,
        dt=dt,
        horizon=prediction_horizon,
    )

# Main role: retrieve all predicted objects at one exact timestamp.
def objects_at_time(
    predicted_environment: PredictedEnvironment,
    timestamp: int,
) -> List[DynamicObjectStamped]:
    """Return predicted objects that exist exactly at timestamp [ms]."""
    result = []
    for preds in predicted_environment.objects.values():
        result.extend(obj for obj in preds if obj.timestamp == timestamp)
    return result

# Main role: find the closest vehicle ahead of ego in the same lane.
def find_lead_vehicle(
    environment: Environment,
    ego_state: EgoStateStamped,
    lane_y_tolerance: float = 2.0, #how far sideways an object can be and still count as same lane.
    max_lookahead_m: float = 150.0, #ignore objects too far away.
) -> Optional[DynamicObjectStamped]:
    """
    Find the closest object ahead of ego in the same lane.

    lane_y_tolerance  : max lateral offset to count as same lane [m]
    max_lookahead_m   : ignore objects further than this distance ahead [m]
                        Prevents reacting to irrelevant distant traffic.
    """
    candidates = []
    for obj in environment.objects:
        rel_x, rel_y, _ = global_to_ego_axis(
            obj.state.pos.x,
            obj.state.pos.y,
            ego_state.state.pos.x,
            ego_state.state.pos.y,
            ego_state.state.yaw,
        )
        if 0.0 < rel_x <= max_lookahead_m and abs(rel_y) <= lane_y_tolerance:
            candidates.append((rel_x, obj))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None

# Main role: find the closest vehicle ahead of ego that is driving in the opposite direction.
def find_oncoming_vehicle(
    environment: Environment,
    ego_state: EgoStateStamped,
    opposite_lane_y_tolerance: float = 4.0,
    max_lookahead_m: float = 150.0,
) -> Optional[DynamicObjectStamped]:
    """
    Find the closest object ahead of ego moving in the opposite direction.

    opposite_lane_y_tolerance : lateral search band covering the oncoming lane [m]
    max_lookahead_m           : ignore objects further than this distance ahead [m]

    Heading check: |relative_yaw| > pi/2 means the object
    is travelling more than 90° away from ego — i.e. oncoming.
    """
    candidates = []
    for obj in environment.objects:
        rel_x, rel_y, rel_yaw = global_to_ego_axis(
            obj.state.pos.x,
            obj.state.pos.y,
            ego_state.state.pos.x,
            ego_state.state.pos.y,
            ego_state.state.yaw,
            poi_yaw=obj.state.yaw,
        )
        heading_delta = abs(rel_yaw) if rel_yaw is not None else 0.0
        if (
            0.0 < rel_x <= max_lookahead_m
            and abs(rel_y) <= opposite_lane_y_tolerance
            and heading_delta > math.pi / 2
        ):
            candidates.append((rel_x, obj))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None

# Overtake specific prediction

# Main role: predict only the lead and oncoming vehicles needed for an overtake decision.
def predict_overtake_environment(
    environment: Environment,
    ego_state: EgoStateStamped,
    prediction_horizon: int,
    dt: int,
    model: PredictionModel = "constant_acceleration",
    accelerations: Optional[Dict[int, Vector2D | float]] = None,
    yaw_rates: Optional[Dict[int, float]] = None,
) -> PredictedEnvironment:
    """
    Predict only the lead and oncoming vehicles relevant to overtaking.

    Use model="constant_turn_rate" when either vehicle is on a curved road
    and a measured yaw_rate is available from the sensor pipeline.
    Pass yaw_rates as {object_id: measured_yaw_rate_rad_per_s}.
    """
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
        accelerations=accelerations,
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
    """
    end_ms = overtake_start_ms + overtake_duration_ms
    ego_by_ts: Dict[int, EgoStateStamped] = {es.timestamp: es for es in ego_trajectory}

    for ts in range(overtake_start_ms, end_ms + predicted_environment.dt, predicted_environment.dt):
        ego_state = ego_by_ts.get(ts)
        if ego_state is None:
            raise ValueError(
                f"ego_trajectory missing timestamp {ts} ms. "
                f"Ensure it covers the full overtake window at dt={predicted_environment.dt} ms."
            )
        for obj in objects_at_time(predicted_environment, ts):
            if obj.state.id == lead_vehicle_id:
                continue
            dist = get_signed_magnitude(
                Vector2D(
                    x=obj.state.pos.x - ego_state.state.pos.x,
                    y=obj.state.pos.y - ego_state.state.pos.y,
                ),
                obj.state.yaw
            )
            if dist < safety_distance:
                return False
    return True


# Event-based visualization for the overtake decision.
@dataclass(frozen=True)
class OvertakeReplayBehavior:
    """Behavior policy used only by the prediction verification plot."""

    lead_cruise_time_ms: int
    lead_brake_acceleration: float
    ego_follow_gap_m: float
    ego_max_acceleration: float
    ego_max_deceleration: float
    safety_distance_m: float
    oncoming_clear_after_lead_stop_ms: int
    visualization_dt_ms: int
    plot_horizon_ms: int
    plot_lead_stop_x_m: float


def _cfg_lookup(cfg, dotted_path: str, default):
    """Read an optional nested config value without requiring it to exist."""
    current = cfg
    for key in dotted_path.split("."):
        if current is None or key not in current:
            return default
        current = current[key]
    return current


def _is_auto_config_value(value) -> bool:
    """Return True for config values that request stack-derived behavior."""
    return isinstance(value, str) and value.strip().lower() in {"auto", "dynamic"}


def _flatten_predictions(
    predicted_env: PredictedEnvironment,
    object_id: int,
) -> tuple["np.ndarray", "np.ndarray", "np.ndarray", "np.ndarray"]:
    """Return timestamps, x positions, y positions, and scalar speeds."""
    predictions = sorted(
        predicted_env.objects[object_id],
        key=lambda obj: obj.timestamp,
    )
    timestamps = np.array([obj.timestamp for obj in predictions])
    xs = np.array([obj.state.pos.x for obj in predictions])
    ys = np.array([obj.state.pos.y for obj in predictions])
    speeds = np.array([get_magnitude(obj.state.velocity) for obj in predictions])
    return timestamps, xs, ys, speeds


def _to_ego_frame(
    x: float,
    y: float,
    ego_state: EgoStateStamped,
) -> tuple[float, float]:
    """Convert one global point into the decision ego frame."""
    local_x, local_y, _ = global_to_ego_axis(
        x,
        y,
        ego_state.state.pos.x,
        ego_state.state.pos.y,
        ego_state.state.yaw,
    )
    return local_x, local_y


def _arrays_to_ego_frame(
    xs: "np.ndarray",
    ys: "np.ndarray",
    ego_state: EgoStateStamped,
) -> tuple["np.ndarray", "np.ndarray"]:
    """Convert arrays of global points into the decision ego frame."""
    local_points = [_to_ego_frame(float(x), float(y), ego_state) for x, y in zip(xs, ys)]
    if not local_points:
        return np.array([]), np.array([])
    local_xs, local_ys = zip(*local_points)
    return np.array(local_xs), np.array(local_ys)


def _load_prediction_visualization_cfg(config_overrides: Optional[List[str]] = None):
    """Load the project config exactly like the simulation stack does."""
    from hydra import compose, initialize_config_dir
    from hydra.core.global_hydra import GlobalHydra

    config_dir = Path(__file__).resolve().parents[1] / "configs"
    if GlobalHydra.instance().is_initialized():
        GlobalHydra.instance().clear()

    with initialize_config_dir(version_base=None, config_dir=str(config_dir)):
        return compose(
            config_name="config",
            overrides=list(config_overrides or []),
        )


def _ceil_ms_to_step(time_ms: float, dt: int) -> int:
    """Round a millisecond timestamp up to the next prediction grid step."""
    return int(math.ceil(time_ms / dt) * dt)


def _lead_braking_acceleration(cfg) -> float:
    """Return the configured braking acceleration as a negative scalar."""
    deceleration = float(
        _cfg_lookup(
            cfg,
            "scenario.behavior.lead_brake_acceleration",
            cfg.vehicle.max_deceleration,
        )
    )
    return deceleration if deceleration < 0.0 else -deceleration


def _speed_along_heading(obj: DynamicObjectStamped) -> float:
    """Return object speed signed along its own yaw."""
    return get_signed_magnitude(obj.state.velocity, obj.state.yaw)


def _ego_speed_along_heading(ego_state: EgoStateStamped) -> float:
    """Return ego speed signed along its own yaw."""
    return get_signed_magnitude(ego_state.state.velocity, ego_state.state.yaw)


def _longitudinal_acceleration(obj: DynamicObjectStamped) -> float:
    """Project an object's acceleration onto its heading direction."""
    return get_signed_magnitude(obj.state.acceleration, obj.state.yaw)


def _estimate_stop_time_ms(
    obj: DynamicObjectStamped,
    braking_acceleration: float,
) -> Optional[int]:
    """Estimate when an object will stop under constant longitudinal braking."""
    speed = get_signed_magnitude(obj.state.velocity, obj.state.yaw)
    if speed <= 0.0 or braking_acceleration >= 0.0:
        return None
    return int(math.ceil(1000.0 * speed / abs(braking_acceleration)))


def _estimate_oncoming_clear_time_ms(
    ego_state: EgoStateStamped,
    oncoming: DynamicObjectStamped,
    safety_distance: float,
) -> Optional[int]:
    """Estimate when the oncoming car clears ego by safety_distance."""
    heading_x = math.cos(oncoming.state.yaw)
    heading_y = math.sin(oncoming.state.yaw)
    rel_pos = Vector2D(
        x=oncoming.state.pos.x - ego_state.state.pos.x,
        y=oncoming.state.pos.y - ego_state.state.pos.y,
    )
    signed_clearance_now = get_signed_magnitude(rel_pos, oncoming.state.yaw)
    rel_velocity_x = oncoming.state.velocity.x - ego_state.state.velocity.x
    rel_velocity_y = oncoming.state.velocity.y - ego_state.state.velocity.y
    clearance_rate = rel_velocity_x * heading_x + rel_velocity_y * heading_y

    if signed_clearance_now >= safety_distance:
        return 0
    if clearance_rate <= 1e-6:
        return None

    return int(
        math.ceil(
            1000.0 * (safety_distance - signed_clearance_now) / clearance_rate
        )
    )


def _object_with_state(
    obj: DynamicObjectStamped,
    timestamp: int,
    pos: Vector2D,
    velocity: Vector2D,
    acceleration: Vector2D,
) -> DynamicObjectStamped:
    """Copy object metadata while replacing motion state."""
    return DynamicObjectStamped(
        timestamp=timestamp,
        state=DynamicObject(
            id=obj.state.id,
            obj_class=obj.state.obj_class,
            pos=pos,
            yaw=obj.state.yaw,
            velocity=velocity,
            acceleration=acceleration,
            width=obj.state.width,
            length=obj.state.length,
        ),
    )


def _lead_policy_state(
    lead: DynamicObjectStamped,
    elapsed_ms: int,
    behavior: OvertakeReplayBehavior,
) -> DynamicObjectStamped:
    """Lead vehicle policy: cruise, then brake to a complete stop."""
    lead_speed = max(0.0, _speed_along_heading(lead))
    yaw = lead.state.yaw
    cruise_time_s = min(elapsed_ms, behavior.lead_cruise_time_ms) / 1000.0
    cruise_pos = Vector2D(
        x=lead.state.pos.x + lead_speed * math.cos(yaw) * cruise_time_s,
        y=lead.state.pos.y + lead_speed * math.sin(yaw) * cruise_time_s,
    )

    if elapsed_ms <= behavior.lead_cruise_time_ms:
        return _object_with_state(
            obj=lead,
            timestamp=lead.timestamp + elapsed_ms,
            pos=cruise_pos,
            velocity=get_vector(lead_speed, yaw),
            acceleration=Vector2D(x=0.0, y=0.0),
        )

    brake_time_s = (elapsed_ms - behavior.lead_cruise_time_ms) / 1000.0
    brake_acceleration = get_vector(behavior.lead_brake_acceleration, yaw)
    pos, velocity = _predict_constant_acceleration_step(
        position=cruise_pos,
        velocity=get_vector(lead_speed, yaw),
        acceleration=brake_acceleration,
        time_s=brake_time_s,
    )
    if get_magnitude(velocity) <= 1e-6:
        brake_acceleration = Vector2D(x=0.0, y=0.0)
    return _object_with_state(
        obj=lead,
        timestamp=lead.timestamp + elapsed_ms,
        pos=pos,
        velocity=velocity,
        acceleration=brake_acceleration,
    )


def _constant_acceleration_object_state(
    obj: DynamicObjectStamped,
    elapsed_ms: int,
) -> DynamicObjectStamped:
    """Replay a non-lead object from its current stack state."""
    pos, velocity = _predict_constant_acceleration_step(
        position=obj.state.pos,
        velocity=obj.state.velocity,
        acceleration=obj.state.acceleration,
        time_s=elapsed_ms / 1000.0,
    )
    return _object_with_state(
        obj=obj,
        timestamp=obj.timestamp + elapsed_ms,
        pos=pos,
        velocity=velocity,
        acceleration=obj.state.acceleration,
    )


def _ego_follow_acceleration(
    ego_state: EgoStateStamped,
    lead_state: DynamicObjectStamped,
    behavior: OvertakeReplayBehavior,
) -> float:
    """Ego policy: follow the lead vehicle and brake before the desired gap."""
    rel_x, _, _ = global_to_ego_axis(
        lead_state.state.pos.x,
        lead_state.state.pos.y,
        ego_state.state.pos.x,
        ego_state.state.pos.y,
        ego_state.state.yaw,
    )
    ego_speed = max(0.0, _ego_speed_along_heading(ego_state))
    lead_speed = max(0.0, _speed_along_heading(lead_state))
    lead_acceleration = _longitudinal_acceleration(lead_state)
    gap_error = rel_x - behavior.ego_follow_gap_m

    target_speed = max(0.0, lead_speed + 0.06 * gap_error)
    requested_acceleration = (target_speed - ego_speed) / 1.5

    if gap_error <= 0.0:
        requested_acceleration = min(requested_acceleration, -behavior.ego_max_deceleration)
    elif ego_speed > lead_speed and (lead_acceleration < -1e-6 or gap_error <= 10.0):
        stop_acceleration = (lead_speed**2 - ego_speed**2) / (2.0 * gap_error)
        requested_acceleration = min(requested_acceleration, stop_acceleration)

    return max(
        -behavior.ego_max_deceleration,
        min(behavior.ego_max_acceleration, requested_acceleration),
    )


def _step_ego_follow_policy(
    ego_state: EgoStateStamped,
    lead_state: DynamicObjectStamped,
    behavior: OvertakeReplayBehavior,
    dt: int,
) -> EgoStateStamped:
    """Advance ego one step while following the lead in the same lane."""
    acceleration_scalar = _ego_follow_acceleration(ego_state, lead_state, behavior)
    acceleration = get_vector(acceleration_scalar, ego_state.state.yaw)
    pos, velocity = _predict_constant_acceleration_step(
        position=ego_state.state.pos,
        velocity=ego_state.state.velocity,
        acceleration=acceleration,
        time_s=dt / 1000.0,
    )
    if get_magnitude(velocity) <= 1e-6:
        acceleration = Vector2D(x=0.0, y=0.0)
    return EgoStateStamped(
        timestamp=ego_state.timestamp + dt,
        state=EgoState(
            pos=pos,
            velocity=velocity,
            acceleration=acceleration,
            yaw=ego_state.state.yaw,
            steering_angle=ego_state.state.steering_angle,
        ),
    )


def _waiting_ego_trajectory(
    ego_state: EgoStateStamped,
    prediction_horizon: int,
    dt: int,
) -> List[EgoStateStamped]:
    """Build a waiting ego trajectory from the stopped-lead decision moment."""
    return [
        EgoStateStamped(
            timestamp=ego_state.timestamp + timestamp,
            state=EgoState(
                pos=ego_state.state.pos,
                velocity=Vector2D(x=0.0, y=0.0),
                acceleration=Vector2D(x=0.0, y=0.0),
                yaw=ego_state.state.yaw,
                steering_angle=ego_state.state.steering_angle,
            ),
        )
        for timestamp in range(0, prediction_horizon + dt, dt)
    ]


def _clearance_series(
    predicted_env: PredictedEnvironment,
    ego_trajectory: List[EgoStateStamped],
    oncoming_id: int,
) -> tuple["np.ndarray", "np.ndarray"]:
    """Return time since decision and signed ego-to-oncoming clearance."""
    oncoming_predictions = {
        obj.timestamp: obj
        for obj in predicted_env.objects.get(oncoming_id, [])
    }
    times = []
    clearances = []
    start_timestamp = ego_trajectory[0].timestamp
    for ego_at_t in ego_trajectory:
        oncoming_at_t = oncoming_predictions.get(ego_at_t.timestamp)
        if oncoming_at_t is None:
            continue
        times.append(ego_at_t.timestamp - start_timestamp)
        clearances.append(
            get_signed_magnitude(
                Vector2D(
                    x=oncoming_at_t.state.pos.x - ego_at_t.state.pos.x,
                    y=oncoming_at_t.state.pos.y - ego_at_t.state.pos.y,
                ),
                oncoming_at_t.state.yaw,
            )
        )
    return np.array(times), np.array(clearances)


def _lane_y_offsets(
    environment: Environment,
    ego_state: EgoStateStamped,
) -> List[tuple[int, float]]:
    """Return lane centerline y offsets relative to the decision ego frame."""
    lane_offsets = []
    for lane in environment.lanes:
        if not lane.centerline:
            continue
        nearest_point = min(
            (point for point, _ in lane.centerline),
            key=lambda point: math.hypot(
                point.x - ego_state.state.pos.x,
                point.y - ego_state.state.pos.y,
            ),
        )
        _, local_y = _to_ego_frame(nearest_point.x, nearest_point.y, ego_state)
        lane_offsets.append((lane.id, local_y))
    return lane_offsets


def _resolve_overtake_replay_behavior(
    cfg,
    environment: Environment,
    ego_state: EgoStateStamped,
    lead: DynamicObjectStamped,
) -> OvertakeReplayBehavior:
    """Build visualization behavior from scenario, vehicle, and planner config."""
    safety_distance = float(_cfg_lookup(cfg, "scenario.behavior.safety_distance_m", 15.0))
    lead_brake_acceleration = _lead_braking_acceleration(cfg)
    ego_max_deceleration = abs(float(cfg.vehicle.max_deceleration))
    if ego_max_deceleration <= 0.0:
        ego_max_deceleration = abs(lead_brake_acceleration)

    rel_x, _, _ = global_to_ego_axis(
        lead.state.pos.x,
        lead.state.pos.y,
        ego_state.state.pos.x,
        ego_state.state.pos.y,
        ego_state.state.yaw,
    )
    configured_gap = float(
        _cfg_lookup(
            cfg,
            "scenario.behavior.ego_follow_gap_m",
            min(rel_x * 0.5, safety_distance),
        )
    )
    ego_follow_gap = max(5.0, min(rel_x - 1.0, configured_gap)) if rel_x > 6.0 else 5.0
    visualization_dt = int(_cfg_lookup(cfg, "scenario.behavior.visualization_dt_ms", cfg.planner.dt_sim))
    visualization_dt = max(100, visualization_dt)
    plot_horizon = _ceil_ms_to_step(
        int(_cfg_lookup(cfg, "scenario.behavior.plot_horizon_ms", 6000)),
        visualization_dt,
    )

    behavior = OvertakeReplayBehavior(
        lead_cruise_time_ms=0,
        lead_brake_acceleration=lead_brake_acceleration,
        ego_follow_gap_m=ego_follow_gap,
        ego_max_acceleration=float(cfg.vehicle.max_acceleration),
        ego_max_deceleration=ego_max_deceleration,
        safety_distance_m=safety_distance,
        oncoming_clear_after_lead_stop_ms=int(
            _cfg_lookup(cfg, "scenario.behavior.oncoming_clear_after_lead_stop_ms", 4800)
        ),
        visualization_dt_ms=visualization_dt,
        plot_horizon_ms=plot_horizon,
        plot_lead_stop_x_m=float(
            _cfg_lookup(cfg, "scenario.behavior.plot_lead_stop_x_m", 36.0)
        ),
    )

    raw_cruise_time = _cfg_lookup(cfg, "scenario.behavior.lead_cruise_time_ms", "auto")
    lead_brake_duration_ms = _estimate_stop_time_ms(lead, behavior.lead_brake_acceleration)
    if lead_brake_duration_ms is None:
        raise ValueError("Cannot build replay: lead vehicle cannot brake to a stop.")

    if _is_auto_config_value(raw_cruise_time):
        lead_cruise_time_ms = _resolve_dynamic_lead_cruise_time_ms(
            environment=environment,
            ego_state=ego_state,
            lead=lead,
            behavior_template=behavior,
            lead_brake_duration_ms=lead_brake_duration_ms,
            dt=visualization_dt,
        )
    else:
        lead_cruise_time_ms = max(0, _ceil_ms_to_step(int(raw_cruise_time), visualization_dt))

    return replace(behavior, lead_cruise_time_ms=lead_cruise_time_ms)


def _oncoming_clear_after_candidate_stop_ms(
    ego_state: EgoStateStamped,
    lead: DynamicObjectStamped,
    oncoming: DynamicObjectStamped,
    behavior_template: OvertakeReplayBehavior,
    lead_brake_duration_ms: int,
    candidate_stop_ms: int,
    dt: int,
) -> Optional[int]:
    """Replay one candidate lead-stop time and return oncoming clear time after it."""
    behavior = replace(
        behavior_template,
        lead_cruise_time_ms=max(0, candidate_stop_ms - lead_brake_duration_ms),
    )
    decision_offset_ms = _ceil_ms_to_step(
        behavior.lead_cruise_time_ms + lead_brake_duration_ms,
        dt,
    )

    ego_at_t = ego_state
    for elapsed_ms in range(0, decision_offset_ms, dt):
        next_lead_state = _lead_policy_state(lead, elapsed_ms + dt, behavior)
        ego_at_t = _step_ego_follow_policy(ego_at_t, next_lead_state, behavior, dt)

    decision_ego = EgoStateStamped(
        timestamp=ego_state.timestamp + decision_offset_ms,
        state=EgoState(
            pos=ego_at_t.state.pos,
            velocity=Vector2D(x=0.0, y=0.0),
            acceleration=Vector2D(x=0.0, y=0.0),
            yaw=ego_at_t.state.yaw,
            steering_angle=ego_at_t.state.steering_angle,
        ),
    )
    oncoming_at_decision = _constant_acceleration_object_state(oncoming, decision_offset_ms)
    return _estimate_oncoming_clear_time_ms(
        decision_ego,
        oncoming_at_decision,
        behavior.safety_distance_m,
    )


def _resolve_dynamic_lead_cruise_time_ms(
    environment: Environment,
    ego_state: EgoStateStamped,
    lead: DynamicObjectStamped,
    behavior_template: OvertakeReplayBehavior,
    lead_brake_duration_ms: int,
    dt: int,
) -> int:
    """Choose lead cruise time so the unchanged oncoming car reaches the decision window."""
    oncoming = find_oncoming_vehicle(
        environment,
        ego_state,
        opposite_lane_y_tolerance=5.0,
        max_lookahead_m=float("inf"),
    )
    if oncoming is None:
        raise ValueError("Cannot auto-time replay: no oncoming vehicle found.")

    target_clear_ms = max(0, behavior_template.oncoming_clear_after_lead_stop_ms)
    min_stop_ms = _ceil_ms_to_step(lead_brake_duration_ms, dt)
    oncoming_speed = max(get_magnitude(oncoming.state.velocity), 1e-6)
    initial_oncoming_distance = math.hypot(
        oncoming.state.pos.x - ego_state.state.pos.x,
        oncoming.state.pos.y - ego_state.state.pos.y,
    )
    max_stop_ms = _ceil_ms_to_step(
        1000.0 * initial_oncoming_distance / oncoming_speed + target_clear_ms + 60000,
        dt,
    )

    previous_stop_ms = min_stop_ms
    hi_stop_ms = min_stop_ms
    coarse_step_ms = max(dt, 5000)
    while hi_stop_ms <= max_stop_ms:
        clear_after_stop_ms = _oncoming_clear_after_candidate_stop_ms(
            ego_state,
            lead,
            oncoming,
            behavior_template,
            lead_brake_duration_ms,
            hi_stop_ms,
            dt,
        )
        if clear_after_stop_ms is not None and clear_after_stop_ms <= target_clear_ms:
            break
        previous_stop_ms = hi_stop_ms
        hi_stop_ms += coarse_step_ms
    else:
        raise ValueError("Cannot auto-time replay: oncoming never reaches the target window.")

    lo_stop_ms = previous_stop_ms
    while hi_stop_ms - lo_stop_ms > dt:
        mid_stop_ms = ((lo_stop_ms + hi_stop_ms) // 2 // dt) * dt
        if mid_stop_ms <= lo_stop_ms:
            mid_stop_ms = lo_stop_ms + dt
        clear_after_stop_ms = _oncoming_clear_after_candidate_stop_ms(
            ego_state,
            lead,
            oncoming,
            behavior_template,
            lead_brake_duration_ms,
            mid_stop_ms,
            dt,
        )
        if clear_after_stop_ms is None or clear_after_stop_ms > target_clear_ms:
            lo_stop_ms = mid_stop_ms
        else:
            hi_stop_ms = mid_stop_ms

    return max(0, int(hi_stop_ms - lead_brake_duration_ms))


def _combined_object_predictions(
    before_decision: PredictedEnvironment,
    after_decision: PredictedEnvironment,
    object_id: int,
) -> List[DynamicObjectStamped]:
    """Return object predictions before and after the decision without duplicate timestamps."""
    combined = {
        obj.timestamp: obj
        for obj in before_decision.objects.get(object_id, [])
    }
    combined.update(
        {
            obj.timestamp: obj
            for obj in after_decision.objects.get(object_id, [])
        }
    )
    return [combined[timestamp] for timestamp in sorted(combined)]


def _ego_overtake_preview(
    decision_ego: EgoStateStamped,
    start_delay_ms: int,
    cfg,
) -> List[EgoStateStamped]:
    """Roll out an ego overtake preview using the project's bicycle overtake phases."""
    from models.models import EgoInput
    from motion_planner.bicylce_model2 import OVERTAKE_PHASES, kinematic_bicycle_model

    dt = int(_cfg_lookup(cfg, "scenario.behavior.overtake_preview_dt_ms", 100))
    dt = max(20, dt)
    acceleration = float(
        _cfg_lookup(cfg, "scenario.behavior.overtake_preview_acceleration", 2.0)
    )
    start_delay_ms = max(0, _ceil_ms_to_step(start_delay_ms, dt))

    stopped_state = EgoStateStamped(
        timestamp=decision_ego.timestamp + start_delay_ms,
        state=EgoState(
            pos=Vector2D(x=decision_ego.state.pos.x, y=decision_ego.state.pos.y),
            velocity=Vector2D(x=0.0, y=0.0),
            acceleration=Vector2D(x=0.0, y=0.0),
            yaw=decision_ego.state.yaw,
            steering_angle=0.0,
        ),
    )
    preview = [stopped_state]
    current = stopped_state

    for duration_s, steering_deg in OVERTAKE_PHASES:
        phase_ms = int(round(duration_s * 1000.0))
        steps = max(1, int(math.ceil(phase_ms / dt)))
        for _ in range(steps):
            speed = get_magnitude(current.state.velocity)
            control = EgoInput(
                steering_angle=math.radians(-steering_deg),
                acceleration=acceleration if speed < 8.0 else 0.0,
            )
            current = kinematic_bicycle_model(
                current,
                control,
                dt,
                cfg.vehicle,
                limit_steer_rate=True,
            )
            preview.append(current)

    return preview


def _decision_environment_at_lead_stop(
    environment: Environment,
    ego_state: EgoStateStamped,
    cfg,
) -> tuple[
    Environment,
    EgoStateStamped,
    PredictedEnvironment,
    List[EgoStateStamped],
    int,
    int,
    OvertakeReplayBehavior,
]:
    """Replay current stack data through lead cruise, braking, and ego following."""
    lead = find_lead_vehicle(environment, ego_state)
    if lead is None:
        raise ValueError("Cannot build overtake replay: no lead vehicle found.")

    behavior = _resolve_overtake_replay_behavior(cfg, environment, ego_state, lead)
    dt = behavior.visualization_dt_ms
    lead_brake_duration_ms = _estimate_stop_time_ms(lead, behavior.lead_brake_acceleration)
    if lead_brake_duration_ms is None:
        raise ValueError("Cannot build replay: lead vehicle cannot brake to a stop.")

    lead_stop_estimate_ms = behavior.lead_cruise_time_ms + lead_brake_duration_ms
    decision_offset_ms = _ceil_ms_to_step(lead_stop_estimate_ms, dt)
    replay_objects: Dict[int, List[DynamicObjectStamped]] = {
        obj.state.id: [] for obj in environment.objects
    }
    ego_replay: List[EgoStateStamped] = [ego_state]

    for elapsed_ms in range(0, decision_offset_ms + dt, dt):
        for obj in environment.objects:
            if obj.state.id == lead.state.id:
                replay_obj = _lead_policy_state(lead, elapsed_ms, behavior)
            else:
                replay_obj = _constant_acceleration_object_state(obj, elapsed_ms)
            replay_objects[obj.state.id].append(replay_obj)

        if elapsed_ms < decision_offset_ms:
            next_lead_state = _lead_policy_state(lead, elapsed_ms + dt, behavior)
            ego_replay.append(
                _step_ego_follow_policy(ego_replay[-1], next_lead_state, behavior, dt)
            )

    replay_prediction = PredictedEnvironment(
        objects=replay_objects,
        lanes=environment.lanes,
        dt=dt,
        horizon=decision_offset_ms,
    )
    decision_timestamp = ego_state.timestamp + decision_offset_ms
    decision_objects = objects_at_time(replay_prediction, decision_timestamp)
    if not decision_objects:
        raise ValueError("Cannot build overtake replay: no objects at decision timestamp.")

    stopped_objects = []
    for obj in decision_objects:
        velocity = obj.state.velocity
        if obj.state.id == lead.state.id:
            velocity = Vector2D(x=0.0, y=0.0)
        stopped_objects.append(
            _object_with_state(
                obj=obj,
                timestamp=decision_timestamp,
                pos=obj.state.pos,
                velocity=velocity,
                acceleration=Vector2D(x=0.0, y=0.0),
            )
        )

    ego_at_decision = ego_replay[-1]
    decision_ego = EgoStateStamped(
        timestamp=decision_timestamp,
        state=EgoState(
            pos=ego_at_decision.state.pos,
            velocity=Vector2D(x=0.0, y=0.0),
            acceleration=Vector2D(x=0.0, y=0.0),
            yaw=ego_at_decision.state.yaw,
            steering_angle=ego_at_decision.state.steering_angle,
        ),
    )

    return (
        Environment(objects=stopped_objects, lanes=environment.lanes),
        decision_ego,
        replay_prediction,
        ego_replay,
        decision_offset_ms,
        lead_stop_estimate_ms,
        behavior,
    )


def visualize_overtake_decision_replay(
    output_path: str = "prediction_visualization.png",
    show: bool = True,
    config_overrides: Optional[List[str]] = None,
) -> None:
    """
    Event-based verification plot for the overtake decision.

    The current scenario is replayed until the lead vehicle stops. From that
    decision moment, ego waits behind the lead vehicle while prediction checks
    when the oncoming vehicle has passed far enough for overtaking.
    """
    import matplotlib.pyplot as plt
    from matplotlib import colors
    from simulation.simulate import Simulation

    cfg = _load_prediction_visualization_cfg(
        config_overrides or ["scenario=overtake_scenario"]
    )
    simulation = Simulation(cfg)
    initial_environment = simulation.get_environment(noise_level=0.0)
    initial_ego = simulation.get_ego_state(noise_level=0.0)

    (
        decision_environment,
        decision_ego,
        replay_prediction,
        ego_replay,
        decision_offset_ms,
        lead_stop_estimate_ms,
        behavior,
    ) = _decision_environment_at_lead_stop(
        environment=initial_environment,
        ego_state=initial_ego,
        cfg=cfg,
    )
    dt = behavior.visualization_dt_ms
    safety_distance = behavior.safety_distance_m

    lead = find_lead_vehicle(decision_environment, decision_ego)
    oncoming = find_oncoming_vehicle(
        decision_environment,
        decision_ego,
        opposite_lane_y_tolerance=5.0,
        max_lookahead_m=float("inf"),
    )
    if lead is None:
        raise ValueError("Cannot evaluate overtake decision: no stopped lead vehicle found.")
    if oncoming is None:
        raise ValueError("Cannot evaluate overtake decision: no oncoming vehicle found.")

    oncoming_clear_estimate_ms = _estimate_oncoming_clear_time_ms(
        decision_ego,
        oncoming,
        safety_distance,
    )
    prediction_horizon = max(
        behavior.plot_horizon_ms,
        _ceil_ms_to_step((oncoming_clear_estimate_ms or 0) + dt, dt),
    )

    decision_prediction = predict_environment(
        environment=decision_environment,
        prediction_horizon=prediction_horizon,
        dt=dt,
        model="constant_acceleration",
        accelerations={
            obj.state.id: Vector2D(x=0.0, y=0.0)
            for obj in decision_environment.objects
        },
    )
    waiting_ego = _waiting_ego_trajectory(decision_ego, prediction_horizon, dt)

    overtake_now_safe = is_overtake_gap_safe(
        predicted_environment=decision_prediction,
        ego_trajectory=waiting_ego,
        lead_vehicle_id=lead.state.id,
        overtake_start_ms=decision_ego.timestamp,
        overtake_duration_ms=prediction_horizon,
        safety_distance=safety_distance,
    )

    clearance_ts, signed_clearance = _clearance_series(
        decision_prediction,
        waiting_ego,
        oncoming.state.id,
    )
    unsafe_mask = signed_clearance < safety_distance
    clear_indices = np.flatnonzero(~unsafe_mask)
    oncoming_clear_time_ms = int(clearance_ts[clear_indices[0]]) if len(clear_indices) else None
    ego_overtake_preview = _ego_overtake_preview(
        decision_ego,
        oncoming_clear_time_ms or behavior.oncoming_clear_after_lead_stop_ms,
        cfg,
    )

    fig, (ax_scene, ax_gap) = plt.subplots(
        2,
        1,
        figsize=(11, 9),
        height_ratios=[1.0, 1.1],
    )

    for lane_id, lane_y in _lane_y_offsets(decision_environment, decision_ego):
        ax_scene.axhline(
            lane_y,
            color="#b8b8b8",
            linestyle="--",
            linewidth=1.0,
            label="_nolegend_",
            zorder=1,
        )

    lead_x, lead_y = _to_ego_frame(lead.state.pos.x, lead.state.pos.y, decision_ego)
    display_shift_x = behavior.plot_lead_stop_x_m - lead_x
    color_norm = colors.Normalize(vmin=0, vmax=behavior.plot_horizon_ms)

    ego_replay_xs = np.array([state.state.pos.x for state in ego_replay])
    ego_replay_ys = np.array([state.state.pos.y for state in ego_replay])
    ego_replay_rel_xs, ego_replay_rel_ys = _arrays_to_ego_frame(
        ego_replay_xs,
        ego_replay_ys,
        decision_ego,
    )
    ego_display_xs = ego_replay_rel_xs + display_shift_x
    ego_mask = (0.0 <= ego_display_xs) & (ego_display_xs <= 50.0)
    if not np.any(ego_mask):
        ego_mask = np.ones_like(ego_display_xs, dtype=bool)
    ax_scene.plot(
        ego_display_xs[ego_mask],
        ego_replay_rel_ys[ego_mask],
        color="black",
        linewidth=2,
        label="Ego path",
        zorder=3,
    )
    ax_scene.scatter(
        ego_display_xs[ego_mask],
        ego_replay_rel_ys[ego_mask],
        c="black",
        s=22,
        zorder=4,
    )
    preview_xs = np.array([state.state.pos.x for state in ego_overtake_preview])
    preview_ys = np.array([state.state.pos.y for state in ego_overtake_preview])
    preview_rel_xs, preview_rel_ys = _arrays_to_ego_frame(
        preview_xs,
        preview_ys,
        decision_ego,
    )
    preview_display_xs = preview_rel_xs + display_shift_x
    ax_scene.plot(
        preview_display_xs,
        preview_rel_ys,
        color="black",
        linestyle="--",
        linewidth=2,
        label="Ego future overtake path",
        zorder=4,
    )

    lead_replay_ts, lead_replay_xs, lead_replay_ys, _ = _flatten_predictions(
        replay_prediction,
        lead.state.id,
    )
    lead_replay_rel_xs, lead_replay_rel_ys = _arrays_to_ego_frame(
        lead_replay_xs,
        lead_replay_ys,
        decision_ego,
    )
    lead_display_xs = lead_replay_rel_xs + display_shift_x
    lead_brake_elapsed = lead_replay_ts - (
        initial_ego.timestamp + behavior.lead_cruise_time_ms
    )
    lead_mask = (
        (0 <= lead_brake_elapsed)
        & (lead_brake_elapsed <= behavior.plot_horizon_ms)
        & (0.0 <= lead_display_xs)
        & (lead_display_xs <= 50.0)
    )
    ax_scene.plot(
        lead_display_xs[lead_mask],
        lead_replay_rel_ys[lead_mask],
        color="tab:purple",
        linewidth=1.5,
        alpha=0.8,
        label="Lead vehicle braking to stop",
        zorder=2,
    )
    ax_scene.scatter(
        lead_display_xs[lead_mask],
        lead_replay_rel_ys[lead_mask],
        c=lead_brake_elapsed[lead_mask],
        cmap=plt.cm.viridis,
        norm=color_norm,
        marker="s",
        s=55,
        edgecolors="black",
        linewidths=0.4,
        zorder=3,
    )
    ax_scene.scatter(
        [lead_x + display_shift_x],
        [lead_y],
        facecolors="none",
        edgecolors="red",
        linewidths=2.0,
        s=160,
        label="Lead stopped",
        zorder=5,
    )

    oncoming_ts, oncoming_xs, oncoming_ys, _ = _flatten_predictions(
        decision_prediction,
        oncoming.state.id,
    )
    oncoming_rel_xs, oncoming_rel_ys = _arrays_to_ego_frame(
        oncoming_xs,
        oncoming_ys,
        decision_ego,
    )
    oncoming_display_xs = oncoming_rel_xs + display_shift_x
    oncoming_elapsed = oncoming_ts - decision_ego.timestamp
    oncoming_mask = (
        (0 <= oncoming_elapsed)
        & (oncoming_elapsed <= behavior.plot_horizon_ms)
        & (0.0 <= oncoming_display_xs)
        & (oncoming_display_xs <= 85.0)
    )
    if not np.any(oncoming_mask):
        oncoming_mask = np.ones_like(oncoming_display_xs, dtype=bool)
    oncoming_scatter = ax_scene.scatter(
        oncoming_display_xs[oncoming_mask],
        oncoming_rel_ys[oncoming_mask],
        c=oncoming_elapsed[oncoming_mask],
        cmap=plt.cm.viridis,
        norm=color_norm,
        s=80,
        marker="^",
        edgecolors="black",
        linewidths=0.5,
        label="Oncoming vehicle constant acceleration",
        zorder=3,
    )
    color_bar = fig.colorbar(oncoming_scatter, ax=ax_scene, pad=0.01)
    color_bar.set_label("Prediction time [ms]")

    ax_scene.set_xlim(-4.0, max(84.0, float(np.max(preview_display_xs)) + 5.0))
    ax_scene.set_ylim(-25.0, 25.0)
    ax_scene.set_xlabel("x [m]")
    ax_scene.set_ylabel("y [m]")
    ax_scene.set_title("Predicted trajectories: color encodes prediction time")
    ax_scene.legend(loc="upper left", fontsize=8)
    ax_scene.grid(True, alpha=0.3)

    combined_oncoming = _combined_object_predictions(
        replay_prediction,
        decision_prediction,
        oncoming.state.id,
    )
    oncoming_full_xs = np.array([obj.state.pos.x for obj in combined_oncoming])
    oncoming_full_ys = np.array([obj.state.pos.y for obj in combined_oncoming])
    oncoming_full_ts = np.array([obj.timestamp for obj in combined_oncoming])

    _, lead_full_xs, lead_full_ys, _ = _flatten_predictions(
        replay_prediction,
        lead.state.id,
    )
    ax_inset = ax_scene.inset_axes([0.48, 0.08, 0.48, 0.26])
    for lane in decision_environment.lanes:
        if lane.centerline:
            lane_y = lane.centerline[0][0].y
            ax_inset.axhline(
                lane_y,
                color="#b8b8b8",
                linestyle="--",
                linewidth=0.8,
                zorder=1,
            )
    ax_inset.plot(
        oncoming_full_xs,
        oncoming_full_ys,
        color="tab:purple",
        linewidth=1.2,
        zorder=2,
    )
    sample_step = max(1, len(oncoming_full_xs) // 30)
    ax_inset.scatter(
        oncoming_full_xs[::sample_step],
        oncoming_full_ys[::sample_step],
        c=oncoming_full_ts[::sample_step] - initial_ego.timestamp,
        cmap=plt.cm.viridis,
        s=22,
        marker="^",
        edgecolors="black",
        linewidths=0.2,
        zorder=3,
    )
    ax_inset.plot(
        lead_full_xs,
        lead_full_ys,
        color="tab:blue",
        linewidth=1.0,
        zorder=2,
    )
    ax_inset.scatter(
        [oncoming_full_xs[0]],
        [oncoming_full_ys[0]],
        c="tab:purple",
        marker="^",
        s=45,
        edgecolors="black",
        zorder=5,
    )
    ax_inset.scatter(
        [lead.state.pos.x],
        [lead.state.pos.y],
        facecolors="none",
        edgecolors="red",
        linewidths=1.2,
        s=55,
        zorder=5,
    )
    ax_inset.set_xlim(
        min(float(np.min(preview_xs)), decision_ego.state.pos.x) - 30.0,
        float(np.max(oncoming_full_xs)) + 40.0,
    )
    ax_inset.set_ylim(-2.0, 10.0)
    ax_inset.set_title("Full path: oncoming starts at x = 1800 m", fontsize=8)
    ax_inset.tick_params(labelsize=7)
    ax_inset.grid(True, alpha=0.25)

    ax_gap.plot(
        clearance_ts,
        signed_clearance,
        color="tab:red",
        linewidth=2,
        label="Signed ego-to-oncoming clearance",
    )
    ax_gap.axhline(
        safety_distance,
        color="black",
        linestyle="--",
        label=f"Safety distance ({safety_distance} m)",
    )
    ax_gap.fill_between(
        clearance_ts,
        signed_clearance,
        safety_distance,
        where=unsafe_mask,
        color="red",
        alpha=0.25,
        label="Unsafe to overtake",
    )
    ax_gap.axvline(
        0,
        color="tab:blue",
        linestyle=":",
        label="Lead stopped / decision point",
    )
    if oncoming_clear_time_ms is not None:
        ax_gap.axvline(
            oncoming_clear_time_ms,
            color="tab:green",
            linestyle=":",
            label=f"Oncoming clear ({oncoming_clear_time_ms} ms)",
        )
    ax_gap.set_xlim(0, behavior.plot_horizon_ms)
    ax_gap.set_xlabel("Time after lead stop [ms]")
    ax_gap.set_ylabel("Signed clearance [m]")
    ax_gap.set_title(
        f"Overtake gap safety over time: is_overtake_gap_safe = {overtake_now_safe}"
    )
    ax_gap.legend(loc="lower center", fontsize=8)
    ax_gap.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    if show:
        plt.show()
    else:
        plt.close(fig)


def visualize_prediction(
    output_path: str = "prediction_visualization.png",
    show: bool = True,
    config_overrides: Optional[List[str]] = None,
) -> None:
    """Backward-compatible entry point for the overtake decision replay."""
    visualize_overtake_decision_replay(
        output_path=output_path,
        show=show,
        config_overrides=config_overrides,
    )


if __name__ == "__main__":
    visualize_overtake_decision_replay(config_overrides=["scenario=overtake_scenario"])
