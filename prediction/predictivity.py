import math
import sys
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
    deceleration = float(cfg.vehicle.max_deceleration)
    return deceleration if deceleration < 0.0 else -deceleration


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


def _ego_at_decision(
    ego_state: EgoStateStamped,
    decision_timestamp: int,
) -> EgoStateStamped:
    """
    Advance ego to the decision moment, then make it wait.

    This models the workflow under test: ego follows the lead vehicle, reaches
    the stopped-lead decision point, and waits before overtaking.
    """
    time_s = (decision_timestamp - ego_state.timestamp) / 1000.0
    pos, _ = _predict_constant_acceleration_step(
        position=ego_state.state.pos,
        velocity=ego_state.state.velocity,
        acceleration=ego_state.state.acceleration,
        time_s=time_s,
    )
    return EgoStateStamped(
        timestamp=decision_timestamp,
        state=EgoState(
            pos=pos,
            velocity=Vector2D(x=0.0, y=0.0),
            acceleration=Vector2D(x=0.0, y=0.0),
            yaw=ego_state.state.yaw,
            steering_angle=0.0,
        ),
    )


def _ego_replay_trajectory(
    ego_state: EgoStateStamped,
    replay_horizon: int,
    dt: int,
) -> List[EgoStateStamped]:
    """Replay ego before the lead-stop decision using the current ego state."""
    replay_states = []
    for timestamp in range(0, replay_horizon + dt, dt):
        time_s = timestamp / 1000.0
        pos, velocity = _predict_constant_acceleration_step(
            position=ego_state.state.pos,
            velocity=ego_state.state.velocity,
            acceleration=ego_state.state.acceleration,
            time_s=time_s,
        )
        replay_states.append(
            EgoStateStamped(
                timestamp=ego_state.timestamp + timestamp,
                state=EgoState(
                    pos=pos,
                    velocity=velocity,
                    acceleration=ego_state.state.acceleration,
                    yaw=ego_state.state.yaw,
                    steering_angle=ego_state.state.steering_angle,
                ),
            )
        )
    return replay_states


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


def _decision_environment_at_lead_stop(
    environment: Environment,
    ego_state: EgoStateStamped,
    cfg,
    dt: int,
) -> tuple[Environment, EgoStateStamped, PredictedEnvironment, List[EgoStateStamped], int, int]:
    """Replay current stack data to the moment the lead vehicle stops."""
    lead = find_lead_vehicle(environment, ego_state)
    if lead is None:
        raise ValueError("Cannot build overtake replay: no lead vehicle found.")

    braking_acceleration = _lead_braking_acceleration(cfg)
    current_acceleration = _longitudinal_acceleration(lead)
    if current_acceleration < -1e-6:
        braking_acceleration = current_acceleration

    lead_stop_estimate_ms = _estimate_stop_time_ms(lead, braking_acceleration)
    if lead_stop_estimate_ms is None:
        raise ValueError("Cannot build overtake replay: lead vehicle is not braking to a stop.")

    decision_offset_ms = _ceil_ms_to_step(lead_stop_estimate_ms, dt)
    replay_accelerations: Dict[int, Vector2D | float] = {
        obj.state.id: obj.state.acceleration for obj in environment.objects
    }
    replay_accelerations[lead.state.id] = braking_acceleration

    replay_prediction = predict_environment(
        environment=environment,
        prediction_horizon=decision_offset_ms,
        dt=dt,
        model="constant_acceleration",
        accelerations=replay_accelerations,
    )
    decision_timestamp = ego_state.timestamp + decision_offset_ms
    decision_objects = objects_at_time(replay_prediction, decision_timestamp)
    if not decision_objects:
        raise ValueError("Cannot build overtake replay: no objects at decision timestamp.")

    stopped_objects = []
    for obj in decision_objects:
        acceleration = Vector2D(x=0.0, y=0.0)
        velocity = obj.state.velocity
        if obj.state.id == lead.state.id:
            velocity = Vector2D(x=0.0, y=0.0)
        stopped_objects.append(
            _object_with_state(
                obj=obj,
                timestamp=decision_timestamp,
                pos=obj.state.pos,
                velocity=velocity,
                acceleration=acceleration,
            )
        )

    decision_environment = Environment(objects=stopped_objects, lanes=environment.lanes)
    decision_ego = _ego_at_decision(ego_state, decision_timestamp)
    ego_replay = _ego_replay_trajectory(
        ego_state=ego_state,
        replay_horizon=decision_offset_ms,
        dt=dt,
    )
    return (
        decision_environment,
        decision_ego,
        replay_prediction,
        ego_replay,
        decision_offset_ms,
        lead_stop_estimate_ms,
    )


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
    from simulation.simulate import Simulation

    cfg = _load_prediction_visualization_cfg(
        config_overrides or ["scenario=overtake_scenario"]
    )
    simulation = Simulation(cfg)
    initial_environment = simulation.get_environment(noise_level=0.0)
    initial_ego = simulation.get_ego_state(noise_level=0.0)
    dt = int(cfg.planner.dt_sim)
    safety_distance = 15.0

    (
        decision_environment,
        decision_ego,
        replay_prediction,
        ego_replay,
        decision_offset_ms,
        lead_stop_estimate_ms,
    ) = _decision_environment_at_lead_stop(
        environment=initial_environment,
        ego_state=initial_ego,
        cfg=cfg,
        dt=dt,
    )

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
    if oncoming_clear_estimate_ms is None:
        prediction_horizon = int(cfg.planner.horizon)
    else:
        prediction_horizon = _ceil_ms_to_step(oncoming_clear_estimate_ms + 5000, dt)

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

    fig, (ax_scene, ax_gap) = plt.subplots(
        2,
        1,
        figsize=(11, 9),
        height_ratios=[1.0, 1.25],
    )

    for lane_id, lane_y in _lane_y_offsets(decision_environment, decision_ego):
        ax_scene.axhline(
            lane_y,
            color="#b8b8b8",
            linestyle="--",
            linewidth=1.0,
            label=f"Lane {lane_id}",
            zorder=1,
        )

    ego_replay_xs = np.array([state.state.pos.x for state in ego_replay])
    ego_replay_ys = np.array([state.state.pos.y for state in ego_replay])
    ego_replay_rel_xs, ego_replay_rel_ys = _arrays_to_ego_frame(
        ego_replay_xs,
        ego_replay_ys,
        decision_ego,
    )
    ax_scene.plot(
        ego_replay_rel_xs,
        ego_replay_rel_ys,
        color="black",
        linewidth=2,
        label="Ego before lead stops",
        zorder=3,
    )
    ax_scene.scatter(
        ego_replay_rel_xs,
        ego_replay_rel_ys,
        c="black",
        s=22,
        zorder=4,
    )
    ax_scene.scatter([0.0], [0.0], c="black", s=80, label="Ego waiting", zorder=5)

    lead_x, lead_y = _to_ego_frame(lead.state.pos.x, lead.state.pos.y, decision_ego)
    lead_replay_ts, lead_replay_xs, lead_replay_ys, _ = _flatten_predictions(
        replay_prediction,
        lead.state.id,
    )
    lead_replay_rel_xs, lead_replay_rel_ys = _arrays_to_ego_frame(
        lead_replay_xs,
        lead_replay_ys,
        decision_ego,
    )
    ax_scene.plot(
        lead_replay_rel_xs,
        lead_replay_rel_ys,
        color="tab:purple",
        linewidth=1.5,
        alpha=0.8,
        label="Lead braking history",
        zorder=2,
    )
    ax_scene.scatter(
        lead_replay_rel_xs,
        lead_replay_rel_ys,
        c=lead_replay_ts - initial_ego.timestamp,
        cmap=plt.cm.plasma,
        marker="s",
        s=55,
        edgecolors="black",
        linewidths=0.4,
        zorder=3,
    )
    ax_scene.scatter(
        [lead_x],
        [lead_y],
        c="tab:purple",
        marker="s",
        s=100,
        label="Lead stopped",
        zorder=4,
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
    local_mask = (-150.0 <= oncoming_rel_xs) & (oncoming_rel_xs <= 220.0)
    oncoming_scatter = ax_scene.scatter(
        oncoming_rel_xs[local_mask],
        oncoming_rel_ys[local_mask],
        c=oncoming_ts[local_mask] - decision_ego.timestamp,
        cmap=plt.cm.viridis,
        s=80,
        marker="^",
        edgecolors="black",
        linewidths=0.5,
        label="Oncoming prediction near ego",
        zorder=3,
    )
    color_bar = fig.colorbar(oncoming_scatter, ax=ax_scene, pad=0.01)
    color_bar.set_label("Time after lead stop [ms]")

    decision_label = "WAIT" if not overtake_now_safe else "OVERTAKE"
    ax_scene.text(
        0.02,
        0.95,
        (
            f"Lead stop replay: {decision_offset_ms} ms "
            f"(estimate {lead_stop_estimate_ms} ms)\n"
            f"Initial gap: {lead_replay_rel_xs[0] - ego_replay_rel_xs[0]:.1f} m\n"
            f"Decision at lead stop: {decision_label}\n"
            f"Oncoming clear: {oncoming_clear_time_ms} ms after decision"
        ),
        transform=ax_scene.transAxes,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
    )
    ax_scene.set_xlim(-150.0, 220.0)
    ax_scene.set_ylim(-4.0, 8.0)
    ax_scene.set_xlabel("x relative to waiting ego [m]")
    ax_scene.set_ylabel("y relative to waiting ego [m]")
    ax_scene.set_title("Decision replay: lead stopped, ego waits, oncoming passes")
    ax_scene.legend(loc="upper right", fontsize=8)
    ax_scene.grid(True, alpha=0.3)

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
    ax_gap.set_xlabel("Time after lead stop [ms]")
    ax_gap.set_ylabel("Signed clearance [m]")
    ax_gap.set_title(
        f"Overtake decision after lead stops: safe now = {overtake_now_safe}"
    )
    ax_gap.legend(loc="best", fontsize=8)
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
