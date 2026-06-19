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
    ObjectType,
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


# Visual sanity-check for the straight-road overtake prediction scenario.
def _flatten_predictions(
    predicted_env: PredictedEnvironment,
    object_id: int,
) -> tuple["np.ndarray", "np.ndarray", "np.ndarray", "np.ndarray"]:
    """Return timestamps, x positions, y positions, and x velocities."""
    import numpy as np

    predictions = sorted(
        predicted_env.objects[object_id],
        key=lambda obj: obj.timestamp,
    )
    timestamps = np.array([obj.timestamp for obj in predictions])
    xs = np.array([obj.state.pos.x for obj in predictions])
    ys = np.array([obj.state.pos.y for obj in predictions])
    vxs = np.array([obj.state.velocity.x for obj in predictions])
    return timestamps, xs, ys, vxs


def visualize_prediction(
    output_path: str = "prediction_visualization.png",
    show: bool = True,
) -> None:
    """
    Visualize the lead-car stop and oncoming-traffic wait decision.

    The scenario uses only the constant-acceleration predictor:
      - ego starts behind the lead vehicle
      - the lead vehicle brakes to zero speed
      - the oncoming vehicle moves straight in the opposite lane
      - the lower plot shows when the signed clearance becomes safe
    """
    import matplotlib.pyplot as plt
    import numpy as np

    prediction_horizon_ms = 6000
    dt_ms = 200
    safety_distance_m = 15.0

    ego_state_t0 = EgoStateStamped(
        timestamp=0,
        state=EgoState(
            pos=Vector2D(x=0.0, y=0.0),
            velocity=Vector2D(x=8.0, y=0.0),
            acceleration=Vector2D(x=0.0, y=0.0),
            yaw=0.0,
            steering_angle=0.0,
        ),
    )

    lead_vehicle = DynamicObjectStamped(
        timestamp=0,
        state=DynamicObject(
            id=1,
            obj_class=ObjectType.VEHICLE,
            pos=Vector2D(x=20.0, y=0.0),
            yaw=0.0,
            velocity=Vector2D(x=8.0, y=0.0),
            acceleration=Vector2D(x=-2.0, y=0.0),
            width=1.8,
            length=4.5,
        ),
    )

    oncoming_vehicle = DynamicObjectStamped(
        timestamp=0,
        state=DynamicObject(
            id=2,
            obj_class=ObjectType.VEHICLE,
            pos=Vector2D(x=80.0, y=3.5),
            yaw=np.pi,
            velocity=Vector2D(x=-12.0, y=0.0),
            acceleration=Vector2D(x=0.0, y=0.0),
            width=1.8,
            length=4.5,
        ),
    )

    environment = Environment(objects=[lead_vehicle, oncoming_vehicle], lanes=[])
    predicted_env = predict_environment(
        environment,
        prediction_horizon=prediction_horizon_ms,
        dt=dt_ms,
        model="constant_acceleration",
    )

    lead_obj = find_lead_vehicle(environment, ego_state_t0)
    oncoming_obj = find_oncoming_vehicle(environment, ego_state_t0)
    print(f"find_lead_vehicle found id: {lead_obj.state.id if lead_obj else None}")
    print(
        "find_oncoming_vehicle found id: "
        f"{oncoming_obj.state.id if oncoming_obj else None}"
    )

    lead_id = lead_obj.state.id if lead_obj else lead_vehicle.state.id
    oncoming_id = (
        oncoming_obj.state.id if oncoming_obj else oncoming_vehicle.state.id
    )

    ego_trajectory: List[EgoStateStamped] = []
    for timestamp in range(0, prediction_horizon_ms + dt_ms, dt_ms):
        time_s = timestamp / 1000.0
        ego_trajectory.append(
            EgoStateStamped(
                timestamp=timestamp,
                state=EgoState(
                    pos=Vector2D(
                        x=ego_state_t0.state.velocity.x * time_s,
                        y=0.0,
                    ),
                    velocity=ego_state_t0.state.velocity,
                    acceleration=Vector2D(x=0.0, y=0.0),
                    yaw=0.0,
                    steering_angle=0.0,
                ),
            )
        )

    gap_is_safe = is_overtake_gap_safe(
        predicted_environment=predicted_env,
        ego_trajectory=ego_trajectory,
        lead_vehicle_id=lead_id,
        overtake_start_ms=0,
        overtake_duration_ms=prediction_horizon_ms,
        safety_distance=safety_distance_m,
    )
    print(f"is_overtake_gap_safe over full horizon: {gap_is_safe}")

    lead_ts, lead_xs, lead_ys, lead_vxs = _flatten_predictions(
        predicted_env,
        lead_id,
    )
    oncoming_ts, oncoming_xs, oncoming_ys, _ = _flatten_predictions(
        predicted_env,
        oncoming_id,
    )
    ego_xs = np.array([state.state.pos.x for state in ego_trajectory])
    ego_ys = np.array([state.state.pos.y for state in ego_trajectory])

    stopped_mask = np.isclose(lead_vxs, 0.0, atol=1e-6)
    lead_stop_time_ms = int(lead_ts[stopped_mask][0]) if stopped_mask.any() else None

    signed_clearance = np.array(
        [
            get_signed_magnitude(
                Vector2D(
                    x=oncoming_x - ego_x,
                    y=oncoming_y - ego_y,
                ),
                oncoming_vehicle.state.yaw,
            )
            for oncoming_x, oncoming_y, ego_x, ego_y in zip(
                oncoming_xs,
                oncoming_ys,
                ego_xs,
                ego_ys,
            )
        ]
    )
    unsafe_mask = signed_clearance < safety_distance_m
    clear_indices = np.flatnonzero(~unsafe_mask)
    oncoming_clear_time_ms = (
        int(oncoming_ts[clear_indices[0]]) if len(clear_indices) else None
    )

    fig, (ax_top, ax_gap) = plt.subplots(
        2,
        1,
        figsize=(10, 10),
        height_ratios=[1.4, 1],
    )

    color_map = plt.cm.viridis
    norm = plt.Normalize(vmin=0, vmax=prediction_horizon_ms)

    ax_top.plot(
        ego_xs,
        ego_ys,
        color="black",
        linewidth=2,
        label="Ego path",
        zorder=3,
    )
    ax_top.scatter(ego_xs, ego_ys, c="black", s=20, zorder=4)

    ax_top.scatter(
        lead_xs,
        lead_ys,
        c=lead_ts,
        cmap=color_map,
        norm=norm,
        s=60,
        marker="s",
        edgecolors="black",
        linewidths=0.5,
        label="Lead vehicle braking to stop",
        zorder=3,
    )
    oncoming_scatter = ax_top.scatter(
        oncoming_xs,
        oncoming_ys,
        c=oncoming_ts,
        cmap=color_map,
        norm=norm,
        s=60,
        marker="^",
        edgecolors="black",
        linewidths=0.5,
        label="Oncoming vehicle constant velocity",
        zorder=3,
    )

    if lead_stop_time_ms is not None:
        stop_x = lead_xs[lead_ts == lead_stop_time_ms][0]
        stop_y = lead_ys[lead_ts == lead_stop_time_ms][0]
        ax_top.scatter(
            [stop_x],
            [stop_y],
            facecolors="none",
            edgecolors="red",
            s=200,
            linewidths=2,
            zorder=5,
            label=f"Lead stopped at {lead_stop_time_ms} ms",
        )

    ax_top.set_xlabel("x [m]")
    ax_top.set_ylabel("y [m]")
    ax_top.set_title("Predicted trajectories: color encodes prediction time")
    ax_top.legend(loc="upper left", fontsize=8)
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.grid(True, alpha=0.3)

    color_bar = fig.colorbar(oncoming_scatter, ax=ax_top, pad=0.01)
    color_bar.set_label("Prediction time [ms]")

    ax_gap.plot(
        oncoming_ts,
        signed_clearance,
        color="tab:red",
        linewidth=2,
        label="Signed ego-to-oncoming clearance",
    )
    ax_gap.axhline(
        safety_distance_m,
        color="black",
        linestyle="--",
        label=f"Safety distance ({safety_distance_m} m)",
    )
    ax_gap.fill_between(
        oncoming_ts,
        signed_clearance,
        safety_distance_m,
        where=unsafe_mask,
        color="red",
        alpha=0.25,
        label="Unsafe to overtake",
    )
    if lead_stop_time_ms is not None:
        ax_gap.axvline(
            lead_stop_time_ms,
            color="tab:blue",
            linestyle=":",
            label=f"Lead stops ({lead_stop_time_ms} ms)",
        )
    if oncoming_clear_time_ms is not None:
        ax_gap.axvline(
            oncoming_clear_time_ms,
            color="tab:green",
            linestyle=":",
            label=f"Oncoming clear ({oncoming_clear_time_ms} ms)",
        )

    ax_gap.set_xlabel("Time [ms]")
    ax_gap.set_ylabel("Signed clearance [m]")
    ax_gap.set_title(
        f"Overtake gap safety over time: is_overtake_gap_safe = {gap_is_safe}"
    )
    ax_gap.legend(loc="best", fontsize=8)
    ax_gap.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    if show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    visualize_prediction()
