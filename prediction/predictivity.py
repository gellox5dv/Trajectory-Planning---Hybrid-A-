import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

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


# Main role: predict one future position/velocity using constant acceleration and stop clamping.
def _predict_constant_acceleration_step(
    position: Vector2D,
    velocity: Vector2D,
    acceleration: Vector2D,
    time_s: float,
) -> tuple[Vector2D, Vector2D]:
    """
    Predict one step and clamp braking objects at the stop point.

    Constant acceleration equations:
        x(t)  = x0 + vx0 * t + 0.5 * ax * t^2
        y(t)  = y0 + vy0 * t + 0.5 * ay * t^2
        vx(t) = vx0 + ax * t
        vy(t) = vy0 + ay * t

    Stop-time equation for braking:
        t_stop = -dot(v, a) / dot(a, a)
    """
    velocity_dot_accel = velocity.x * acceleration.x + velocity.y * acceleration.y
    acceleration_norm_sq = acceleration.x**2 + acceleration.y**2

    if velocity_dot_accel < 0.0 and acceleration_norm_sq > 1e-9:
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


# Main role: generate future states for every object over the prediction horizon.
def predict_motion_constant_acceleration(
    objects: List[DynamicObjectStamped],  # current dynamic objects from Environment.objects
    prediction_horizon: int,              # total prediction time [ms]
    dt: int,                              # prediction step size [ms]
    last_only: bool = False,              # return only final state per object if True
) -> List[DynamicObjectStamped]:
    """Predict object motion over the horizon using constant acceleration."""
    _validate_prediction_inputs(prediction_horizon, dt)
    predicted_objects: List[DynamicObjectStamped] = []

    for obj in objects:
        cs = obj.state
        velocity = _vector_from_motion(cs.velocity, cs.yaw)          # initial velocity [m/s]
        acceleration = _vector_from_motion(cs.acceleration, cs.yaw)  # constant acceleration [m/s^2]
        predicted_states = []

        for t in range(0, prediction_horizon + dt, dt):
            time_s = t / 1000.0
            new_pos, new_velocity = _predict_constant_acceleration_step(
                position=cs.pos,
                velocity=velocity,
                acceleration=acceleration,
                time_s=time_s,
            )

            predicted_states.append(
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

        if last_only:
            predicted_objects.append(predicted_states[-1])
        else:
            predicted_objects.extend(predicted_states)

    return predicted_objects

# Main role: reorganize flat predictions into object_id -> list of predicted states.
def group_predictions_by_object(
    predicted_objects: List[DynamicObjectStamped],  # flat list of predictions for all objects
) -> Dict[int, List[DynamicObjectStamped]]:
    """Convert flat predictions into object_id -> predicted states."""
    grouped: Dict[int, List[DynamicObjectStamped]] = {}
    for obj in predicted_objects:
        grouped.setdefault(obj.state.id, []).append(obj)
    return grouped

# Main role: build a PredictedEnvironment from the current Environment.
def predict_environment(
    environment: Environment,                 # current scene with dynamic objects and lanes
    prediction_horizon: int,                  # total prediction time [ms]
    dt: int,                                  # prediction step size [ms]
    model: str = "constant_acceleration",     # kept for future model selection
) -> PredictedEnvironment:
    """Predict every object in the environment and return PredictedEnvironment."""
    if not environment.objects:
        raise ValueError("Empty object list — prediction would be vacuously safe.")
    raw = predict_motion_constant_acceleration(environment.objects, prediction_horizon, dt)
    return PredictedEnvironment(
        objects=group_predictions_by_object(raw),
        lanes=environment.lanes,
        dt=dt,
        horizon=prediction_horizon,
    )