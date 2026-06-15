import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from models.models import DynamicState, EgoInput, EgoState, EgoStateStamped, Vector2D
from omegaconf import DictConfig
from utils.helper import get_magnitude, get_vector, global_to_ego_axis


_MIN_SPEED_FOR_SLIP = 0.1


# Role: fast stateless model for planner trajectory rollout.
def kinematic_bicycle_model(
    state: EgoStateStamped,
    control: EgoInput,
    dt_ms: int,
    veh_cfg: DictConfig,
    limit_steer_rate: bool = False,
) -> EgoStateStamped:
    """Fast stateless bicycle model intended for planner rollout."""

    current_state = state.state
    dt_sec = float(dt_ms) / 1000.0

    max_acceleration = float(veh_cfg.max_acceleration)
    max_deceleration = float(veh_cfg.max_deceleration)
    if max_deceleration > 0.0:
        max_deceleration = -max_deceleration
    acceleration = max(
        max_deceleration,
        min(max_acceleration, float(control.acceleration)),
    )

    steering_angle = max(
        -float(veh_cfg.max_steer),
        min(float(veh_cfg.max_steer), float(control.steering_angle)),
    )

    if limit_steer_rate:
        max_steering_delta = float(veh_cfg.max_steer_rate) * dt_sec
        steering_angle = max(
            current_state.steering_angle - max_steering_delta,
            min(
                current_state.steering_angle + max_steering_delta,
                steering_angle,
            ),
        )

    speed = get_magnitude(current_state.velocity)
    next_speed = max(0.0, speed + acceleration * dt_sec)
    average_speed = 0.5 * (speed + next_speed)

    yaw = current_state.yaw
    yaw_rate = average_speed * math.tan(steering_angle) / veh_cfg.wheel_base

    if abs(yaw_rate) > 1e-9:
        next_yaw = (yaw + yaw_rate * dt_sec + math.pi) % (2.0 * math.pi) - math.pi
        turn_radius = average_speed / yaw_rate
        next_x = current_state.pos.x + turn_radius * (
            math.sin(next_yaw) - math.sin(yaw)
        )
        next_y = current_state.pos.y - turn_radius * (
            math.cos(next_yaw) - math.cos(yaw)
        )
    else:
        next_yaw = yaw
        next_x = current_state.pos.x + average_speed * math.cos(yaw) * dt_sec
        next_y = current_state.pos.y + average_speed * math.sin(yaw) * dt_sec

    next_velocity = get_vector(next_speed, next_yaw)

    lateral_acceleration = next_speed * yaw_rate
    acceleration_x, acceleration_y, _ = global_to_ego_axis(
        acceleration,
        lateral_acceleration,
        0.0,
        0.0,
        -next_yaw,
    )
    next_acceleration = Vector2D(x=acceleration_x, y=acceleration_y)

    return EgoStateStamped(
        timestamp=state.timestamp + dt_ms,
        state=EgoState(
            pos=Vector2D(x=next_x, y=next_y),
            velocity=next_velocity,
            acceleration=next_acceleration,
            yaw=next_yaw,
            steering_angle=steering_angle,
        ),
    )


class DynamicBicycleModel:
    """Stateful dynamic bicycle model intended for simulation."""

    # Role: initialize simulator state from an EgoState and vehicle config.
    def __init__(
        self,
        ego_state: EgoState,
        veh_cfg: DictConfig,
        yaw_rate: float = 0.0,
    ) -> None:
        self.veh_cfg = veh_cfg
        vx_body, vy_body, _ = global_to_ego_axis(
            ego_state.velocity.x,
            ego_state.velocity.y,
            0.0,
            0.0,
            ego_state.yaw,
        )
        #Forward velocity is forced nonnegative, so this model does not drive backward.
        self.dynamic_state = DynamicState(
            pos=Vector2D(x=ego_state.pos.x, y=ego_state.pos.y),
            yaw=ego_state.yaw,
            velocity=Vector2D(x=max(0.0, vx_body), y=vy_body),
            yaw_rate=yaw_rate,
        )
        self.ego_state = EgoState(
            pos=Vector2D(x=ego_state.pos.x, y=ego_state.pos.y),
            velocity=ego_state.velocity,
            acceleration=ego_state.acceleration,
            yaw=ego_state.yaw,
            steering_angle=ego_state.steering_angle,
        )

    # Role: replace the internal simulator state with a new EgoState.
    #new state without creating a new model.
    def reset(self, ego_state: EgoState, yaw_rate: float = 0.0) -> None:
        vx_body, vy_body, _ = global_to_ego_axis(
            ego_state.velocity.x,
            ego_state.velocity.y,
            0.0,
            0.0,
            ego_state.yaw,
        )

        self.dynamic_state = DynamicState(
            pos=Vector2D(x=ego_state.pos.x, y=ego_state.pos.y),
            yaw=ego_state.yaw,
            velocity=Vector2D(x=max(0.0, vx_body), y=vy_body),
            yaw_rate=yaw_rate,
        )
        self.ego_state = EgoState(
            pos=Vector2D(x=ego_state.pos.x, y=ego_state.pos.y),
            velocity=ego_state.velocity,
            acceleration=ego_state.acceleration,
            yaw=ego_state.yaw,
            steering_angle=ego_state.steering_angle,
        )

    # Role: convert target steering input into steer rate, then use nonlinear dynamics.
    def step_control(self, control: EgoInput, dt_ms: int) -> EgoState:
        """This accepts target steering angle and acceleration."""

        dt_sec = float(dt_ms) / 1000.0
        target_steering_angle = max(
            -float(self.veh_cfg.max_steer),
            min(float(self.veh_cfg.max_steer), float(control.steering_angle)),
        )
        steering_error = target_steering_angle - self.ego_state.steering_angle
        steer_rate = steering_error / dt_sec if dt_sec > 0.0 else 0.0

        return self.step(
            acceleration=control.acceleration,
            steer_rate=steer_rate,
            dt_ms=dt_ms,
        )

    # Role: convert target steering input into steer rate, then use linear dynamics.
    def step_linear_control(self, control: EgoInput, dt_ms: int) -> EgoState:
        """Step with the linear bicycle formula and a target steering command."""

        dt_sec = float(dt_ms) / 1000.0
        target_steering_angle = max(
            -float(self.veh_cfg.max_steer),
            min(float(self.veh_cfg.max_steer), float(control.steering_angle)),
        )
        steering_error = target_steering_angle - self.ego_state.steering_angle
        steer_rate = steering_error / dt_sec if dt_sec > 0.0 else 0.0

        return self.step_linear(
            acceleration=control.acceleration,
            steer_rate=steer_rate,
            dt_ms=dt_ms,
        )

    # Role: propagate simulator state with the bicycle formula.
    def step_linear(
        self,
        acceleration: float,
        steer_rate: float,
        dt_ms: int,
    ) -> EgoState:
        """Advance the vehicle with the linear dynamic bicycle formula."""

        dt_sec = float(dt_ms) / 1000.0
        if dt_sec <= 0.0:
            return self.to_ego_state(Vector2D(x=0.0, y=0.0))

        max_acceleration = float(self.veh_cfg.max_acceleration)
        max_deceleration = float(self.veh_cfg.max_deceleration)
        if max_deceleration > 0.0:
            max_deceleration = -max_deceleration
        acceleration = max(
            max_deceleration,
            min(max_acceleration, float(acceleration)),
        )
        steer_rate = max(
            -float(self.veh_cfg.max_steer_rate),
            min(float(self.veh_cfg.max_steer_rate), float(steer_rate)),
        )

        previous = self.dynamic_state
        steering_angle = max(
            -float(self.veh_cfg.max_steer),
            min(
                float(self.veh_cfg.max_steer),
                self.ego_state.steering_angle + steer_rate * dt_sec,
            ),
        )

        speed = max(
            0.0,
            get_magnitude(previous.velocity) + acceleration * dt_sec,
        )

        if speed > 1e-3:
            beta = math.atan2(previous.velocity.y, previous.velocity.x)
            yaw_rate = previous.yaw_rate

            cf = float(self.veh_cfg.Cf)
            cr = float(self.veh_cfg.Cr)
            lf = float(self.veh_cfg.lf)
            lr = float(self.veh_cfg.lr)
            mass = float(self.veh_cfg.m)
            inertia_z = float(self.veh_cfg.Iz)

            a11 = -(cf + cr) / (mass * speed)
            a12 = (-lf * cf + lr * cr) / (mass * speed**2) - 1.0
            a21 = (-lf * cf + lr * cr) / inertia_z
            a22 = -(lf**2 * cf + lr**2 * cr) / (inertia_z * speed)
            b1 = cf / (mass * speed)
            b2 = lf * cf / inertia_z

            beta_dot = a11 * beta + a12 * yaw_rate + b1 * steering_angle
            yaw_acceleration = a21 * beta + a22 * yaw_rate + b2 * steering_angle

            next_beta = beta + beta_dot * dt_sec
            next_yaw_rate = yaw_rate + yaw_acceleration * dt_sec
        else:
            next_beta = 0.0
            next_yaw_rate = 0.0

        next_yaw = (
            previous.yaw + next_yaw_rate * dt_sec + math.pi
        ) % (2.0 * math.pi) - math.pi

        next_vx = speed * math.cos(next_beta)
        next_vy = speed * math.sin(next_beta)

        next_velocity_x, next_velocity_y, _ = global_to_ego_axis(
            next_vx,
            next_vy,
            0.0,
            0.0,
            -next_yaw,
        )
        next_velocity_global = Vector2D(x=next_velocity_x, y=next_velocity_y)

        next_x = previous.pos.x + next_velocity_global.x * dt_sec
        next_y = previous.pos.y + next_velocity_global.y * dt_sec

        acceleration_x, acceleration_y, _ = global_to_ego_axis(
            acceleration,
            speed * next_yaw_rate,
            0.0,
            0.0,
            -next_yaw,
        )
        acceleration_global = Vector2D(x=acceleration_x, y=acceleration_y)

        self.dynamic_state = DynamicState(
            pos=Vector2D(x=next_x, y=next_y),
            yaw=next_yaw,
            velocity=Vector2D(x=next_vx, y=next_vy),
            yaw_rate=next_yaw_rate,
        )
        self.ego_state = EgoState(
            pos=Vector2D(x=next_x, y=next_y),
            velocity=next_velocity_global,
            acceleration=acceleration_global,
            yaw=next_yaw,
            steering_angle=steering_angle,
        )

        return self.ego_state

    # Role: propagate simulator state with the nonlinear tire-force bicycle model.
    def step(self, acceleration: float, steer_rate: float, dt_ms: int) -> EgoState:
        """Advance the internal vehicle state by dt_ms."""

        dt_sec = float(dt_ms) / 1000.0
        if dt_sec <= 0.0:
            return self.to_ego_state(Vector2D(x=0.0, y=0.0))

        max_acceleration = float(self.veh_cfg.max_acceleration)
        max_deceleration = float(self.veh_cfg.max_deceleration)
        if max_deceleration > 0.0:
            max_deceleration = -max_deceleration
        acceleration = max(
            max_deceleration,
            min(max_acceleration, float(acceleration)),
        )
        steer_rate = max(
            -float(self.veh_cfg.max_steer_rate),
            min(float(self.veh_cfg.max_steer_rate), float(steer_rate)),
        )

        previous = self.dynamic_state
        steering_angle = max(
            -float(self.veh_cfg.max_steer),
            min(
                float(self.veh_cfg.max_steer),
                self.ego_state.steering_angle + steer_rate * dt_sec,
            ),
        )

        vx = previous.velocity.x
        vy = previous.velocity.y
        yaw_rate = previous.yaw_rate
        slip_vx = max(abs(vx), _MIN_SPEED_FOR_SLIP)

        alpha_front = steering_angle - math.atan2(
            vy + float(self.veh_cfg.lf) * yaw_rate,
            slip_vx,
        )
        alpha_rear = -math.atan2(
            vy - float(self.veh_cfg.lr) * yaw_rate,
            slip_vx,
        )

        front_lateral_force = float(self.veh_cfg.Cf) * alpha_front
        rear_lateral_force = float(self.veh_cfg.Cr) * alpha_rear

        #Normal force means vertical load from gravity. Heavier load allows more friction force.
        wheel_base = float(self.veh_cfg.lf) + float(self.veh_cfg.lr)
        front_normal_force = (
            float(self.veh_cfg.m) * 9.81 * float(self.veh_cfg.lr) / wheel_base
        )
        rear_normal_force = (
            float(self.veh_cfg.m) * 9.81 * float(self.veh_cfg.lf) / wheel_base
        )

        force_scale = min(1.0, math.hypot(vx, vy) / _MIN_SPEED_FOR_SLIP)
        friction = float(self.veh_cfg.mu)
        front_lateral_force = force_scale * max(
            -friction * front_normal_force,
            min(friction * front_normal_force, front_lateral_force),
        )
        rear_lateral_force = force_scale * max(
            -friction * rear_normal_force,
            min(friction * rear_normal_force, rear_lateral_force),
        )

        dvx = (
            acceleration
            + yaw_rate * vy
            - front_lateral_force * math.sin(steering_angle) / float(self.veh_cfg.m)
        )
        dvy = (
            -yaw_rate * vx
            + (
                front_lateral_force * math.cos(steering_angle)
                + rear_lateral_force
            )
            / float(self.veh_cfg.m)
        )
        yaw_acceleration = (
            float(self.veh_cfg.lf) * front_lateral_force * math.cos(steering_angle)
            - float(self.veh_cfg.lr) * rear_lateral_force
        ) / float(self.veh_cfg.Iz)

        next_vx = max(0.0, vx + dvx * dt_sec)
        next_vy = vy + dvy * dt_sec
        next_yaw_rate = yaw_rate + yaw_acceleration * dt_sec
        next_yaw = (
            previous.yaw + next_yaw_rate * dt_sec + math.pi
        ) % (2.0 * math.pi) - math.pi

        next_velocity_x, next_velocity_y, _ = global_to_ego_axis(
            next_vx,
            next_vy,
            0.0,
            0.0,
            -next_yaw,
        )
        next_velocity_global = Vector2D(x=next_velocity_x, y=next_velocity_y)

        next_x = previous.pos.x + next_velocity_global.x * dt_sec
        next_y = previous.pos.y + next_velocity_global.y * dt_sec

        self.dynamic_state = DynamicState(
            pos=Vector2D(x=next_x, y=next_y),
            yaw=next_yaw,
            velocity=Vector2D(x=next_vx, y=next_vy),
            yaw_rate=next_yaw_rate,
        )

        acceleration_x, acceleration_y, _ = global_to_ego_axis(
            dvx - yaw_rate * vy,
            dvy + yaw_rate * vx,
            0.0,
            0.0,
            -next_yaw,
        )
        acceleration_global = Vector2D(x=acceleration_x, y=acceleration_y)

        self.ego_state = EgoState(
            pos=Vector2D(x=next_x, y=next_y),
            velocity=next_velocity_global,
            acceleration=acceleration_global,
            yaw=next_yaw,
            steering_angle=steering_angle,
        )

        return self.ego_state

    # Role: convert the internal DynamicState back into the public EgoState format.
    def to_ego_state(self, acceleration: Vector2D | None = None) -> EgoState:
        if acceleration is None:
            acceleration = Vector2D(x=0.0, y=0.0)

        velocity_x, velocity_y, _ = global_to_ego_axis(
            self.dynamic_state.velocity.x,
            self.dynamic_state.velocity.y,
            0.0,
            0.0,
            -self.dynamic_state.yaw,
        )

        return EgoState(
            pos=Vector2D(x=self.dynamic_state.pos.x, y=self.dynamic_state.pos.y),
            velocity=Vector2D(x=velocity_x, y=velocity_y),
            acceleration=acceleration,
            yaw=self.dynamic_state.yaw,
            steering_angle=self.ego_state.steering_angle,
        )
