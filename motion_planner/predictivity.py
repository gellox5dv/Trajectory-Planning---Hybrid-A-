import math
from dataclasses import dataclass, field
from typing import Literal, List, Optional, Tuple

from matplotlib.pyplot import step


PredictionModel = Literal["constant_velocity", "constant_acceleration"]

StaticObstacle = Tuple[float, float, float, float]

@dataclass
class DynamicObjectStamped:
    """State of one dynamic object at a specific time."""

    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float = 0.0
    timestamp: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    id: Optional[str] = None
    width: float = 0.0
    length: float = 0.0

    def speed(self) -> float:
        """Calculate the speed of the object."""
        return math.sqrt(self.vx**2 + self.vy**2)

    def to_ego_frame(self, ego_x: float, ego_y: float, ego_yaw: float):
        """""""""
        Returns (dx, dy) — position of this object relative to the ego car.
        The ego car always faces forward. To know if an object is
        'ahead' or 'to the left', we rotate the global offset into
        the car's local frame using the inverse rotation matrix.
        """""""""
        # Offset from ego to object in global frame
        gx = self.x - ego_x
        gy = self.y - ego_y

        # Rotate by -ego_yaw to get local coordinates
        cos_y = math.cos(-ego_yaw)
        sin_y = math.sin(-ego_yaw)
        dx = gx * cos_y - gy * sin_y
        dy = gx * sin_y + gy * cos_y
        return dx, dy

@dataclass
class Environment:
    """Current static and dynamic scene observed by the planner."""

    dynamic_objects: list[DynamicObjectStamped] = field(default_factory=list)
    static_obstacles: list[StaticObstacle] = field(default_factory=list)
    timestamp: float = 0.0


@dataclass
class PredictedEnvironment:
    """Dynamic object predictions indexed by planning time step."""

    predictions: list[list[DynamicObjectStamped]]
    dt: float
    horizon: int
    static_obstacles: list[StaticObstacle] = field(default_factory=list)

    def objects_at_step(self, step: int) -> list[DynamicObjectStamped]:
        """Return predicted objects at a planning step."""
        step = max(0, min(step, self.horizon))
        return self.predictions[step]
        if step < 0 or step > self.horizon:
            raise ValueError(f"Step must be between 0 and {self.horizon}")
        return self.predictions[step]
    
    def objects_at_time(self, t: float) -> List[DynamicObjectStamped]:
        """Look up by time in seconds instand of step index. Converting countinous to discrete."""
        step = round(t / self.dt)
        return self.objects_at_step(step)

#--- Helpers ---

def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _validate_prediction_inputs(horizon: int, dt: float) -> None:
    if horizon < 0:
        raise ValueError("horizon must be >= to 0")
    if dt <= 0.0:
        raise ValueError("dt must be greater than 0")

#--- Prediction Models ---

def predict_constant_velocity(
    obj: DynamicObjectStamped,
    horizon: int,
    dt: float,
) -> list[DynamicObjectStamped]:
    """
    Predict future object states assuming constant velocity.
    Formula:
        x(t) = x0 + vx * t
        y(t) = y0 + vy * t
        yaw(t) = yaw0 + yaw_rate * t
    """

    _validate_prediction_inputs(horizon, dt)

    prediction = []
    for step in range(horizon + 1):
        t = step * dt
        prediction.append(
            DynamicObjectStamped(
                x=obj.x + obj.vx * t,
                y=obj.y + obj.vy * t,
                yaw=normalize_angle(obj.yaw + obj.yaw_rate * t),
                vx=obj.vx,
                vy=obj.vy,
                yaw_rate=obj.yaw_rate,
                timestamp=obj.timestamp + t,
                ax=0.0,
                ay=0.0,
                id=obj.id,
                width=obj.width,
                length=obj.length,
            )
        )

    return prediction


def predict_constant_acceleration(
    obj: DynamicObjectStamped,
    horizon: int,
    dt: float,
) -> list[DynamicObjectStamped]:
    """
    Predict future object states assuming constant acceleration.
    Formula (kinematic equations of motion):
        x(t)  = x0 + vx*t + 0.5*ax*t²
        y(t)  = y0 + vy*t + 0.5*ay*t²
        vx(t) = vx0 + ax*t
        vy(t) = vy0 + ay*t
    """
    
    _validate_prediction_inputs(horizon, dt)

    prediction = []
    for step in range(horizon + 1):
        t = step * dt
        # This are the predicted velocity componenrs
        vx_pred = obj.vx + obj.ax * t
        vy_pred = obj.vy + obj.ay * t
        # Not allowing braking to start moving backwards
        speed_pred = math.sqrt(vx_pred**2 + vy_pred**2)
        if speed_pred < 0.0:
            vx_pred = 0.0
            vy_pred = 0.0

        prediction.append(
            DynamicObjectStamped(
                x=obj.x + obj.vx * t + 0.5 * obj.ax * t**2,
                y=obj.y + obj.vy * t + 0.5 * obj.ay * t**2,
                yaw=normalize_angle(obj.yaw + obj.yaw_rate * t),
                vx=obj.vx + obj.ax * t,
                vy=obj.vy + obj.ay * t,
                yaw_rate=obj.yaw_rate,
                timestamp=obj.timestamp + t,
                ax=obj.ax,
                ay=obj.ay,
                id=obj.id,
                width=obj.width,
                length=obj.length,
            )
        )

    return prediction

#--- Environment Prediction ---

def predict_environment(
    environment: Environment,
    horizon: int,
    dt: float,
    model: PredictionModel = "constant_velocity",
) -> PredictedEnvironment:
    """Generate a predicted environment over the planning horizon."""
    
    _validate_prediction_inputs(horizon, dt)

    predictor = {
        "constant_velocity": predict_constant_velocity,
        "constant_acceleration": predict_constant_acceleration,
    }[model]

    object_predictions = [
        predictor(obj, horizon, dt)
        for obj in environment.dynamic_objects
    ]

    predictions = [
        [obj_prediction[step] for obj_prediction in object_predictions]
        for step in range(horizon + 1)
    ]

    return PredictedEnvironment(
        predictions      = predictions,
        dt               = dt,
        horizon          = horizon,
        static_obstacles = environment.static_obstacles.copy(),
    )

#--- Slowing other vehicles that will be overtaken---

def slow_vehicle (
    x: float,
    y: float,
    speed: float = 5.0,
    yaw: float = 0.0,
    id: str = "slow_vehicle",
) -> DynamicObjectStamped:
    """Create a slow vehicle object for testing."""
    return DynamicObjectStamped(
    x = x,
    y =y ,
    yaw = yaw,
    vx = speed * math.cos(yaw),
    vy = speed * math.sin(yaw),
    width = 1.381,
    length = 2.338,
    id = id,
    )

#--- Collision Prediction ---

def check_collision(
    ego_x: float,
    ego_y: float,
    obj: DynamicObjectStamped,
    margin: float = 0.5,
) -> bool:
    """
    Simple axis-aligned bounding box (AABB) collision check.

    Returns True if the ego car is too close to the object.
    margin adds extra safety buffer around the object in meters.
    """
    half_width = obj.width / 2 + margin
    half_length = obj.length / 2 + margin
    dx = abs(ego_x - obj.x)
    dy = abs(ego_y - obj.y)
    return dx < half_length and dy < half_width

#--- Main ---

if __name__ == "__main__":
    # Example: slow car 30m ahead
    slow_car = slow_vehicle(x=30.0, y=0.0, speed=5.0)
    env = Environment(dynamic_objects=[slow_car], timestamp=0.0)

    # Predict 3s. 0.05s steps = 60 steps
    horizon = 60
    dt = 0.05
    predicted_env = predict_environment(env, horizon, dt, model="constant_velocity")

    # Print where the slow car will be at 1s, 2s, and 3s
    print("slow car predictions:")
    for t_check in [1.0, 2.0, 3.0]:
        objs_at_t = predicted_env.objects_at_time(t_check)
        o = objs_at_t[0]  # Only one object
        print(f" t={t_check:.1f}s: x={o.x:.2f}, y={o.y:.2f} "
              f"vx={o.vx:.2f}, speed={o.speed():.2f}m/s")
        
# Example of collision check at t=0
ego_x, ego_y = 0.0, 0.0
hit = check_collision(ego_x, ego_y, predicted_env.objects_at_time(0.0)[0])
print(f"Collision at t=0: {hit}")