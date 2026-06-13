from math import pi
<<<<<<< HEAD
from types import SimpleNamespace
from models.models import Vector2D, Lane, ObjectType, DynamicObjectStamped, DynamicObject, Environment


LANE_DIMS = SimpleNamespace(width = 4.0, length = 2000.0)
VEHICLE_DIMS = SimpleNamespace(width = 2.0, length = 5.0)


lane1 = Lane(
    id = 1,
    centerline = [(Vector2D(x = float(i), y = 2.0), 0.0) for i in range(int(LANE_DIMS.length))],
    width = LANE_DIMS.width,
    speed_limit = 16.0 + 2/3    # 60 km/h
)

lane2 = Lane(
    id = 2,
    centerline = [(Vector2D(x = float(i), y = 6.0), pi) for i in range(int(LANE_DIMS.length) - 1, -1, -1)],
    width = LANE_DIMS.width,
    speed_limit = 16.0 + 2/3    # 60 km/h
)

obs1 = DynamicObjectStamped(
    timestamp = 0,
    state = DynamicObject(
        id = 1,
        obj_class = ObjectType.VEHICLE,
        pos = Vector2D(x = 100.0, y = 2.0),
        yaw = 0.0,
        velocity = 8.0 + 1/3,       # 30 km/h
        acceleration = 0.0,
        width = VEHICLE_DIMS.width,
        length = VEHICLE_DIMS.length
    )
)

obs2 = DynamicObjectStamped(
    timestamp = 0,
    state = DynamicObject(
        id = 2,
        obj_class = ObjectType.VEHICLE,
        pos = Vector2D(x = 1800.0, y = 6.0),
        yaw = pi,
        velocity = 13.0 + 8/9,      # 50 km/h
        acceleration = 0.0,
        width = VEHICLE_DIMS.width,
        length = VEHICLE_DIMS.length
    )
)

environment = Environment(
    objects = [obs1, obs2],
    lanes = [lane1, lane2]
)
=======
from omegaconf import DictConfig
from utils.helper import get_vector
from models.models import Vector2D, Lane, ObjectType, DynamicObjectStamped, DynamicObject, Environment


def _kmh_to_mps(speed_kmh: float) -> float:
    return speed_kmh / 3.6

def _deg_to_rad(deg: float) -> float:
    return deg * pi / 180.0

def _build_lane(lane_cfg: DictConfig, lane_defaults: DictConfig) -> Lane:
    lane_id = int(lane_cfg.id)
    lane_length = float(lane_cfg.get("length", lane_defaults.length))
    lane_width = float(lane_cfg.get("width", lane_defaults.width))
    lane_y = float(lane_cfg.y)
    lane_yaw = _deg_to_rad(float(lane_cfg.yaw))

    speed_limit_kmh = float(lane_cfg.get("speed_limit_kmh", lane_defaults.speed_limit_kmh))
    speed_limit_mps = _kmh_to_mps(speed_limit_kmh)

    if abs(lane_yaw) < 1e-9:
        centerline = [(Vector2D(x=float(i), y=lane_y), lane_yaw) for i in range(int(lane_length))]
    else:
        centerline = [(Vector2D(x=float(i), y=lane_y), lane_yaw) for i in range(int(lane_length) - 1, -1, -1)]

    return Lane(
        id=lane_id,
        centerline=centerline,
        width=lane_width,
        speed_limit=speed_limit_mps,
    )


def _build_object(obj_cfg: DictConfig, object_defaults: DictConfig) -> DynamicObjectStamped:
    obj_type = ObjectType[obj_cfg.get("type", object_defaults.type)]
    speed_kmh = float(obj_cfg.speed_kmh)

    return DynamicObjectStamped(
        timestamp=int(obj_cfg.get("timestamp", object_defaults.timestamp)),
        state=DynamicObject(
            id=int(obj_cfg.id),
            obj_class=obj_type,
            pos=Vector2D(x=float(obj_cfg.pos.x), y=float(obj_cfg.pos.y)),
            yaw=_deg_to_rad(float(obj_cfg.yaw)),
            velocity=get_vector(_kmh_to_mps(speed_kmh), _deg_to_rad(float(obj_cfg.yaw))),
            acceleration=Vector2D(
                x=float(obj_cfg.get("ax", object_defaults.ax)),
                y=float(obj_cfg.get("ay", object_defaults.ay)),
            ),
            width=float(obj_cfg.get("width", object_defaults.width)),
            length=float(obj_cfg.get("length", object_defaults.length)),
        ),
    )


def create_environment(scenario_cfg: DictConfig) -> Environment:
    lane_defaults = scenario_cfg.lane_defaults
    object_defaults = scenario_cfg.object_defaults

    lanes = [_build_lane(lane_cfg, lane_defaults) for lane_cfg in scenario_cfg.lanes.values()]
    objects = [_build_object(obj_cfg, object_defaults) for obj_cfg in scenario_cfg.objects.values()]

    return Environment(objects=objects, lanes=lanes)
>>>>>>> feature/bicycle-only
