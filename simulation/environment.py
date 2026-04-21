from math import pi
from types import SimpleNamespace
from models.models import Vector2D, Lane, ObjectType, DynamicObjectStamped, DynamicObject, Environment


LANE_DIMS = SimpleNamespace(width = 4.0, length = 8000.0)
VEHICLE_DIMS = SimpleNamespace(width = 2.0, length = 5.0)


lane1 = Lane(
    id = 1,
    centerline = [Vector2D(x = float(i), y = 2.0) for i in range(int(LANE_DIMS.length))],
    width = LANE_DIMS.width,
    speed_limit = 16.0 + 2/3    # 60 km/h
)

lane2 = Lane(
    id = 2,
    centerline = [Vector2D(x = float(i), y = 6.0) for i in range(int(LANE_DIMS.length) - 1, -1, -1)],
    width = LANE_DIMS.width,
    speed_limit = 16.0 + 2/3    # 60 km/h
)

obs1 = DynamicObject(
    id = 1,
    obj_class = ObjectType.VEHICLE,
    pos = Vector2D(x = 1200.0, y = 2.0),
    yaw = 0.0,
    velocity = 8.0 + 1/3,       # 30 km/h
    acceleration = 0.0,
    width = VEHICLE_DIMS.width,
    length = VEHICLE_DIMS.length
)

obs2 = DynamicObject(
    id = 2,
    obj_class = ObjectType.VEHICLE,
    pos = Vector2D(x = 4000.0, y = 6.0),
    yaw = pi,
    velocity = 13.0 + 8/9,      # 50 km/h
    acceleration = 0.0,
    width = VEHICLE_DIMS.width,
    length = VEHICLE_DIMS.length
)

environment = Environment(
    objects = [obs1, obs2],
    lanes = [lane1, lane2]
)