from math import cos, pi, sin
from typing import List, Optional, Tuple
from models.models import Vector2D


def global_to_ego_axis(poi_x: float, poi_y: float, ego_x: float, ego_y: float, ego_yaw: float, poi_yaw: Optional[float] = None) -> Tuple[float, float, Optional[float]]:
    """Convert a point of interest (POI) from global coordinates to the ego vehicle's local coordinate frame."""

    dx = poi_x - ego_x
    dy = poi_y - ego_y

    x_ego =  dx * cos(ego_yaw) + dy * sin(ego_yaw)
    y_ego = -dx * sin(ego_yaw) + dy * cos(ego_yaw)

    if poi_yaw is not None:
        yaw_ego = poi_yaw - ego_yaw
        yaw_ego = (yaw_ego + pi) % (2 * pi) - pi
        return x_ego, y_ego, yaw_ego

    return x_ego, y_ego, None


def get_bbox_corners(x: float, y: float, yaw: float, length: float, width: float) -> List[Vector2D]:
    """Compute the corners of a bounding box given its center position, orientation, length, and width."""

    return [
        Vector2D(
            x = x + (length / 2) * cos(yaw) - (width / 2) * sin(yaw),
            y = y + (length / 2) * sin(yaw) + (width / 2) * cos(yaw)
        ),
        Vector2D(
            x = x + (length / 2) * cos(yaw) + (width / 2) * sin(yaw),
            y = y + (length / 2) * sin(yaw) - (width / 2) * cos(yaw)
        ),
        Vector2D(
            x = x - (length / 2) * cos(yaw) + (width / 2) * sin(yaw),
            y = y - (length / 2) * sin(yaw) - (width / 2) * cos(yaw)
        ),
        Vector2D(
            x = x - (length / 2) * cos(yaw) - (width / 2) * sin(yaw),
            y = y - (length / 2) * sin(yaw) + (width / 2) * cos(yaw)
        )
    ]


def check_line_intersection(p1: Vector2D, p2: Vector2D, p3: Vector2D, p4: Vector2D) -> bool:
    """Check if the line segments p1p2 and p3p4 intersect."""

    d1 = Vector2D(x = p2.x - p1.x, y = p2.y - p1.y)
    d2 = Vector2D(x = p4.x - p3.x, y = p4.y - p3.y)

    det = d1.x * d2.y - d1.y * d2.x

    if det == 0:
        return False

    t = ((p3.x - p1.x) * d2.y - (p3.y - p1.y) * d2.x) / det
    u = ((p3.x - p1.x) * d1.y - (p3.y - p1.y) * d1.x) / det

    return (0 <= t <= 1) and (0 <= u <= 1)