import math
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import transforms

# module-level state: reuse the same figure and axes across repeated calls
_fig = None
_ax = None
_initialized = False


def visualize_scene(env, ego, vehicle_params, trajectory=None) -> None:
    """
    Main entry point. Call once per simulation step for live visualization.
    Clears and redraws the full scene on every call.
    """
    global _fig, _ax, _initialized

    # create the figure only once; subsequent calls reuse it
    if not _initialized:
        plt.ion()  # non-blocking interactive mode
        _fig, _ax = plt.subplots(figsize=(10, 8))
        _initialized = True

    _ax.clear()  # wipe the previous frame before redrawing

    # draw scene layers in order: road → objects → ego → trajectory
    _draw_lanes(_ax, env)
    _draw_objects(_ax, env)
    _draw_ego(_ax, ego, vehicle_params)
    _draw_trajectory(_ax, trajectory)

    # axis formatting
    _ax.set_title("Trajectory Visualization")
    _ax.set_xlabel("x [m]")
    _ax.set_ylabel("y [m]")
    _ax.set_aspect("equal", adjustable="box")  # equal scale on both axes
    _ax.grid(True)
    _ax.legend(loc="upper left", fontsize=8)

    # center the view around the ego vehicle
    _set_view(_ax, ego)

    plt.draw()
    plt.pause(0.001)  # tiny pause so the GUI can refresh without blocking


# ─── State Unwrappers ────────────────────────────────────────────────────────

def _get_ego_state(ego_entry):
    """Accept either EgoStateStamped or plain EgoState."""
    return ego_entry.state if hasattr(ego_entry, "state") else ego_entry


def _get_object_state(obj_entry):
    """Accept either DynamicObjectStamped or plain DynamicObject."""
    return obj_entry.state if hasattr(obj_entry, "state") else obj_entry


def _get_traj_state(state_entry):
    """Accept either EgoStateStamped or plain EgoState."""
    return state_entry.state if hasattr(state_entry, "state") else state_entry


# ─── Drawing Functions ───────────────────────────────────────────────────────

def _draw_lanes(ax, env) -> None:
    """
    Draw lane centerlines (dashed) and boundaries (solid white).
    Lane.centerline is List[Tuple[Vector2D, float]]: each entry is (point, tangent_angle).
    Lane boundaries are offset ± width/2 perpendicular to the tangent.
    """
    if env is None or not hasattr(env, "lanes") or env.lanes is None:
        return

    for lane in env.lanes:
        if not hasattr(lane, "centerline") or not lane.centerline:
            continue

        # unpack (Vector2D, tangent_angle) tuples
        pts      = [p[0] for p in lane.centerline]
        tangents = [p[1] for p in lane.centerline]

        xs = [p.x for p in pts]
        ys = [p.y for p in pts]

        # dashed centerline
        ax.plot(xs, ys, linestyle="--", linewidth=1.0, color="gray")

        # lane boundaries: offset perpendicular to tangent by ± half width
        half_w = lane.width / 2
        left_xs  = [p.x - half_w * math.sin(t) for p, t in zip(pts, tangents)]
        left_ys  = [p.y + half_w * math.cos(t) for p, t in zip(pts, tangents)]
        right_xs = [p.x + half_w * math.sin(t) for p, t in zip(pts, tangents)]
        right_ys = [p.y - half_w * math.cos(t) for p, t in zip(pts, tangents)]

        ax.plot(left_xs,  left_ys,  linewidth=1.0, color="white")
        ax.plot(right_xs, right_ys, linewidth=1.0, color="white")


def _draw_objects(ax, env) -> None:
    """Draw all dynamic objects as oriented bounding boxes (orange)."""
    if env is None or not hasattr(env, "objects") or env.objects is None:
        return

    for obj_entry in env.objects:
        obj = _get_object_state(obj_entry)  # unwrap stamped if needed

        _draw_box(
            ax=ax,
            x=obj.pos.x,
            y=obj.pos.y,
            yaw=obj.yaw,
            length=obj.length,
            width=obj.width,
            color="orange",
            label=f"Obj {obj.id}",
        )


def _draw_ego(ax, ego, vehicle_params) -> None:
    """
    Draw the ego vehicle assuming ego_state.pos is the rear axle center.
    """
    if ego is None or vehicle_params is None:
        return

    ego_state = _get_ego_state(ego)

    # Shift from rear axle center to geometric center of the vehicle body
    center_offset = vehicle_params.length / 2 - vehicle_params.rear_to_wheel
    x_center = ego_state.pos.x + center_offset * math.cos(ego_state.yaw)
    y_center = ego_state.pos.y + center_offset * math.sin(ego_state.yaw)

    _draw_box(
        ax=ax,
        x=x_center,
        y=y_center,
        yaw=ego_state.yaw,
        length=vehicle_params.length,
        width=vehicle_params.width,
        color="blue",
        label="Ego",
    )

    # Draw rear axle reference point
    ax.scatter(ego_state.pos.x, ego_state.pos.y, color="cyan", s=25, zorder=6)

    # Heading arrow starts at rear axle center
    arrow_len = max(vehicle_params.length * 0.6, 0.5)
    ax.arrow(
        ego_state.pos.x,
        ego_state.pos.y,
        arrow_len * math.cos(ego_state.yaw),
        arrow_len * math.sin(ego_state.yaw),
        head_width=max(vehicle_params.width * 0.2, 0.15),
        length_includes_head=True,
        color="blue",
    )


def _draw_trajectory(ax, trajectory) -> None:
    """
    Draw the planned trajectory as a green line with point markers.
    Safe to call with trajectory=None — draws nothing.
    """
    if trajectory is None or not hasattr(trajectory, "states") or not trajectory.states:
        return

    # unwrap each EgoStateStamped to EgoState
    states = [_get_traj_state(s) for s in trajectory.states]
    xs = [s.pos.x for s in states]
    ys = [s.pos.y for s in states]

    # guard: need at least 2 points for a line
    if len(xs) < 2:
        ax.scatter(xs, ys, s=30, color="green", zorder=5)
        return

    ax.plot(xs, ys, linewidth=2.0, color="green", label="Planned trajectory")
    ax.scatter(xs, ys, s=10, color="green", zorder=5)  # small dots at each waypoint


def _draw_box(ax, x: float, y: float, yaw: float, length: float, width: float,
              color: str = "red", label: Optional[str] = None) -> None:
    """
    Draw a yaw-rotated bounding box centered at (x, y).
    Uses an Affine2D transform to rotate the Rectangle around its center.
    """
    # guard against degenerate vehicle sizes
    if length <= 0 or width <= 0:
        return

    # Rectangle anchor is bottom-left corner before rotation
    rect = Rectangle(
        (x - length / 2, y - width / 2),
        length,
        width,
        fill=False,       # transparent fill
        edgecolor=color,
        linewidth=2,
    )

    # rotate the rectangle around (x, y) by yaw angle
    t = transforms.Affine2D().rotate_around(x, y, yaw) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

    if label:
        ax.text(
            x, y + width / 2 + 0.3,   # offset above the box
            label,
            fontsize=8,
            color=color,
            ha="center",
            va="bottom",
            bbox=dict(
                boxstyle="round,pad=0.2",
                facecolor="white",
                edgecolor="none",
                alpha=0.7,             # semi-transparent background
            )
    )


# ─── Camera View ─────────────────────────────────────────────────────────────

def _set_view(ax, ego) -> None:
    """
    Center the view around the ego vehicle.
    Uses a wide horizontal margin (60 m) and narrow vertical margin (15 m)
    to match a typical road-driving perspective.
    Falls back to a ±10 m default view if ego is None.
    """
    if ego is None:
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        return

    ego_state = _get_ego_state(ego)

    x = ego_state.pos.x
    y = ego_state.pos.y

    ax.set_xlim(x - 60.0, x + 60.0)
    ax.set_ylim(y - 15.0, y + 15.0)