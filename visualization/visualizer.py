import math
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import transforms
from models.models import *

# Extended module-level state
_fig = None
_ax = None
_initialized = False

# New variables for camera control
_follow_ego = True
_saved_xlim = None
_saved_ylim = None

for key in ['f', 'F']:
    if key in plt.rcParams['keymap.fullscreen']:
        plt.rcParams['keymap.fullscreen'].remove(key)


def visualize_scene(env, ego, vehicle_params, trajectory=None, goal_region=None) -> None:
    """
    Main entry point. Call once per simulation step for live visualization.
    """
    global _fig, _ax, _initialized, _follow_ego, _saved_xlim, _saved_ylim

    if not _initialized:
        plt.ion()
        _fig, _ax = plt.subplots(figsize=(10, 8))
        
        def on_key(event):
            global _follow_ego
            if event.key == 'f':
                _follow_ego = not _follow_ego
                mode = "FOLLOW" if _follow_ego else "FREE CAMERA"
                print(f"🎥 Camera Mode: {mode}")

        _fig.canvas.mpl_connect('key_press_event', on_key)
        
        _initialized = True

    if not _follow_ego:
        _saved_xlim = _ax.get_xlim()
        _saved_ylim = _ax.get_ylim()

    _ax.clear()  # wipe the previous frame before redrawing
    
    _ax.set_facecolor("#444444")

    # draw scene layers
    _draw_lanes(_ax, env)
    _draw_goal_region(_ax, goal_region)
    _draw_objects(_ax, env)
    _draw_ego(_ax, ego, vehicle_params)
    _draw_trajectory(_ax, trajectory)

    # axis formatting
    # Show the current mode in the title so the user knows what is happening
    title_suffix = "(Follow Mode)" if _follow_ego else "(Free Camera - Press 'F' to follow)"
    _ax.set_title(f"Trajectory Visualization {title_suffix}")
    _ax.set_xlabel("x [m]")
    _ax.set_ylabel("y [m]")
    _ax.set_aspect("equal", adjustable="box")
    _ax.grid(True, color="#555555", linestyle=":")
    _ax.legend(loc="upper left", fontsize=8, facecolor="white", edgecolor="none")

    # Conditional camera tracking ---
    if _follow_ego:
        _set_view(_ax, ego)
    elif _saved_xlim is not None and _saved_ylim is not None:
        # In Free mode, restore the saved viewport (or your manual zoom)
        _ax.set_xlim(_saved_xlim)
        _ax.set_ylim(_saved_ylim)

    plt.draw()
    plt.pause(0.001)


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

        # dashed centerline (yellow/lightgray)
        #ax.plot(xs, ys, linestyle="--", linewidth=1.5, color="#CCCCCC", label="Centerline" if lane.id == env.lanes[0].id else "")

        # lane boundaries: offset perpendicular to tangent by ± half width
        half_w = lane.width / 2
        left_xs  = [p.x - half_w * math.sin(t) for p, t in zip(pts, tangents)]
        left_ys  = [p.y + half_w * math.cos(t) for p, t in zip(pts, tangents)]
        right_xs = [p.x + half_w * math.sin(t) for p, t in zip(pts, tangents)]
        right_ys = [p.y - half_w * math.cos(t) for p, t in zip(pts, tangents)]

        # Draw solid white road boundaries
        ax.plot(left_xs,  left_ys,  linewidth=2.0, color="white", label="Lane Boundary" if lane.id == env.lanes[0].id else "")
        ax.plot(right_xs, right_ys, linewidth=2.0, color="white")


def _draw_goal_region(ax, goal_region) -> None:
    """Draw the target goal region as a green transparent box."""
    if goal_region is None:
        return

    _draw_box(
        ax=ax,
        x=goal_region.center.x,
        y=goal_region.center.y,
        yaw=goal_region.yaw,
        length=goal_region.length,
        width=goal_region.width,
        color="lime",
        label="Goal Region",
        linestyle="--",
        alpha=0.6,
        fill=True
    )


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
        color="dodgerblue",
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
        color="cyan",
    )


def _draw_trajectory(ax, trajectory) -> None:
    """
    Draw the planned trajectory as a red line with dark red cross markers.
    Safe to call with trajectory=None — draws nothing.
    """
    # Robust guard clause: check for None, lack of 'states' attribute, or empty states
    if trajectory is None:
        return
    
    if not hasattr(trajectory, "states") or not trajectory.states:
        return

    # unwrap each EgoStateStamped to EgoState
    states = [_get_traj_state(s) for s in trajectory.states]
    
    # Extract coordinates
    xs = [s.pos.x for s in states]
    ys = [s.pos.y for s in states]

    # guard: need at least 2 points for a line
    if len(xs) < 2:
        if len(xs) == 1:
            ax.scatter(xs, ys, s=40, color="darkred", marker="x", zorder=5)
        return

    # Draw the continuous trajectory line
    ax.plot(xs, ys, linewidth=2.0, color="red", label="Trajectory")
    
    # Mark waypoints with dark red crosses
    ax.scatter(xs, ys, s=40, color="darkred", marker="x", linewidths=1.5, zorder=5)


def _draw_box(ax, x: float, y: float, yaw: float, length: float, width: float,
              color: str = "red", label: Optional[str] = None, 
              linestyle: str = "-", alpha: float = 1.0, fill: bool = False) -> None:
    """
    Draw a yaw-rotated bounding box centered at (x, y).
    Uses an Affine2D transform to rotate the Rectangle around its center.
    """
    # guard against degenerate vehicle sizes
    if length <= 0 or width <= 0:
        return

    # Set facecolor for filled boxes, otherwise transparent
    facecolor = color if fill else "none"

    # Rectangle anchor is bottom-left corner before rotation
    rect = Rectangle(
        (x - length / 2, y - width / 2),
        length,
        width,
        fill=fill,
        facecolor=facecolor,
        edgecolor=color,
        linewidth=2,
        linestyle=linestyle,
        alpha=alpha,
    )

    # rotate the rectangle around (x, y) by yaw angle
    t = transforms.Affine2D().rotate_around(x, y, yaw) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

    if label:
        ax.text(
            x, y + width / 2 + 0.5,   # offset slightly higher above the box
            label,
            fontsize=8,
            color="black",            # better contrast on gray background
            ha="center",
            va="bottom",
            bbox=dict(
                boxstyle="round,pad=0.2",
                facecolor="white",
                edgecolor=color,
                alpha=0.8,            
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