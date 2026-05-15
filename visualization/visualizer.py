## TODO: TBD after next meeting

import math  
from typing import Optional 

import matplotlib.pyplot as plt  
from matplotlib.patches import Rectangle  
from matplotlib import transforms  

_fig = None # will store the matplotlib figure object
_ax = None # will store the matplotlib axes object
_initialized = False  # tracks whether the plotting window has already been created

def visualize_scene(
        env,  # current environment: lanes + objects
        ego,  # current ego vehicle state
        vehicle_params,  # ego vehicle geometry parameters
        trajectory=None,  # optional planner trajectory
) -> None:
    """
    Live, non-blocking scene visualization.

    Expected inputs:
    - env.lanes -> list of Lane
    - env.objects -> list of DynamicObjectStamped
    - ego.pos.x, ego.pos.y, ego.yaw
    - vehicle_params.length, vehicle_params.width
    - trajectory.states -> list of EgoStateStamped
    """

    global _fig, _ax, _initialized  # reuse the same figure/axes across repeated function calls

    if not _initialized:  # only initialize the plotting window once
        plt.ion()  # enable interactive mode so the plot updates without blocking the program
        _fig, _ax = plt.subplots(figsize=(10, 8))  # create the figure and axes for visualization
        _initialized = True  # mark the plot as initialized

    _ax.clear()  # clear the old frame before drawing the new scene

    _draw_lanes(_ax, env)  # draw lane geometry
    _draw_objects(_ax, env)  # draw dynamic/static objects in the environment
    _draw_ego(_ax, ego, vehicle_params)  # draw the ego vehicle
    _draw_trajectory(_ax, trajectory)  # draw the planner output trajectory

    _ax.set_title("Trajectory Visualization")  # set the plot title
    _ax.set_xlabel("x [m]")  # label x-axis in meters
    _ax.set_ylabel("y [m]")  # label y-axis in meters
    _ax.set_aspect("equal", adjustable="box")  # keep the same scale on both axes
    _ax.grid(True)  # show a grid for easier reading of positions

    _set_view(_ax, env, ego, trajectory)  # adjust the visible plot area

    plt.draw()  # redraw the current figure
    plt.pause(0.001)  # tiny non-blocking pause so the GUI can refresh

def _draw_lanes(ax, env) -> None:
    if env is None or not hasattr(env, "lanes") or env.lanes is None:  # check that environment and lanes exist
        return  # stop if there is nothing to draw
    
    for lane in env.lanes:  # loop through all lanes in the environment
        if not hasattr(lane, "centerline") or not lane.centerline:  # ensure the lane has centerline points
            continue  # skip invalid/empty lanes

        xs = [p.x for p in lane.centerline]  # collect all x-coordinates of the lane centerline
        ys = [p.y for p in lane.centerline]  # collect all y-coordinates of the lane centerline

        ax.plt(xs, ys, linestyle="--", linewidth=1.5, label=f"Lane {lane.id}")  # draw the lane centerline as a dashed line

def _get_object_state(obj_entry):
    # Supports both DynamicObjectStamped and DynamicObject
    return obj_entry.state if hasattr(obj_entry, "entry") else obj_entry  # unwrap stamped object if needed, otherwise return object directly

def _draw_objects(ax, env) -> None:
    if env is None or not hasattr(env, "objects") or env.objects is None:  # check that environment objects exist
        return  # stop if there are no objects to draw
    
    for obj_entry in env.objects:  # loop through all objects in the environment
        obj = _get_object_state(obj_entry)  # get the actual object state

        _draw_box(
            ax=ax,  # axes to draw on
            x=obj.pos.x,  # object x-position
            y=obj.pos.y,  # object y-position
            yaw=obj.yaw,  # object heading angle
            length=obj.length,  # object bounding box length
            width=obj.width,  # object bounding box width
            label=f"Obj {obj.id}",  # label shown near the object
        )

def _get_ego_state(ego_entry):
    """
    Supports both EgoStatedStamped via ego_entry.state
    and plain EgoState directly.
    """
    return ego_entry.state if hasattr(ego_entry, "state") else ego_entry  # unwrap stamped ego state if needed

def _draw_ego(ax, ego, vehicle_params) -> None:
    if ego is None or vehicle_params is None:  # ensure ego state and vehicle parameters are available
        return  # stop if required ego data is missing
    
    _draw_box(
        ax=ax,  # axes to draw on
        x=ego_state.pos.x,  # ego vehicle x-position
        y=ego_state.pos.y,  # ego vehicle y-position
        yaw=ego_state.yaw,  # ego vehicle heading angle
        length=vehicle_params.length,  # ego vehicle length
        width=vehicle_params.width,  # ego vehicle width
        label="Ego",  # label shown near the ego vehicle
    )

    # heading indicator
    arrow_len = max(vehicle_params.length * 0.6, 0.5)  # choose arrow length based on vehicle length with a minimum value
    ax.arrow(
        ego_state.pos.x,  # arrow start x-position
        ego_state.pos.y,  # arrow start y-position
        arrow_len * math.cos(ego_state.yaw),  # arrow x-component based on heading direction
        arrow_len * math.sin(ego_state.yaw),  # arrow y-component based on heading direction
        head_width=max(vehicle_params.width * 0.2, 0.15),  # arrow head width scaled from vehicle width
        length_includes_head=True,  # make the arrow length include the head
    )

def _draw_trajectory(ax, trajectory) -> None:
    if trajectory is None or not hasattr(trajectory, "states") or not trajectory.states:  # check that a valid trajectory exists
        return  # stop if there is no trajectory to draw
    
    xs = [state.state.pos.x for state in trajectory.states]  # extract trajectory x-coordinates
    ys = [state.state.pos.y for state in trajectory.states]  # extract trajectory y-coordinates

    ax.plot(xs, ys, linewidth=2.0, label="Planned trajectory")  # draw the trajectory as a line
    ax.scatter(xs, ys, s=10)  # draw small markers on the trajectory points

def _draw_box(ax, x: float, y: float, yaw: float, length: float, width: float, label: Optional[str] = None) -> None:
    rect = Rectangle(
        (x - length / 2, y - width / 2),  # create rectangle centered at (x, y) before rotation
        length,  # rectangle length
        width,  # rectangle width
        fill=False,  # keep the box transparent inside
        linewidth=2,  # border thickness
    )

    t = transforms.Affine2D().rotate_around(x, y, yaw) + ax.transData  # create rotation transform around the box center
    rect.set_transform(t)  # apply the rotation to the rectangle
    ax.add_patch(rect)  # add the rotated rectangle to the axes

    if label:  # check whether a label was provided
        ax.text(x, y, label)  # draw the label at the box position

def _set_view(ax, ego) -> None:
    """
    Ego-centered camera view.
    Better than fitting all lanes, especially for very long roads.
    """
    if ego is None:  # fallback view if no ego vehicle is available
        ax.set_xlim(-10, 10)  # default x-axis range
        ax.set_ylim(-10, 10)  # default y-axis range
        return  # stop after setting fallback view

    ego_state = _get_ego_state(ego)  # get plain ego state regardless of wrapper type

    x = ego_state.pos.x  # ego x-position
    y = ego_state.pos.y  # ego y-position

    x_margin = 60.0  # horizontal margin around the ego vehicle
    y_margin = 15.0  # vertical margin around the ego vehicle

    ax.set_xlim(x - x_margin, x + x_margin)  # center x-axis view around ego position
    ax.set_ylim(y - y_margin, y + y_margin)  # center y-axis view around ego position