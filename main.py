from visualization.visualizer import visualize_scene
from simulation.simulate import Simulation
from planner.planner import plan
from models.models import PlanningRequest, GoalRegion, Vector2D, Trajectory
from utils.helper import load_vehicle_parameters

def main():
    sim = Simulation()

    goal_region = GoalRegion(
        center=Vector2D(x=100.0, y=2.0),
        length=5.0,
        width=5.0,
        yaw=0.0,
        yaw_tolerance=0.1,
        target_velocity=50.0,
        velocity_tolerance=5.0
    )

    vehicle_params = load_vehicle_parameters()
    goal_reached = False
    full_trajectory = Trajectory(states=[])

    while not goal_reached:

        planning_request = PlanningRequest(
            start_state=sim.get_ego_state(),
            goal_region=goal_region,
            vehicle_params=vehicle_params,
            environment=sim.get_environment(),
            horizon=1000,
            dt=100,
            output_dt=100,
            max_compute_time=200
        )

        plan_result = plan(planning_request)

        if plan_result.success and plan_result.trajectory is not None:
            full_trajectory.states.extend(plan_result.trajectory.states)

            if plan_result.goal_reached:
                print("Goal reached!")
                goal_reached = True
        
        else:
            print(f"Planning failed: {plan_result.status_message}")
            break
        
    
    visualize_scene(
        env=sim.get_environment(),
        ego=sim.get_ego_state(),
        vehicle_params=vehicle_params,
        trajectory=full_trajectory
    )