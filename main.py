from visualization.visualizer import visualize_scene
from simulation.simulate import Simulation
from planner.planner import plan
from models.models import PlanningRequest, GoalRegion, Vector2D, Trajectory, PredictedEnvironment
from utils.helper import load_vehicle_parameters
from motion.motion_prediction import predict_motion_constant_velocity

import hydra
from omegaconf import DictConfig


@hydra.main(version_base=None, config_path="configs", config_name="config")
def main(cfg: DictConfig):
    sim = Simulation()

    goal_region = GoalRegion(
        center=Vector2D(x=1800.0, y=2.0),
        length=5.0,
        width=5.0,
        yaw=0.0
    )

    HORIZON = cfg.planner.horizon
    DT = cfg.planner.dt

    vehicle_params = load_vehicle_parameters()
    goal_reached = False
    full_trajectory = Trajectory(states=[])

    while not goal_reached:

        curr_env = sim.get_environment()
        pred_objs = predict_motion_constant_velocity(curr_env.objects, prediction_horizon=HORIZON, dt=DT)
        pred_env = PredictedEnvironment(
            objects={obj.state.id: [obj] for obj in pred_objs},
            lanes=curr_env.lanes,
            dt=DT,
            horizon=HORIZON
        )

        planning_request = PlanningRequest(
            start_state=sim.get_ego_state(),
            goal_region=goal_region,
            target_speed=13+8/9,
            environment=pred_env
        )

        plan_result = plan(planning_request, cfg)

        if plan_result.success and plan_result.trajectory is not None:
            # controller recalculates the ego input
            # simulation applies the steering rate and acceleration

            # full_trajectory.states.extend()# applied trajectory from the simulation)

            if goal_reached:
                print("Goal reached!")
                break
        
        else:
            print(f"Planning failed: {plan_result.status_message}")
            break
        
    
    visualize_scene(
        env=sim.get_environment(),
        ego=sim.get_ego_state(),
        vehicle_params=vehicle_params,
        trajectory=full_trajectory
    )