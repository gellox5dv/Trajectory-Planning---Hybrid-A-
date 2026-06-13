from visualization.visualizer import visualize_scene
from simulation.simulate import Simulation
from planner.planner import plan
<<<<<<< HEAD
from models.models import PlanningRequest, GoalRegion, Vector2D, Trajectory, PredictedEnvironment
from utils.helper import load_vehicle_parameters
from motion.motion_prediction import predict_motion_constant_velocity
=======
from models.models import PlanningRequest, Trajectory, PredictedEnvironment
from utils.helper import get_goal_region
from motion.motion_prediction import predict_motion_constant_velocity
from controllers.controllers import MPCController
>>>>>>> feature/bicycle-only

import hydra
from omegaconf import DictConfig


@hydra.main(version_base=None, config_path="configs", config_name="config")
def main(cfg: DictConfig):
<<<<<<< HEAD
    sim = Simulation()

    goal_region = GoalRegion(
        center=Vector2D(x=1800.0, y=2.0),
        length=5.0,
        width=5.0,
        yaw=0.0,
        yaw_tolerance=0.1,
        target_velocity=50.0,
        velocity_tolerance=5.0
    )

    HORIZON = 1000
    DT = 100

    vehicle_params = load_vehicle_parameters()
    goal_reached = False
    full_trajectory = Trajectory(states=[])
=======
    sim = Simulation(cfg)

    goal_reached = False
    full_trajectory = Trajectory(states=[])
    controller = MPCController(cfg.vehicle, cfg.controller.mpc)
>>>>>>> feature/bicycle-only

    while not goal_reached:

        curr_env = sim.get_environment()
<<<<<<< HEAD
        pred_objs = predict_motion_constant_velocity(curr_env.objects, prediction_horizon=HORIZON, dt=DT)
        pred_env = PredictedEnvironment(
            objects={obj.state.id: [obj] for obj in pred_objs},
            lanes=curr_env.lanes,
            dt=DT,
            horizon=HORIZON
=======
        pred_objs = predict_motion_constant_velocity(curr_env.objects, prediction_horizon=cfg.planner.horizon, dt=cfg.planner.dt_sim)
        pred_env = PredictedEnvironment(
            objects={obj.state.id: [obj] for obj in pred_objs},
            lanes=curr_env.lanes,
            dt=cfg.planner.dt_sim,
            horizon=cfg.planner.horizon
>>>>>>> feature/bicycle-only
        )

        planning_request = PlanningRequest(
            start_state=sim.get_ego_state(),
<<<<<<< HEAD
            goal_region=goal_region,
            target_speed=goal_region.target_velocity,
            vehicle_params=vehicle_params,
            environment= pred_env,
            horizon=HORIZON,
            dt=DT,
            dt_output=DT,
            max_compute_time=200
=======
            goal_region=get_goal_region(
                curr_ego_state=sim.get_ego_state(),
                lanes=curr_env.lanes,
                horizon=cfg.scenario.goal.horizon,
                length=cfg.scenario.goal.length,
                width=cfg.scenario.goal.width
            ),
            target_speed=13+8/9,
            environment=pred_env
>>>>>>> feature/bicycle-only
        )

        plan_result = plan(planning_request, cfg)

        if plan_result.success and plan_result.trajectory is not None:
<<<<<<< HEAD
            # controller recalculates the ego input
            # simulation applies the steering rate and acceleration

            # full_trajectory.states.extend()# applied trajectory from the simulation)
=======
            acceleration, steering_rate = controller.compute_control(sim.get_ego_state(), plan_result.trajectory)
            sim.step(acceleration, steering_rate, cfg.controller.mpc.dt)
>>>>>>> feature/bicycle-only

            if goal_reached:
                print("Goal reached!")
                break
        
        else:
            print(f"Planning failed: {plan_result.status_message}")
            break
        
    
    visualize_scene(
<<<<<<< HEAD
        env=sim.get_environment(),
        ego=sim.get_ego_state(),
        vehicle_params=vehicle_params,
        trajectory=full_trajectory
    )
=======
        env=sim.get_environment(0.0),
        ego=sim.get_ego_state(0.0),
        vehicle_params=cfg.vehicle,
        trajectory=full_trajectory
    )

if __name__ == "__main__":
    main()
>>>>>>> feature/bicycle-only
