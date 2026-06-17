from visualization.visualizer import visualize_scene
from simulation.simulate import Simulation
from planner.planner import plan
from models.models import PlanningRequest, GoalRegion, Vector2D, Trajectory, PredictedEnvironment
from motion.motion_prediction import predict_motion_constant_velocity
from visualization.visualizer import visualize_scene
import hydra
from omegaconf import DictConfig
import threading
import time
from controllers.controllers import MPCController
from utils.helper import get_goal_region, get_signed_magnitude
from prediction.predictivity import predict_environment
from models.models import *
from collision.collision import get_ego_lane_info
from collision.collision import get_distance_to_objects





        




@hydra.main(version_base=None, config_path="configs", config_name="config")
def main(cfg: DictConfig) -> None:
    """
    Main entry point for the autonomous driving stack. Initializes all components, 
    starts the background controller thread, and runs the high-level planner continuously.

    Args:
        cfg (DictConfig): The Hydra configuration object containing system and vehicle parameters.

    Returns:
        None
    """
    sim = Simulation(cfg)
    controller = MPCController(cfg.vehicle, cfg.controller)
    
    newest_trajectory = None

    
    while True:
        #TODO possibly stop time and wait at the end for remaining time
        # 1. Fetch environment and state for the planner
        curr_env = sim.get_environment()
        ego_state_stamped = sim.get_ego_state()
        # TODO: predict environment
        pred_env = predict_environment(environment = curr_env,
                                       prediction_horizon = cfg.planner.horizon,
                                       dt = cfg.planner.dt_sim)
        

         # TODO: get dynamic velocity limit
        
        velocity_limit = 10.0

        _,_,_,_,_,velocity_limit_lane = get_ego_lane_info(ego_state=ego_state_stamped.state,
                                                           ego_length = cfg.vehicle.length,
                                                           ego_width = cfg.vehicle.width,
                                                           ego_rear_to_wheel = cfg.vehicle.width,
                                                           lanes = curr_env.lanes)
         
        if(velocity_limit_lane is not None):
            velocity_limit = velocity_limit_lane

        
        # TODO: calculate goal region (dummy arguments used)
        goal_region = get_goal_region(curr_ego_state=ego_state_stamped,
                                      lanes=curr_env.lanes,
                                      horizon=cfg.planner.horizon,
                                      length=3.0,
                                      width=3.0,
                                      target_speed= velocity_limit_lane)
        
       
       
        
        planning_request = PlanningRequest(
            start_state=ego_state_stamped,
            goal_region=goal_region,
            target_speed=velocity_limit,
            environment=pred_env
        )
        # 2. Compute the plan 
        plan_result = plan(planning_request, cfg, True)
        # 3. Pass the new trajectory to the controller
        if plan_result.success and plan_result.trajectory is not None:
            print('Path found')
            newest_trajectory = plan_result.trajectory
        else:
            newest_trajectory = plan_result.trajectory
            print(f"No path found: {plan_result.status_message}")


            # If no path is found, the old trajectory remains in shared_state.
            # The MPC will simply continue following it (default fallback behavior).
        
        
        res = get_distance_to_objects(current_ego=ego_state_stamped,
                                previous_ego=ego_state_stamped,
                                predicted_env=pred_env,
                                ego_length = cfg.vehicle.length,
                                ego_width = cfg.vehicle.width,
                                ego_rear_to_wheel = cfg.vehicle.width,
                                resolution_ms=10,
                                calc_exact_distance=5)
        
        print(res)
        
        
        delta_time = 100
        x = 0

        

        from visualization.tree_visualizer import visualize_search_tree
        visualize_search_tree(
            plan_result=plan_result,
            pred_env=pred_env,
            goal_region=goal_region,
            cfg=cfg,
            output_filename="debug_tree.html",
            vehicle_cfg = cfg.vehicle
        )

        

        #while x < cfg.planner.max_compute_time / delta_time:
        while x < 4000 / delta_time:

            start_time = time.perf_counter()
            # 1. Get current state from simulation
            ego_state = sim.get_ego_state()
            # 2. Safely read the latest trajectory using the lock

            curr_env = sim.get_environment()

           
 
           
            # 3. Compute control commands and apply fallback if necessary
            acc, steer_rate = 0.0, 0.0
            if newest_trajectory is not None:
                try:
                    acc, steer_rate = controller.compute_control(ego_state, newest_trajectory)
                except Exception:
                    # Fallback: Emergency braking if the MPC fails to find a solution
                    acc, steer_rate = cfg.vehicle.max_deceleration, 0.0 
                    print(Exception)
                    print('The Controller didnt find a solution -> breaking')
            else:
                # No trajectory available yet -> standstill / emergency brake
                acc = cfg.vehicle.max_deceleration #TODO aus config maximale verzögerung auslesen
            # 4. Advance the simulation (the controller drives the simulation time)
            dt_ms = int(delta_time)
            sim.step(acc, steer_rate, dt_ms)
            # 4. Visualization (runs safely in the main thread)

            
            visualize_scene(
                env=curr_env,
               ego=ego_state,
                vehicle_params=cfg.vehicle,
                trajectory=plan_result.trajectory,
                goal_region = goal_region
           )
            
            end_time = time.perf_counter()
            duration = end_time - start_time
            delta = (delta_time/1000) - duration

            if delta > 0:
                time.sleep(delta)
            else:
                print('Compute to slow for realtime sim')
                #print(delta)

            x = x +1
        
   

if __name__ == "__main__":
    main()


#@hydra.main(version_base=None, config_path="configs", config_name="config")
def _main(cfg: DictConfig):
    sim = Simulation()
    controller = None #TODO init of the controller
    #TODO start thread for controller

    while True:

        curr_env = sim.get_environment()
        ego_state_stamped = sim.get_ego_state()

        #TODO predict environment
        pred_env = curr_env
        ############

        #TODO calculate goal region
        goal_region = GoalRegion(...)
        #############

        #TODO get velocity_limit -> target_speed
        velocity_limit = None
        

        planning_request = PlanningRequest(start_state=ego_state_stamped,
                                           goal_region=goal_region,
                                           target_speed=velocity_limit,
                                           environment=pred_env)

        plan_result = plan(planning_request, cfg)

        if plan_result.success and plan_result.trajectory is not None:
              #TODO speak with controller thread
              ...

        else:
            print(f"No path found: {plan_result.status_message}")
            #TODO: Vehicle should follow the last caluclated trajectorie and break 
        
        #TODO change visualizier to get vehicle config
        visualize_scene(
            env=curr_env,
            ego=ego_state_stamped,
            vehicle_params=vehicle_params,
            trajectory=plan_result.trajectory
        )
        