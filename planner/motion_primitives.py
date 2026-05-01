from models.models import *
import numpy as np
import math
import timeit
from dataclasses import dataclass


@dataclass
class MotionPrimitive:
    steering_angle: float       # steering input δ [rad]
    acceleration: float         # longitudinal acceleration [m/s²]  
    dt:int                      # the time the MotionPrimitive is used


def get_max_steering_angle(velocity: float, max_a_lat: float, params: VehicleParameters) -> float:
    """
    Compute the maximum feasible steering angle (in radians) based on a lateral
    acceleration constraint using a simplified kinematic bicycle model.

    Model assumption:
        a_lat = (v^2 / L) * tan(delta)

    Solving for the steering angle delta:
        delta = arctan((a_lat * L) / v^2)

    The resulting steering angle is additionally limited by the vehicle's
    physical maximum steering angle (params.max_steer).

    Parameters
    ----------
    v : float
        Vehicle speed in meters per second (m/s). Must be > 0.
    max_a_lat : float
        Maximum allowable lateral acceleration in meters per second squared (m/s²).
    params : VehicleParameters
        Vehicle parameter object containing at least:
        - L: wheelbase [m]
        - max_steer: maximum steering angle [rad]

    Returns
    -------
    float
        Maximum steering angle in radians, limited by both the lateral
        acceleration constraint and the vehicle's steering limit.
    """
    # If moving very slowly, the lateral acceleration constraint is irrelevant.
    if abs(velocity) < 0.1: 
        return params.max_steer
    
    # Compute steering angle using the rearranged bicycle model formula
    res =  np.arctan((max_a_lat*params.wheel_base)/np.square(velocity))
    return min(res, params.max_steer)


def get_steering_angle_range(velocity: float,
                             steering_angle: float,
                             vehicle_params: VehicleParameters,
                             max_a_lat: float,
                             internal_dt: int) -> tuple[float, float]: 
    """
    Compute the feasible steering range based on mechanical limits, 
    actuator rate, and lateral acceleration safety constraints.

    Parameters
    ----------
    velocity : float
        Current vehicle speed in m/s.
    steering_angle : float
        Current Steering angle in radians.
    vehicle_params : VehicleParameters
        Vehicle constants (wheelbase, max steer, max steer rate).
    max_a_lat : float
        Maximum allowable lateral acceleration in m/s².
    internal_dt : int
        Time step duration in milliseconds (ms).

    Returns
    -------
    tuple[float, float]
        Feasible (min_angle, max_angle) in radians.
    """
    
    # 1. Physical Sanity Check
    if abs(steering_angle) > vehicle_params.max_steer:
        raise ValueError("Steering Angle exceeds VehicleParameters limits")
    
    # 2. Safety & Mechanical: Get max angle allowed by lateral acceleration or physical stop
    max_steering_angle_global = get_max_steering_angle(velocity, max_a_lat, vehicle_params)

    # 3. Dynamic: Max steering travel possible within the time step (dt)
    max_steering_delta = vehicle_params.max_steer_rate * (internal_dt / 1000)
    min_steering_angle_local = steering_angle - max_steering_delta
    max_steering_angle_local = steering_angle + max_steering_delta

    # 4. Intersection: Limit reachable range by safety and mechanical bounds
    min_steering_angle = max(min_steering_angle_local, -max_steering_angle_global)
    max_steering_angle = min(max_steering_angle_local, max_steering_angle_global)

    # 5. Reachability: Ensure min_steering_angle <= max_steering_angle by collapsing the range 
    # if safety limits force a movement faster than the actuator's rate capacity.
    if min_steering_angle > max_steering_angle:
        max_steering_angle = min_steering_angle
    elif max_steering_angle < min_steering_angle:
        min_steering_angle = max_steering_angle

    return (min_steering_angle, max_steering_angle)


def get_acceleration_range(velocity: float,
                           velocity_limit: float,
                           velocity_limit_tolerance: float,
                           velocity_limit_thresh: float,
                           acceleration: float,
                           acceleration_limit: float,
                           deceleration_limit: float,
                           positiv_jerk_limit: float,
                           negativ_jerk_limit: float,
                           internal_dt: int) -> tuple[float, float]:
    """
    Calculates the permissible acceleration range [a_min, a_max] for the next time step.
    Takes jerk limits, velocity limits, and tolerance bands into account for predictive 
    and deadbeat control.

    
    Parameters
    ----------
    velocity : float
        Current vehicle speed in m/s.
    velocity_limit : float
        The target maximum speed limit in m/s.
    velocity_limit_tolerance : float
        Upper tolerance above the velocity limit. Exceeding this triggers maximum hard braking.
    velocity_limit_thresh : float
        Threshold defining the docking zone (below the limit and above 0) 
        to smoothly transition into deadbeat control and prevent chattering.
    acceleration : float
        Current vehicle acceleration in m/s².
    acceleration_limit : float
        Absolute physical maximum acceleration bound (> 0) in m/s².
    deceleration_limit : float
        Absolute physical maximum deceleration bound (< 0) in m/s².
    positiv_jerk_limit : float
        Maximum allowed rate of increasing acceleration (> 0) in m/s³.
    negativ_jerk_limit : float
        Maximum allowed rate of decreasing acceleration (< 0) in m/s³.
    internal_dt : int
        Time step duration in seconds (ms).

    Returns
    -------
    tuple[float, float]
        Feasible (min_acceleration, max_acceleration) range in m/s² for the next time step.
    """
    #Conversion from milliseconds to seconds
    internal_dt_sec = internal_dt/1000.0

    # Prevent Division by Zero if internal_dt is strictly 0 (failsafe)
    if internal_dt_sec <= 0.0:
        return 0.0, 0.0

    # --- 1. Physical limits for the current time step (Jerk limitation) ---
    # Determine the mechanically achievable acceleration boundaries for this specific time step.
    abs_max_a = min(acceleration + (positiv_jerk_limit * internal_dt_sec), acceleration_limit)
    abs_min_a = max(acceleration + (negativ_jerk_limit * internal_dt_sec), deceleration_limit)

    # --- 2. Calculate maximum permissible acceleration (a_accel_max) ---
    
    # CASE A: Velocity strictly exceeds the upper tolerance band.
    if velocity > (velocity_limit + velocity_limit_tolerance):
        # Force maximum possible deceleration (initiate hard braking).
        a_accel_max = abs_min_a
        a_decel_max = abs_min_a

        return float(a_decel_max), float(a_accel_max)

    # CASE B: Velocity is in the upper docking zone or within the tolerance band.
    elif velocity >= (velocity_limit - velocity_limit_thresh):
        # Apply deadbeat control to hit V_limit exactly, strictly bounded by physical jerk limits.
        req_a_max = (velocity_limit - velocity) / internal_dt_sec    
        a_accel_max = min(req_a_max, abs_max_a)
        a_accel_max = max(a_accel_max, abs_min_a)
        
    # CASE C: Velocity is safely below the limit (Predictive look-ahead).
    else:
        # Simulate: If we start reducing acceleration to 0 right now,
        # what will our resulting velocity be?
        predicted_v = velocity
        temp_a = acceleration
        
        # Calculate the discrete steps required to bring current acceleration down to 0.
        steps_to_zero = math.ceil(abs(temp_a / (negativ_jerk_limit * internal_dt_sec)))
        
        for _ in range(steps_to_zero):
            # Simulate gradually shedding acceleration towards 0.
            temp_a = max(0.0, temp_a + (negativ_jerk_limit * internal_dt_sec))
            predicted_v += temp_a * internal_dt_sec

        # If the predicted velocity enters the upper docking zone, enforce maximum deceleration limit.
        if predicted_v > (velocity_limit - velocity_limit_thresh):
            a_accel_max = abs_min_a
        else:
            a_accel_max = abs_max_a

    # --- 3. Calculate minimum permissible acceleration (a_decel_max) ---
    # Goal: Prevent the vehicle from reversing (v < 0).

    # CASE D: Velocity is in the lower docking zone (approaching standstill).
    if velocity <= velocity_limit_thresh:
        # Apply deadbeat control to bring velocity exactly to 0, bounded by physical jerk limits.
        req_a_min = (0.0 - velocity) / internal_dt_sec
        a_decel_max = min(req_a_min, abs_max_a)
        a_decel_max = max(a_decel_max, abs_min_a)
    
    # CASE E: Velocity is safely above 0 (Predictive look-ahead for stopping).
    else:
        predicted_v_min = velocity
        temp_a_min = acceleration
        
        # Calculate the discrete steps required to bring negative acceleration (braking) back up to 0.
        steps_to_zero_dec = math.ceil(abs(temp_a_min / (positiv_jerk_limit * internal_dt_sec)))
        
        for _ in range(steps_to_zero_dec):
            # Simulate gradually reducing the braking force towards 0.
            temp_a_min = min(0.0, temp_a_min + (positiv_jerk_limit * internal_dt_sec))
            predicted_v_min += temp_a_min * internal_dt_sec 

        # If the predicted velocity drops into the lower docking zone, force an increase in acceleration.
        if predicted_v_min < velocity_limit_thresh:
            a_decel_max = abs_max_a 
        else:
            a_decel_max = abs_min_a

    # --- 4. Return as [Min, Max] ---
    # Safety check: Ensure the minimum bound never exceeds the maximum bound.
    if a_decel_max > a_accel_max:
        a_decel_max = a_accel_max
        
    return float(a_decel_max), float(a_accel_max)



def get_motion_primitives(velocity: float,
                          steering_angle: float,
                          vehicle_params: VehicleParameters,
                          acceleration_split: list[float],
                          steering_angle_split: list[float],
                          max_a_lat: float,
                          velocity_limit: float,
                          velocity_limit_tolerance: float,
                          velocity_limit_thresh: float,
                          acceleration: float,
                          acceleration_limit: float,
                          deceleration_limit: float,
                          positiv_jerk_limit: float,
                          negativ_jerk_limit: float,
                          internal_dt: int) -> list[MotionPrimitive]:
    """
    Generates a deterministic set of feasible motion primitives (action trajectories) 
    for a Hybrid A* planner based on vehicle dynamics and kinematic limits.

    Parameters
    ----------
    velocity : float
        Current vehicle speed in m/s.
    steering_angle : float
        Current steering angle in radians.
    vehicle_params : VehicleParameters
        Data class containing physical vehicle dimensions and constraints.
    acceleration_split : list[float]
        List of percentages (e.g., [0, 50, 100]) to sample the feasible acceleration range. 
        0% represents max deceleration, 100% represents max acceleration.
    steering_angle_split : list[float]
        List of percentages (e.g., [-100, 0, 100]) to sample the feasible steering range.
        -100% represents max right/left, 0% is straight, 100% represents max left/right.
    max_a_lat : float
        Maximum allowable lateral acceleration in m/s².
    velocity_limit : float
        The target maximum speed limit in m/s.
    velocity_limit_tolerance : float
        Upper tolerance above the velocity limit. Exceeding this triggers braking.
    velocity_limit_thresh : float
        Threshold defining the docking zone to smoothly transition into deadbeat control.
    acceleration : float
        Current vehicle acceleration in m/s².
    acceleration_limit : float
        Absolute physical maximum acceleration bound (> 0) in m/s².
    deceleration_limit : float
        Absolute physical maximum deceleration bound (< 0) in m/s².
    positiv_jerk_limit : float
        Maximum allowed rate of increasing acceleration (> 0) in m/s³.
    negativ_jerk_limit : float
        Maximum allowed rate of decreasing acceleration (< 0) in m/s³.
    internal_dt : int
        Time step duration for the primitive in milliseconds (ms).

    Returns
    -------
    list[MotionPrimitive]
        A list of physically feasible and safe motion primitives for the next expansion step.
    """
    
    # 1. Initialize the empty list for the generated motion primitives
    motion_primitives: list[MotionPrimitive] = []
    
    # 2. Get the dynamically feasible minimum and maximum acceleration bounds
    acceleration_min, acceleration_max = get_acceleration_range(
        velocity = velocity,
        velocity_limit = velocity_limit,
        velocity_limit_tolerance = velocity_limit_tolerance,
        velocity_limit_thresh = velocity_limit_thresh,
        acceleration = acceleration,
        acceleration_limit = acceleration_limit,
        deceleration_limit = deceleration_limit,
        positiv_jerk_limit = positiv_jerk_limit,
        negativ_jerk_limit = negativ_jerk_limit,
        internal_dt = internal_dt
    )
    
    # Time conversion for kinematic projection calculations
    internal_dt_sec = internal_dt / 1000.0
    
    # Compute the total available acceleration span
    acceleration_delta = acceleration_max - acceleration_min
    acceleration_values: list[float] = []

    # 3. Generate the discrete acceleration samples based on the provided percentages
    for split in acceleration_split:
        # Convert the percentage split (0-100) into an actual acceleration offset
        acceleration_delta_split = acceleration_delta * (split / 100.0)
        acceleration_value = acceleration_min + acceleration_delta_split
        
        # Append uniquely to maintain deterministic ordering without duplicates
        if acceleration_value not in acceleration_values:
            acceleration_values.append(acceleration_value)

    # 4. Generate steering samples for each valid acceleration sample
    for acceleration_value in acceleration_values:
        
        # Predict the resulting velocity at the end of this primitive's time step.
        # Bounded to >= 0.0 to prevent evaluating backward motion physics.
        target_velocity = max(0.0, velocity + (acceleration_value * internal_dt_sec))
        
        # Retrieve the dynamically feasible steering bounds for the predicted target velocity
        min_angle, max_angle = get_steering_angle_range(
            velocity = target_velocity,
            steering_angle = steering_angle,
            vehicle_params = vehicle_params,
            max_a_lat = max_a_lat,
            internal_dt = internal_dt
        )
        
        # Calculate the geometric center and radius of the available steering envelope
        center_point = (max_angle + min_angle) / 2.0
        distance_from_center = abs(max_angle - center_point)

        # Generate discrete steering commands based on the provided percentages
        for split in steering_angle_split:
            # Convert the percentage split (-100 to 100) into a steering offset
            steering_angle_delta_split = distance_from_center * (split / 100.0)
            steering_angle_value = center_point + steering_angle_delta_split
            
            # Instantiate the primitive and add it to the expansion list
            motion_primitive = MotionPrimitive(
                steering_angle = steering_angle_value,
                acceleration = acceleration_value,
                dt = internal_dt
            )
            motion_primitives.append(motion_primitive)

    # Sort primitives to optimize A* expansion (straight and constant speed first):
    # 1. Absolute steering magnitude (closest to 0 straight ahead first).
    # 2. Steering sign preference (positive steering angles before negative ones).
    # 3. Absolute acceleration magnitude (closest to 0 m/s² / constant speed first).
    # 4. Acceleration sign preference (positive acceleration before braking).
    motion_primitives.sort(key=lambda mp: (
        round(abs(mp.steering_angle), 5), 
        -mp.steering_angle, 
        round(abs(mp.acceleration), 5),
        -mp.acceleration
    ))
    
    return motion_primitives



def print_motion_primitives(primitives: list[MotionPrimitive]) -> None:
    """
    Prints a list of MotionPrimitives as a clearly formatted table in the console.
    """
    if not primitives:
        print("The list of Motion Primitives is empty.")
        return

    # 1. Define the table header (with fixed column widths)
    header = f"{'No.':<5} | {'Steering Angle [rad]':<22} | {'Acceleration [m/s²]':<22} | {'dt [ms]':<10}"
    print("\n" + header)
    
    # 2. Dynamically adjust the separator line to match the header length
    print("-" * len(header))

    # 3. Iterate through all primitives and print them with formatting
    for i, mp in enumerate(primitives, start=1):
        steer_str = f"{mp.steering_angle:.5f}"
        accel_str = f"{mp.acceleration:.2f}"
        
        print(f"{i:<5} | {steer_str:<22} | {accel_str:<22} | {mp.dt:<10}")
    
    print(f"{'-' * len(header)}")
    print(f"Total: {len(primitives)} Motion Primitives\n")  
    
    




def main():
    vehicle  = VehicleParameters(
            max_steer=0.7,            # maximum steering angle wheels [rad]
            max_steer_rate=0.7  ,     # maximum steering rate wheels [rad/s]

            Lf=0.9442,                  # CoG to front axle [m]
            Lr=0.7417,                    # CoG to rear axle [m]
            Iz=430.166,                    #Moment of inertia [kg.m2]

            wheel_length= 0.531,          #Wheel length [m]
            wheel_width = 0.125,            #Wheel width [m]

            wheel_base= 1.686,            #Wheel base [m]
            track= 1.094,                #Vehile track [m]

            width= 1.381,                 # vehicle width [m]
            length = 2.338,               # vehicle length [m]
            rear_to_wheel = 0.339,        #Distance rear to axel [m]

            m= 633,                   # mass [kg]

            Cf= 2*32857.5,                    # front cornering stiffness [N/rad]
            Cr= 2*32857.5,                   # rear cornering stiffness [N/rad]

            max_acceleration= 2.0,     # max longitudinal acceleration [m/s²]
            max_deceleration = -2.0,     # max braking deceleration [m/s²]

            mu = 2.0                   # tire-road friction coefficient [-]
        )
    
    
    
    #res = get_steering_angle_range(velocity=4, steering_angle=0.2, vehicle_params=vehicle, max_a_lat=3, internal_dt=100)
    #print(res)
#
#
    #res = get_max_steering_angle(velocity=4, max_a_lat=3, params=vehicle)
    #print(res)
#
    ## Benchmarking
    #t  = timeit.timeit(lambda: get_steering_angle_range(velocity=100, steering_angle=0.7, vehicle_params=vehicle, max_a_lat=3, internal_dt=100), number=10000)
    #print(f"Average time: {(t/10000)*1000:.6f} ms")



    ## Benchmarking
    t  = timeit.timeit(lambda: get_acceleration_range(velocity = 5,
                           velocity_limit = 13.88,
                           velocity_limit_tolerance = 0.55,
                           velocity_limit_thresh = 0.55,
                           acceleration = 1.8,
                           acceleration_limit = 2,
                           deceleration_limit = -2,
                           positiv_jerk_limit = 1,
                           negativ_jerk_limit = -2,
                           internal_dt = 100),
                           number=100)
    print(f"Average time: {(t/10000)*1000:.6f} ms")
    

    #res = get_acceleration_range(velocity = 10,
    #                       velocity_limit = 13.88,
    #                       velocity_limit_tolerance = 0.55,
    #                       velocity_limit_thresh = 0.55,
    #                       acceleration = -1,
    #                       acceleration_limit = 2,
    #                       deceleration_limit = -2,
    #                       positiv_jerk_limit = 1,
    #                       negativ_jerk_limit = -2,
    #                       internal_dt = 100)
    #print(res)

    res = get_motion_primitives(velocity=5,
                                steering_angle=0,
                                vehicle_params=vehicle,
                                acceleration_split=[0,25,50,75,100],
                                steering_angle_split=[-100,-50,-25,0,25,50,100],
                                max_a_lat=3,
                                velocity_limit=13.88,
                                velocity_limit_tolerance=0.55,
                                velocity_limit_thresh=0.55,
                                acceleration=0,
                                acceleration_limit=2,
                                deceleration_limit=-2,
                                positiv_jerk_limit=1,
                                negativ_jerk_limit=-1,
                                internal_dt=100)
    
    print_motion_primitives(res)


    t  = timeit.timeit(lambda: get_motion_primitives(velocity=5,
                                                    steering_angle=0,
                                                    vehicle_params=vehicle,
                                                    acceleration_split={0,25,50,75,100},
                                                    steering_angle_split={-100,-50,-25,0,25,50,100},
                                                    max_a_lat=3,
                                                    velocity_limit=13.88,
                                                    velocity_limit_tolerance=0.55,
                                                    velocity_limit_thresh=0.55,
                                                    acceleration=0,
                                                    acceleration_limit=2,
                                                    deceleration_limit=-2,
                                                    positiv_jerk_limit=1,
                                                    negativ_jerk_limit=-1,
                                                    internal_dt=100),
                                                    number = 1000)
    
    print(f"Average time: {(t/1000)*1000:.6f} ms")

                                                                            



if __name__ == "__main__":
    main()
