from models.models import *
import numpy as np
import math
import timeit


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
    res =  np.arctan((max_a_lat*params.L)/np.square(velocity))
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
    
    




def main():
    vehicle  = VehicleParameters(
            max_steer=0.7,
            max_steer_rate=0.7,

            L=1.69,
            Lf=0.8,
            Lr=0.89,

            width=1.24,
            length=2.34,

            m=474,
            Iz=600,

            Cf=35000,
            Cr=35000,

            max_acceleration=2.0,
            max_deceleration=6.0,

            mu=0.85
        )
    
    
    
    res = get_steering_angle_range(velocity=4, steering_angle=0.2, vehicle_params=vehicle, max_a_lat=3, internal_dt=100)
    print(res)


    res = get_max_steering_angle(velocity=4, max_a_lat=3, params=vehicle)
    print(res)

    # Benchmarking
    t  = timeit.timeit(lambda: get_steering_angle_range(velocity=100, steering_angle=0.7, vehicle_params=vehicle, max_a_lat=3, internal_dt=100), number=10000)
    print(f"Average time: {(t/10000)*1000:.6f} ms")


if __name__ == "__main__":
    main()
