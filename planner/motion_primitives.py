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
                           velocity_limit_tolerance:float,
                           velocity_limit_thresh:float,
                           acceleration: float,
                           acceleration_limit: float,
                           deceleration_limit: float,
                           positiv_jerk_limit: float,
                           negativ_jerk_limit: float,
                           internal_dt: int) -> tuple[float, float]:
    

    #a_to_v_limit = (velocity_limit - velocity)/internal_dt
    #a_to_v_0     = (0 - velocity)/internal_dt

    if(v < v + velocity_limit_tolerance and v > v - velocity_limit_thresh):
        a_accel_max = 0



    elif v < velocity_limit + velocity_limit_tolerance:
        a = acceleration
        for i in range(math.ceil(acceleration/(negativ_jerk_limit*internal_dt))):
                a = a + (negativ_jerk_limit * internal_dt)
                if a < 0: a = 0
                v = v + a
        if(v>velocity_limit-velocity_limit_thresh):
            a_accel_max = acceleration + (negativ_jerk_limit * internal_dt) 
        else:
            a_accel_max = min(acceleration + positiv_jerk_limit * internal_dt, acceleration_limit)

    else:
        a = acceleration
        #

    a = acceleration
    for i in range(math.ceil(acceleration/(positiv_jerk_limit*internal_dt))):
            a = a + (positiv_jerk_limit * internal_dt)
            if a > 0: a = 0
            v = v + a
    if(v<0+velocity_limit_thresh):
        a_decel_max = acceleration + (positiv_jerk_limit * internal_dt) 
    else:
        a_decel_max = max(acceleration + negativ_jerk_limit * internal_dt, deceleration_limit)
    


    #bereich definieren (eigentlich tolercane) in dem wir eine beschleunigung von 0 tollerieren
    # v  betrachten v aktuell ist größer als v max

#zurcürechen von v_max

#hochrechen von v 0
#am besten v_max-delta
    

        a_decay 


    #velocity_limit_tolerance betrachten!

    #Minimale Geschwindigkeit von 0 betrachten inklusive tolleranz!
    # Wichtig: Es muss vereinfacht auf die velocity_limit(das ist ja i.d.R. die Zielgeschwindigkeit) geregelt werden!
    # Frage wie macht man die Verteilung der Regelung (der Tracker nimmt ja die Waypoints und Regelt auch auf die Zielgeschwindigkeit)



def get_acceleration_range(velocity: float,
                           velocity_limit: float,
                           velocity_limit_tolerance: float,
                           velocity_limit_thresh: float,
                           acceleration: float,
                           acceleration_limit: float,
                           deceleration_limit: float,
                           positiv_jerk_limit: float,
                           negativ_jerk_limit: float,
                           internal_dt: float) -> tuple[float, float]:
    """
    Calculates the permissible acceleration range [a_min, a_max] for the next time step.
    Takes jerk limits, velocity limits, and tolerance bands into account.
    
    Assumptions: 
    - positiv_jerk_limit > 0
    - negativ_jerk_limit < 0
    """

    # --- 1. Physical limits for the current time step (Jerk limitation) ---
    # What is mechanically possible right now?
    abs_max_a = min(acceleration + (positiv_jerk_limit * internal_dt), acceleration_limit)
    abs_min_a = max(acceleration + (negativ_jerk_limit * internal_dt), deceleration_limit)

    # --- 2. Calculate maximum permissible acceleration (a_accel_max) ---
    
    # CASE A: Velocity exceeds the tolerance band
    if velocity > (velocity_limit + velocity_limit_tolerance):
        # Force maximum reduction in acceleration (initiate braking)
        a_accel_max = abs_min_a
        a_decel_max = abs_min_a

        return float(a_decel_max), float(a_accel_max)


    # CASE B: Velocity is within the tolerance band [limit, limit + tolerance]
    elif velocity >= (velocity_limit - velocity_limit_thresh):
        # Control exactly to V_limit, bounded by maximum allowed deceleration
        req_a_max = (velocity_limit-velocity)/internal_dt     
        a_accel_max = min(req_a_max, abs_max_a)
        a_accel_max = max(a_accel_max, abs_min_a)
        

    # CASE C: Velocity is below the limit (Predictive control)
    else:
        # Simulate: If we start reducing acceleration to 0 now,
        # what will our final velocity be?
        predicted_v = velocity
        temp_a = acceleration
        
        # Calculate steps required to bring current acceleration to 0
        steps_to_zero = math.ceil(abs(temp_a / (negativ_jerk_limit * internal_dt)))
        
        for _ in range(steps_to_zero):
            # Gradually reduce acceleration towards 0
            temp_a = max(0.0, temp_a + (negativ_jerk_limit * internal_dt))
            predicted_v += temp_a * internal_dt

        # If the prediction shows we will exceed the threshold:
        if predicted_v > (velocity_limit - velocity_limit_thresh):
            a_accel_max = abs_min_a
        else:
            a_accel_max = abs_max_a

    # --- 3. Calculate minimum permissible acceleration (a_decel_max) ---
    # Goal: Prevent the vehicle from driving backward (v < 0)

    if velocity <= (velocity_limit_thresh):
        # Control exactly to 0, bounded by maximum allowed deceleration
        req_a_min = (0.0 - velocity) / internal_dt
        a_decel_max = min(req_a_min, abs_max_a)
        a_decel_max = max(a_decel_max, abs_min_a)
    

    else:
        predicted_v_min = velocity
        temp_a_min = acceleration
        
        # Calculate steps required to bring negative acceleration (braking) back to 0
        steps_to_zero_dec = math.ceil(abs(temp_a_min / (positiv_jerk_limit * internal_dt)))
        
        for _ in range(steps_to_zero_dec):
            # Gradually increase negative acceleration towards 0
            temp_a_min = min(0.0, temp_a_min + (positiv_jerk_limit * internal_dt))
            predicted_v_min += temp_a_min * internal_dt 

        # If we risk dropping below the zero-velocity threshold:
        if predicted_v_min < velocity_limit_thresh:
            a_decel_max = abs_max_a 
        else:
            a_decel_max = abs_min_a

    # --- 4. Return as [Min, Max] ---
    # Safety check: Min must never be greater than Max
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
