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
