from models.types import VehicleParameters
import numpy as np
import math

def get_max_steering_angle(v: float, max_a_lat: float, params: VehicleParameters) -> float:
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

    Raises
    ------
    ValueError
        If velocity v is zero (division by zero).
    """
    # Prevent division by zero
    if v == 0:
        raise ValueError("Velocity must be non-zero.")
    
    # Compute steering angle using the rearranged bicycle model formula
    res =  np.arctan((max_a_lat*params.L)/np.square(v))
    return min(res, params.max_steer)



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

    res = get_max_steering_angle(3, 3, vehicle)
    print(math.degrees(res))


if __name__ == "__main__":
    main()
