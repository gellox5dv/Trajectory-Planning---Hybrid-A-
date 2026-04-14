import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Dict

def bicycle_step(x: float, y: float, theta: float,
                 steer: float, dist: float, wheelbase: float
                 ) -> Tuple[float, float, float]:
    """
    Propagate (x, y, θ) by arc-length `dist` with steering angle `steer`.
    Positive dist = forward, negative = reverse.
    """
    if abs(steer) < 1e-6:
        # Straight line
        nx = x + dist * math.cos(theta)
        ny = y + dist * math.sin(theta)
        ntheta = theta
    else:
        radius = wheelbase / math.tan(steer)
        dtheta = dist / radius
        nx = x + radius * (math.sin(theta + dtheta) - math.sin(theta))
        ny = y - radius * (math.cos(theta + dtheta) - math.cos(theta))
        ntheta = theta + dtheta
    return nx, ny, ntheta % (2 * math.pi)