# Hybrid A* Planning Framework – System Overview

## Overview

This project implements a modular trajectory planning system based on **Hybrid A***.
The architecture is designed for **team development**, with clearly separated responsibilities:

* Simulation / Environment (Saad)
* Motion & Prediction (Angelo)
* Planning (Hybrid A*) (Daniel)
* Visualization (Diana)
* Control (MPC)

---

# Folder Structure

```text
project/
│
├── main.py                     # entry point
│
├── models/                     # shared data structures
│   └── models.py
│
├── simulation/                 # simulation & environment
│   ├── interface.py
│   ├── simulation.py
│   └── collision.py
│
├── motion/                     # vehicle models
│   ├── kinematic.py
│   └── dynamic.py
│
├── prediction/                 # object prediction
│   └── predictor.py
│
├── planner/                    # hybrid A* implementation
│   ├── planner.py
│   ├── node.py
│   └── cost.py
│
├── control/                    # tracking / MPC
│   └── controller.py
│
├── visualization/              # visualization tools
│   └── viz.py
│
└── utils/                      # helper functions (math, time, etc.)
    └── utils.py

```

---

# Data Models (`models/`)

**Responsible: Team (shared)**

Defined in `models.py`.
Contain all shared data structures used across modules.

---

# Simulation Interface (`simulation/`)

**Responsible: Saad (Simulation / Environment)**

Defines how the planner interacts with the simulation.

```python
class SimulationInterface:

    def get_ego_state(self) -> EgoState:
        """Returns the current ego vehicle state."""
        pass

    def get_environment(self) -> Environment:
        """Returns the current environment representation."""
        pass

    def apply_control(self, control: EgoInput) -> None:
        """Applies a control command to the ego vehicle."""
        pass

    def step(self, dt: int) -> None:
        """Advances the simulation by one time step."""
        pass
```

---

# Motion & Prediction Models

**Responsible: Angelo (Motion & Prediction)**

## Kinematic Model (Planner)

```python
def simulate_kinematic(
    state: EgoState,
    control: EgoInput,
    params: VehicleParameters,
    dt: int
) -> EgoState:
    """
    Computes the next state based on a nonlinear bicycle model.

    Used for fast state propagation inside the planner.
    """
    pass
```

---

## Dynamic Model (Tracking)

```python
class DynamicBicycleModel:

    def __init__(self, params: VehicleParameters) -> None:
        """Initializes the model with vehicle parameters."""
        pass

    def step(
        self,
        state: DynamicState,
        control: EgoInput,
        dt: float
    ) -> DynamicState:
        """
        Computes the next state using a dynamic vehicle model.

        Used for accurate tracking and control.
        """
        pass
```

---

## Prediction

```python
def predict_constant_velocity(
    obj: DynamicObject,
    horizon: int,
    dt: int
) -> list[DynamicObjectStamped]:
    """
    Predicts future object states assuming constant velocity.
    """
    pass
```

```python
def predict_constant_acceleration(
    obj: DynamicObject,
    horizon: int,
    dt: int
) -> list[DynamicObjectStamped]:
    """
    Predicts future object states assuming constant acceleration.
    """
    pass
```

```python
def predict_environment(
    objects: list[DynamicObject],
    lanes: list[Lane],
    horizon: int,
    dt: int
) -> Environment:
    """
    Generates a predicted environment over the planning horizon.
    """
    pass
```

---

# Planner

**Responsible: Daniel (Hybrid A*)**

```python
def plan(request: PlanningRequest) -> PlanResult:
    """
    Computes a trajectory using Hybrid A*.

    Takes the current planning request and returns a planning result.
    """
    pass
```

---

# Collision Checking

**Responsible: Saad (Simulation / Environment)**

```python
def check_collision(
    ego: EgoState,
    obj: DynamicObject
) -> bool:
    """
    Checks whether the ego vehicle collides with a given object.
    """
    pass
```

```python
def is_collision_free(
    state: EgoState,
    env: Environment
) -> bool:
    """
    Checks whether a state is collision-free within the environment.
    """
    pass
```

---

# Control (MPC)

**Responsible: (TBD)**

Not defined yet. Responsible for tracking the planned trajectory.

---

# Main Loop

**Responsible: Integration (all modules)**

```python
while True:

    ego: EgoState = sim.get_ego_state()         # current ego state
    env: Environment = sim.get_environment()    # current environment

    result: PlanResult = plan(request)          # planning step

    control: EgoInput = compute_control(
        result.trajectory,
        current_state
    )

    sim.apply_control(control)                  # apply control
    sim.step(dt)                               # advance simulation
```

---

# Summary

* Planner computes trajectories based on predicted environment
* Motion models propagate vehicle states
* Prediction estimates future object motion
* Simulation provides system state and executes control
* Control module tracks the planned trajectory

---
