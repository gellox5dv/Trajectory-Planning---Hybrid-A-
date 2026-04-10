# Data Contracts

These are the core data structures ('contracts') to be used and shared among modules.

## Core Data Structures

### State

Represents the full vehicle state at a given time.

Fields:

* t: time (seconds)
* x, y: position (meters)
* v: velocity (m/s)
* theta: heading (radians)
* steer: steering angle (radians)

---

### DynamicObject

Represents a moving entity in the environment (e.g., vehicles, pedestrians, cyclists).

Fields:

* id: unique identifier
* state: current State
* type: object type (e.g., 'vehicle', 'pedestrian')
* shape: geometric representation (e.g., bounding box dimensions)

---

### ControlCommand

Represents a single control input applied to the vehicle.

Fields:

* steering_angle (radians)
* acceleration (m/s²)
* dt (seconds)

---

### VehicleLimits

Defines physical and safety constraints of the vehicle.

Fields:

* wheelbase (meters)
* max_steering_angle (radians)
* min_accel, max_accel (m/s²)
* min_speed, max_speed (m/s)

---

### PlanRequest

Input to the planner.

Fields:

* start_state
* goal_state
* vehicle_limits
* horizon_s (planning horizon)

---

### PlanResult

Output of the planner.

Fields:

* success (bool)
* trajectory
* cost
* status_message

---

### Trajectory

Represents a sequence of states and controls.

Fields:

* ordered states
* controls
* timestamps

---

### SimulationSnapshot

Represents the system state at a given simulation step.

Fields:

* tick
* sim_time
* ego_state
* active_plan
* diagnostics

---