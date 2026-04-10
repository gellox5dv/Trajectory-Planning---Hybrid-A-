# Module Responsibilities & Functionss

## Simulation Module

### Responsibility

* Owns time and execution loop
* Coordinates all modules

### Functions

* initialize(initial_state, planner, motion_model, visualizer, config)
* run(max_steps) -> SimulationResult
* step_once() -> SimulationSnapshot
* set_goal(goal_state)
* reset(initial_state)

---

## Motion Module

### Responsibility

* Applies vehicle dynamics
* Deterministic state transitions
* Validates motion against environment constraints

### Functions

* step(state, command, vehicle_limits) -> State
* rollout(initial_state, commands, vehicle_limits) -> Trajectory
* is_state_feasible(state, environment_model, vehicle_limits) -> bool

---

## Environment Module

### Responsibility

* Represents the world (maps, lanes, obstacles, etc.)
* Provides query-based access to environment information
* Acts as the single source of truth for spatial validity

### Functions

* is_state_valid(state) -> bool
* is_collision(state) -> bool
* update_dynamic_objects(objects) -> None

---

## Planner Module (Hybrid A*)

### Responsibility

* Only decision-making
* Generates feasible trajectories

### Functions

* plan(plan_request) -> PlanResult

---

## Visualization Module

### Responsibility

* Read-only observer
* Displays simulation and planning outputs

### Functions

* on_simulation_start(config, environment_model)
* on_simulation_end(summary)

---

## Important Points

1. Simulation owns the main loop
2. Motion is deterministic
3. Visualization is read-only
4. Only shared contracts are used
5. Each module provides a minimal stub implementation
    * Planner: return straight-line trajectory
    * Motion: simple kinematic model
    * Visualization: logging only
    * Environment: simple static map with no obstacles

---