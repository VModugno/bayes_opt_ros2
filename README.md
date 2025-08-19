BAYES_OPT_ROS2 — QUICK GUIDE (TXT)

ROS 2 node that performs Bayesian Optimization over an N-dimensional parameter space.
It publishes parameter suggestions for a real experiment, then waits for (1) the measured
result and (2) an allow/continue flag before stepping.

-------------------------------------------------------------------------------
REQUIREMENTS
-------------------------------------------------------------------------------
- ROS 2 (rclpy) — e.g., Humble / Iron / Jazzy
- Python 3.8+
- ROS messages: std_msgs
- PyPI package: bayesian-optimization
  Install in the same environment used by ROS 2:
    pip install bayesian-optimization

-------------------------------------------------------------------------------
PACKAGE LAYOUT
-------------------------------------------------------------------------------
bayes_opt_ros2/
├─ CMakeLists.txt
├─ package.xml
└─ scripts/
   └─ ros2_bayes_opt_node.py

Ensure the node is executable:
  chmod +x scripts/ros2_bayes_opt_node.py

-------------------------------------------------------------------------------
INSTALLATION (BUILD)
-------------------------------------------------------------------------------
From your colcon workspace root (e.g., ~/ros2_ws):
  colcon build --packages-select bayes_opt_ros2

Then source the overlay:
  Linux/macOS (Bash):  source install/setup.bash
  Linux/macOS (Zsh):   source install/setup.zsh
  Windows PowerShell:  .\install\setup.ps1

-------------------------------------------------------------------------------
RUN
-------------------------------------------------------------------------------
Minimal run (uses defaults inside the node):
  ros2 run bayes_opt_ros2 ros2_bayes_opt_node.py

Run with parameters (recommended):
  ros2 run bayes_opt_ros2 ros2_bayes_opt_node.py --ros-args --params-file config.yaml

-------------------------------------------------------------------------------
CONFIGURATION (PARAMETERS)
-------------------------------------------------------------------------------
Create a file named: config.yaml

Contents:
  bayes_opt_node:
    ros__parameters:
      # Search space (any length; order defines vector order on the output topic)
      param_names: ["x1", "x2"]
      lower_bounds: [-2.0, -2.0]
      upper_bounds: [10.0, 10.0]

      # Optimization controls
      n_iter: 15
      init_points: 3
      acquisition_type: "ucb"   # ucb | ei | poi
      kappa: 0.1                # for UCB
      xi: 0.01                  # for EI/POI
      random_state: 987234
      objective_mode: "max"     # or "min"
      timeout_sec: 0.0          # 0 disables timeout

      # Topics (override if desired)
      topics.params_out: "/bayes_opt/params_to_test"
      topics.result_in: "/bayes_opt/experiment_result"
      topics.allow_in: "/bayes_opt/allow_step"
      topics.status: "/bayes_opt/status"
      topics.best_params: "/bayes_opt/best_params"
      topics.best_value: "/bayes_opt/best_value"

-------------------------------------------------------------------------------
PARAMETERS REFERENCE
-------------------------------------------------------------------------------
param_names        (list<string>)  Default ["x"]
  Ordered parameter names. Determines vector order in suggestions.

lower_bounds       (list<float>)   Default [-2.0]
  Lower bounds per parameter. Length must equal param_names.

upper_bounds       (list<float>)   Default [10.0]
  Upper bounds per parameter. Length must equal param_names.

n_iter             (int)           Default 10
  Number of Bayesian optimization iterations (after init_points).

init_points        (int)           Default 0
  Random initial samples before BO.

acquisition_type   (string)        Default "ucb"
  One of: ucb, ei, poi.

kappa              (float)         Default 2.576
  UCB exploration factor (larger = more exploration).

xi                 (float)         Default 0.0
  EI/POI exploration parameter (higher promotes exploration).

random_state       (int)           Default 12345
  RNG seed for reproducibility.

objective_mode     (string)        Default "max"
  "max" to maximize, "min" to minimize (handled internally).

timeout_sec        (float)         Default 0.0
  If > 0, max seconds to wait for BOTH result and allow flag per step.

topics.params_out  (string)        Default "/bayes_opt/params_to_test"
  Where suggested vectors are published.

topics.result_in   (string)        Default "/bayes_opt/experiment_result"
  Experiment’s measured objective (std_msgs/Float64).

topics.allow_in    (string)        Default "/bayes_opt/allow_step"
  Boolean gate to advance optimization (std_msgs/Bool).

topics.status      (string)        Default "/bayes_opt/status"
  Human-readable status updates (std_msgs/String).

topics.best_params (string)        Default "/bayes_opt/best_params"
  Best-so-far parameter vector.

topics.best_value  (string)        Default "/bayes_opt/best_value"
  Best-so-far objective value (human sign).

-------------------------------------------------------------------------------
TOPICS
-------------------------------------------------------------------------------
Publishes
- /bayes_opt/params_to_test  (std_msgs/Float64MultiArray)
  Suggested next parameters; order = param_names.

- /bayes_opt/status  (std_msgs/String)
  Progress logs and summary events.

- /bayes_opt/best_params  (std_msgs/Float64MultiArray)
  Best-so-far vector after completion.

- /bayes_opt/best_value  (std_msgs/Float64)
  Best-so-far objective value (with original sign if minimizing).

Subscribes
- /bayes_opt/experiment_result  (std_msgs/Float64)
  Measured objective for the last suggested parameters.

- /bayes_opt/allow_step  (std_msgs/Bool)
  Set true to allow the optimizer to proceed to the next suggestion.

-------------------------------------------------------------------------------
INTERACTION FLOW (ASCII)
-------------------------------------------------------------------------------
[Optimizer Node] --(Float64MultiArray: params)--> [Your Experiment]
[Your Experiment] --(Float64: result)-----------> [Optimizer Node]
[Your Experiment] --(Bool: allow_step=true)-----> [Optimizer Node]
                                     (then optimizer computes next suggestion)

The optimizer blocks per iteration until it has received BOTH the numeric
result and allow_step=true.

-------------------------------------------------------------------------------
QUICK WIRING TEST (NO REAL EXPERIMENT)
-------------------------------------------------------------------------------
Start the optimizer:
  ros2 run bayes_opt_ros2 ros2_bayes_opt_node.py --ros-args --params-file config.yaml

Watch suggestions:
  ros2 topic echo /bayes_opt/params_to_test

When a vector arrives, publish a dummy result and allow:
  ros2 topic pub /bayes_opt/experiment_result std_msgs/Float64 "{data: 1.23}" --once
  ros2 topic pub /bayes_opt/allow_step       std_msgs/Bool    "{data: true}"  --once

The node will step, produce the next vector, and repeat until n_iter is reached.

-------------------------------------------------------------------------------
MINIMIZATION VS. MAXIMIZATION
-------------------------------------------------------------------------------
To minimize (e.g., error), set:
  objective_mode: "min"

The node internally flips the sign when interacting with the optimizer, while
published values remain human-readable (not flipped).

-------------------------------------------------------------------------------
ACQUISITION OPTIONS
-------------------------------------------------------------------------------
- ucb : Upper Confidence Bound, controlled by kappa (higher = more exploration).
- ei  : Expected Improvement, exploration via xi.
- poi : Probability of Improvement, exploration via xi.

-------------------------------------------------------------------------------
TROUBLESHOOTING
-------------------------------------------------------------------------------
- No next suggestion appears:
  Ensure both /bayes_opt/experiment_result and /bayes_opt/allow_step were
  published after the last suggestion.

- Dimension mismatch:
  param_names, lower_bounds, and upper_bounds must all have the same length.

- Timeouts:
  Set timeout_sec to a positive value to avoid waiting forever; on timeout, a
  very poor placeholder value is returned to discourage re-sampling.

- Dependency missing:
  Install bayesian-optimization in the same Python env used by ROS 2:
    pip install bayesian-optimization

-------------------------------------------------------------------------------
LICENSE
-------------------------------------------------------------------------------
MIT (or adapt to your project’s license).

-------------------------------------------------------------------------------
ACKNOWLEDGMENTS
-------------------------------------------------------------------------------
Uses the excellent 'bayesian-optimization' Python package for BO internals.
