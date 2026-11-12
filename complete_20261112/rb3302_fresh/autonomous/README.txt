RB3302 Autonomous — independent Nav2 implementation

Purpose
  Navigate four ordered letter goals in the known course, hold still for at
  least three seconds at each, avoid visible obstacles, and finish within
  180 seconds. Start bearings: N, NE, E, SE, S, SW, W, NW.

Architecture
  Hardware / Gazebo -> wheel velocity + IMU -> robot_localization EKF -> odom
  Scan + saved map + odom -> STOCK Nav2 AMCL -> map pose
  Static map + live laser obstacles -> global/local costmaps
  NavFn A* -> RPP -> Nav2 velocity smoother -> freshness gate -> cmd_vel
  A small mission node sends NavigateToPose actions and verifies each stop.
  At startup, three raw scans are matched to the map in the known starting
  cell: search +/-20 cm in x/y and +/-45 degrees around the supplied bearing,
  then refine the best match. The worst 20% of endpoint residuals are trimmed
  so an unknown obstacle does not dominate initialization. This initializes
  stock AMCL, which performs all subsequent localization. No truth is used.

  map.pgm preserves the original occupied geometry, resampled by a factor of
  two with nearest-neighbour sampling. This reduces numerical quantization;
  it does not add measured map detail or claim greater survey accuracy.

  A* operates on the 2.5 cm costmap. Only requested letter goals are prescribed
  centres; the planner chooses intermediate paths. Costs encourage clearance
  from walls. Obstacles are represented by their measured surfaces, marked
  and ray-cleared by Nav2. A scan preprocessor excludes hits within 10 cm
  (plus map discretization) of known walls from NEW obstacle marking. Raw
  scans still drive AMCL and clearing; static walls remain in both costmaps.
  This suppresses duplicated walls from localization/map mismatch. It can
  miss a new object very close to an existing wall; that is a limitation.
  There is no assumed single-obstacle memory and no
  forced 90-degree turn program. The navigator recomputes its path at 2 Hz.

  The goal checker accepts any final heading (yaw tolerance pi), avoiding
  unnecessary final orientation.
  Approach tolerance: 2.5 cm estimated position. After navigation succeeds,
  the mission stops at least 0.8 s and waits for a newer AMCL estimate and
  low measured motion. Position within 5 cm starts the 3 s dwell; otherwise
  the same goal is submitted again. No ground truth is available to navigation.

  Motion: nominal 0.20 m/s, angular output bounded to 0.60 rad/s, smoothed
  linear acceleration 0.30 m/s² and deceleration 0.50 m/s². RPP may rotate
  at the start of a path and slows on curves and on goal approach.
  Local collision checks use the 20x20 cm square body plus 1 cm padding.
  Global planning uses a 15 cm enclosing circle plus 1 cm padding.
  Both use a 45 cm SOFT inflation field (not a 45 cm forbidden radius).

  Motion gate stops when odometry/scan are older than 0.5 s, localization
  older than 1 s, command/mission heartbeat older than 0.3 s, or reported
  AMCL position sigma exceeds 10 cm / yaw sigma exceeds 20 degrees.
  AMCL no-motion updates continue during stops. Gate recovery is automatic.
  Failed navigation actions retry after 0.5 s; the overall 180 s deadline
  bounds all retries. No blind spin or reverse recovery is configured.

Real hardware interface
  Existing sensor drivers and serial feedback decoding are reused unchanged.
  base_driver.py subclasses that ROS driver to send the documented T=13
  X (m/s), Z (rad/s) for every motion, including pure rotation. It does NOT
  change ESP32 firmware, calibration or use the old minimum wheel-speed boost.
  Firmware source: https://www.waveshare.com/wiki/Chassis_Control_Tutorial
  The actual onboard firmware response and physical stopping distance still
  require measurement. No hardware success is claimed from simulation.

  Simulation uses the same EKF, stock AMCL, planner, controller, smoother,
  mission and motion gate. Only hardware sources/sinks are replaced.

Run locally in Gazebo (requires ROS Jazzy, ros-jazzy-ros-gz and python3-pil)
  bash ~/rb3302_fresh/autonomous/run.sh sim X N X,O,Q,X
  SIM_OBSTACLE=Q bash ~/rb3302_fresh/autonomous/run.sh sim X N X,P,G,X
  SIM_HEADLESS=1 NAV_RVIZ=0 bash ~/rb3302_fresh/autonomous/run.sh sim X N X,O,Q,X
  Ctrl+C ends this launch. Default simulation domain is 91, localhost only.

Transfer once from the laptop
  scp ~/rb3302_fresh/rb3302_autonomous.tar.gz ubuntu@192.168.50.7:~/

Connect separately
  ssh ubuntu@192.168.50.7

First setup on robot (source-only bundle, build hardware interfaces once)
  tar -xzf ~/rb3302_autonomous.tar.gz -C ~
  bash ~/rb3302_fresh/autonomous/setup.sh
  Subsequent changes to autonomous Python/config files do not require colcon.

Run on robot (after first setup/build)
  bash ~/rb3302_fresh/autonomous/run.sh robot X N X,O,Q,X
  Default domain is 7. Do not simultaneously run another navigation stack.
  The script shares the existing robot launch lock. Ctrl+C stops the launch;
  command watchdogs stop the vehicle if control processes terminate.

RViz on laptop for real robot
  bash ~/rb3302_fresh/autonomous/rviz.sh
  Shows map, laser, AMCL particles, global/local costmaps and current plan.

Reproduce tests on laptop
  bash ~/rb3302_fresh/autonomous/test.sh
  Six actual Gazebo runs: baseline, two blocked passages, diagonal/offset/noisy
  start, a different corner start, and a smaller obstacle. Results are under
  autonomous/evidence/YYYYMMDD_HHMMSS. Default is one simulator at a time.
  This does not contact the physical robot.

Evidence and limitations
  Evaluator alone receives /sim/ground_truth. It records geometric body/wall
  overlap, actual centre errors, 3-second stationary holds, mission duration,
  actual paths, laser marking in the costmap, and ROS endpoint topology.
  Passing means: four goals in order, <=180 simulated seconds, no sampled
  overlap, each goal centre under the simulated body, and >=3-second holds
  (2.9 s recording allowance), with <2 cm movement during those holds.
  Actual goal error is reported separately; centre coverage is not a claim
  that an entire printed label of unknown size is covered.

  The world uses physical collision walls derived from the supplied map,
  simulated lidar/IMU and a four-wheel vehicle. Noise/initial offsets are
  tested. Geometry matching is more ideal than real mapping; traction,
  battery, firmware latency and range biases are not calibrated to the robot.
  The 30/40 cm obstacle boxes are assumptions, not measured official objects.
  Finite simulation passes do not guarantee every route or physical success.

Files
  nav2.yaml              all navigation/localization configuration
  navigate.xml           replan + follow behavior tree, no spin recovery
  mission.py             ordered goals, initial pose, stop verification
  initial_alignment.py   bounded startup scan-to-map matching
  motion_gate.py         final freshness/uncertainty gate
  obstacle_scan.py       known-wall separation for obstacle marking
  robot.urdf             20 cm four-wheel visualization and sensor TF
  base_driver.py         SI velocity protocol adapter for the real base
  sim_adapter.py         simulated hardware messages only
  navigation.launch.py   same stack for hardware and simulator
  tests/                 read-only evaluator and reproducible test runner

All prior clean/ and legacy code remains intact. This version does not import
old mission, planner or custom AMCL motion-model code.

Tested library version: Nav2 1.3.13 on ROS Jazzy.

Transport and test execution
  Gazebo sensor bridges use sensor-data QoS and depth one; command queues
  in the final gate and simulated adapter keep only the latest command.
  This avoids replaying a backlog after transient processing delays. It
  does not guarantee stopping during a stalled simulator/transport process.
  Run acceptance tests sequentially (the default), and do not suspend the
  computer while testing. A prior concurrent run had a 4.7-second AMCL gap
  and continued simulated movement despite ROS zero commands; the failure
  is retained in evidence/20261103_022817. The cause of the whole transport
  stall is not conclusively established. Baseline re-run with one simulator
  passed before changing navigation or planner settings (61.1 seconds).
