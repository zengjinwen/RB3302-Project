# RB3302 — Evidence-based project memory and presentation/Q&A handbook

Prepared on 2026-11-12. Explanations and speaking lines are in English. The ten requested section headings are retained in Chinese for navigation. This is a revision handbook, not a five-page submission or a claim that every question has an experimentally established answer.

## 1 一页总览

### Read this before using any numbers

**The report baseline is `SUCCESS_RPP_20261111_173648`, not the current working directory.** It uses original successful motion speeds, startup convergence, quarter-circle RPP, and the original obstacle thresholds. The user subsequently reported “成功了” after requesting another successful-version run, but no complete new log was attached to that specific confirmation. The snapshot's own `VERSION.txt` predates that confirmation and explicitly says its two bundled logs are earlier trials. [代码/计算检查] (S/VERSION.txt; 实机验证: conversation, post-freeze success confirmation, no separate log; exact run time and errors 未验证)

**November 12 is a separate experiment:** all motion requests ×1.2; departure alignment also applies when the first action is RPP; later obstacle width 0.30→0.20 m and confirmation 0.5→0.3 s with minimum frames 4→3. The supplied November 12 log demonstrates the faster version missed the obstacle reported at L. The relaxed detector has only code-level checking, not a successful attached robot run. [实机验证] (E-L; 代码/计算检查: W/corner_rpp.py, W/obstacle.py and config/control.yaml)

**Speak:** “Our report describes the frozen successful version; later speed and detection changes remain separate experiments.”

### Evidence convention and source keys

- `[实机验证]` means an actual robot log, video, or explicitly identified operator observation. A log verifies estimated values and software events, not independent physical accuracy. `[实机验证] (user observation)` is qualitative, not a ruler measurement.
- `[代码/计算检查]` means source/configuration inspection, arithmetic, archived development records, software tests, or Gazebo. Gazebo is always explicitly called simulation, never robot validation.
- `[推测]` means a possible explanation or proposed benefit without isolation of cause. “未验证” means unavailable/unverified. Historical records saying a change passed are records of a test, not new independent replication.
- Evidence at the end of a paragraph/table row covers that paragraph/row and its associated **Speak** sentence. The short speaking sentence deliberately simplifies the preceding explanation; retain the qualifications when answering follow-up questions.
- Dates below come from desktop development records/backup names. Robot epoch timestamps are used for within-run differences only: their calendar dates conflict with the desktop chronology. Missing dates are not invented. [代码/计算检查] (H and raw attachments)

| Key | Exact source location / meaning |
|---|---|
| P | `/home/rb3302-7/rb3302_fresh` — active project root |
| S | `/home/rb3302-7/rb3302_fresh/backups/SUCCESS_RPP_20261111_173648` — frozen report baseline |
| W | `P/student_ws/src/rb3302_mission/rb3302_mission` — active Python module directory; baseline equivalent is `S/student_ws/src/rb3302_mission/rb3302_mission` |
| C | `S/student_ws/src/rb3302_mission/config` — baseline control/localization/course YAML |
| H | `/home/rb3302-7/rb3302_fresh/report/DEVELOPMENT_HISTORY.txt` — dated reconstruction, not a controlled experiment |
| E | `/home/rb3302-7/.codex/attachments/<UUID>/pasted-text.txt` — raw user terminal extracts; UUIDs below resolve this template |
| E-base | `P/report/logs/BASELINE_SUCCESS_LOG.txt`; same supplied run as E `d321b4b0-f509-4acf-ae5d-6169f27ae2d2` |
| E-start | `P/report/logs/STARTUP_FEEDBACK_RUN_LOG.txt`; E `941764cd-7237-407f-bf0e-66c0304ceaf9` |
| E-L | E `f80e2997-437d-470e-bbdc-cff0ac214462` — Nov 12, faster version, obstacle at L reported missed |
| V | `/home/rb3302-7/Downloads/IMG_7340.mp4` — 3980 frames, 29.9774039669 fps, duration 132.7666667 s |
| G | `P/report/historical_evidence` — archived simulations/reproductions |
| A | `/home/rb3302-7/Z/RB3302-antonio` — comparison repository, not our active runtime |
| M | `/home/rb3302-7/Z/RB3302-merged` — other merged implementation, not our active runtime |
| R | `/home/rb3302-7/RB3302_Project_Report.docx` — report audited in Section 10 |
| T | `/home/rb3302-7/RB3302_Project_Slides.pptx` — 37-slide deck audited in Section 10 |
| D | `/home/rb3302-7/RB3302_Project_Slides` — grading and project PDFs |

### The system in one page

The dependency chain is **wheel feedback + IMU → EKF odometry; EKF odometry + LiDAR + static map → AMCL; static map + letter centres → A*; TF pose + planned reference → PID/Stanley or RPP → cmd_vel → rover.** Laser obstacle processing runs alongside control and can feed one blocked cell back to A*. It is not a purely sequential pipeline in which obstacle detection runs after arrival. [代码/计算检查] (localization.launch.py, robot.py, grid.py, corner_rpp.py, experiment_obstacles.py)

| Module | What it does; selection and alternative | English speaking line |
|---|---|---|
| Sensors | Wheel feedback supplies forward motion; IMU supplies yaw/yaw rate; LiDAR supplies scan geometry. Commands alone are not motion measurements. [代码/计算检查] (C/localization.yaml, base_driver.py) | “We measure motion and the surrounding walls instead of assuming commanded motion actually happened.” |
| EKF | Fuses local velocity and inertial orientation/rate into continuous odometry; unlike wheel-only odometry it also uses the IMU. It does not establish global position. [代码/计算检查] (C/localization.yaml) | “The EKF provides continuous local motion estimates, while the map provides the global reference.” |
| AMCL | Matches laser observations to a pre-existing map using particles and odometry. Selected because the map already exists; SLAM and corner matching were not the final online method. [代码/计算检查] (localization.launch.py; H) | “AMCL localises in our existing map, so we do not need to rebuild the map during the task.” |
| Startup | Waits ≥3 s and five fresh concentrated AMCL estimates. Concentration is not proof of correctness. [代码/计算检查] (S/.../corner_rpp.py initialize) | “We wait for a stable initial estimate before allowing the robot to move.” |
| A* | Searches the 24-node, four-neighbour, 0.58 m graph. Avoids fine-grid route complexity; it does not optimise execution time or steering effort. [代码/计算检查] (grid.py) | “A star searches the letter cells, giving us a simple route that matches the arena.” |
| Straight control | Heading PID plus Stanley-style lateral angular correction follows a fixed centreline. Heading-only control cannot eliminate parallel offset. [代码/计算检查] (node.py drive_line) | “Heading control keeps us facing along the corridor, and lateral correction brings us back towards its centre.” |
| RPP | Tracks rounded corner references and final goal-cell approach. Replaces most stop-turn-go steps; pivots are still permitted. Not the Nav2 RPP plugin. [代码/计算检查] (corner_rpp.py, corner_geometry.py) | “RPP follows a curved reference at corners and continues into the goal cell.” |
| Obstacles | Recognises a stable broad near face in an open adjacent cell; remembers one blocked cell and replans only when necessary. Not general collision avoidance. [代码/计算检查] (experiment_obstacles.py, obstacle.py, obstacle_node.py) | “We recognise the visible face of a large block and replan only if it obstructs the remaining route.” |

**Five-minute use:** Suggested rehearsal allocation: overview 35 s; final methods 75 s; V1/V2/V3 plus five challenges 110 s; results/limitations 55 s; contributions/conclusion 25 s. This is a proposed 300 s allocation, not a measured presentation duration. The current 37-slide deck is a study deck; showing all slides would require approximately 8.1 s per slide and is likely too dense. [代码/计算检查] (37-slide count and arithmetic; 推测: pacing suitability)
**Speak:** “We selected representative iterations because the main lesson is how each change addressed a concrete observed problem.”

## 2 最终方案逐模块详解

### 2.1 Runtime, frames, map and physical interpretation

The active successful entry is `run_corner_rpp.sh → run.sh → all.launch.py → mission.launch.py → mission_corner_rpp`. `all.launch.py` starts sensors, robot description, base driver, EKF, map server, AMCL, lifecycle manager and the mission. One terminal does not mean one ROS node. `legacy_base` is the executable name but its active `_send_velocity` override sends SI `T=13, X=v, Z=omega`; an old direct-wheel branch in the parent driver is not the current command path. [代码/计算检查] (S root scripts; launch/all.launch.py; base_driver.py)
**Speak:** “One launcher starts several cooperating nodes, and the mission sends linear and angular velocity commands to the base.”

TF ownership is `map → odom` from AMCL, `odom → base_link` from EKF, fixed sensor transforms from robot description. `straight_sample()` reads the latest `map → base_link` TF and filtered `/odom` yaw rate, while checking current AMCL validity. `pose()` reads the latest AMCL message and is used for obstacle current-cell selection. These are different timestamps and purposes. There is no final 70% odom/30% AMCL position blend. [代码/计算检查] (robot.py, localization.yaml, experiment_obstacles.py)
**Speak:** “Control uses the composed TF pose; the latest AMCL message can be older than that pose.”

Physical body size was supplied as 0.20 × 0.20 m. The URDF uses `chassis_length=0.20`, `chassis_width=0.20` m. Its other inherited dimensions/masses are model values, not a new physical measurement; for example `wheel_separation=0.288 m` should not be presented as a calibrated skid-steer track width. Final LiDAR `lidar_from_imu_x=+0.015 m`, lateral offset 0 m, `imu_mount_z=0.050 m`, `lidar_from_imu_z=-0.040 m`; horizontal base/IMU origins coincide. `laser_yaw=pi rad`; inspect the full transform rather than interpreting an offset without its parent frame. [代码/计算检查] (S/robot_ws/src/robot_model/urdf/diffbot.urdf.xacro and launch/description.launch.py; 实机验证: operator centre feedback after Nov 6 offset trial, E 3a18d084-94cb-4269-841f-436d51ba080e; physical metrology 未验证)
**Speak:** “The controller tracks the body reference, so the laser mounting transform must describe its offset from that reference.”

The fine map is `student_ws/maps/rb3302_refined_58cm.yaml` and `.pgm`: resolution 0.04833333333333334 m/pixel; origin (-0.7008333333333336, -0.8458333333333335, 0 rad); `mode=trinary`, `negate=0`, occupied threshold 0.65 and free threshold 0.196, image 98×80 pixels. It was derived from the cleaned aligned map, scaled by 58/60 with outer-wall adjustments. This is not a newly measured high-resolution survey. [代码/计算检查] (map files, H Nov 5)
**Speak:** “We use a fine occupancy map for geometry and a separate coarse graph for the letter-cell routes.”

### 2.2 EKF

**Input/output:** `robot_localization` EKF receives `/odom/raw` forward velocity only and `/imu_data_ros` yaw plus yaw rate; publishes `/odom` and `odom → base_link`. Raw odometry x/y/yaw are not selected in `odom0_config`. `imu0_relative=true` references the initial IMU orientation. It is not a fixed weighted average and does not use LiDAR directly. [代码/计算检查] (C/localization.yaml)
**Speak:** “The EKF fuses measured forward velocity with IMU heading and yaw rate, not the commanded turn.”

Conceptual EKF equations are `x_minus=f(x,u)`, `P_minus=F P F^T+Q`, `K=P_minus H^T(H P_minus H^T+R)^-1`, `x_plus=x_minus+K(z-h(x_minus))`. They explain nonlinear prediction and covariance-weighted correction; we use the library implementation, not a handwritten EKF. Q/R are uncertainty models, not measured final accuracy. [代码/计算检查] (configured robot_localization implementation; standard mathematical description, not a line-for-line reproduction)
**Speak:** “The filter changes measurement influence according to uncertainty, rather than using a constant seventy-thirty position blend.”

| Parameter / location | Final value | Why / limitation |
|---|---|---|
| C/localization.yaml `ekf_filter_node.frequency` | 50.0 Hz | Requested continuous local output; not measured AMCL/scan/particle frequency. Increased from 30 Hz; optimal CPU setting 未验证. |
| `sensor_timeout` | 0.3 s | EKF sensor-timeout setting; does not mean a mission must exit at 0.3 s. |
| `two_d_mode`, `publish_tf` | true, true | Planar arena and one odometry TF publisher. |
| `world_frame`, `odom_frame`, `base_link_frame`, `map_frame` | odom, odom, base_link, map | Consistent frames; world output remains odom. |
| `odom0_queue_size`, `imu0_queue_size` | 10, 50 messages | Buffer settings retained; no demonstrated optimal queue tuning. |
| `odom0_config` | only index 6 true | x velocity selected; array order is library state order. |
| `imu0_config` | indices 5 and 11 true | yaw and yaw rate selected. |
| base driver `linear_velocity_stddev`, `angular_velocity_stddev` | 0.03 m/s, 0.10 rad/s | Provisional measurement uncertainty; not experimentally calibrated noise standard deviations. |

All rows: [代码/计算检查] (C/localization.yaml; S/robot_ws/src/waverover_base/waverover_base/waverover_driver.py).

**Failure:** slip biases wheel velocity, IMU stream faults corrupt heading propagation, stale or inconsistent TF stops control waiting for usable samples. A smoothly published EKF result can still be wrong. The YESENSE CRC parser defect was reproduced in software, but it was not proven to cause every earlier frozen-heading robot incident. [代码/计算检查] (robot.py; G/imu_stream_recovery_20261106/RESULT.txt; 推测: individual incident cause 未验证)
**Speak:** “A high output rate does not make bad input accurate; we also needed a reliable IMU data stream.”

### 2.3 AMCL

**Input/output and algorithm:** a particle represents `(x,y,yaw)` in the map. The motion update propagates particles using scan-time odometry TF. The likelihood-field sensor model scores scan endpoints by proximity to mapped obstacles. Weighting and resampling favour poses explaining the scan. AMCL supplies a pose/covariance and `map → odom`; it neither plans the route nor directly commands the motors. [代码/计算检查] (C/localization.yaml; official AMCL implementation reference below)
**Speak:** “Particles are alternative pose hypotheses, and the scan tells AMCL which hypotheses fit the known walls.”

A useful sensor-model explanation is `p_hit ∝ exp(-d_wall²/(2 sigma_hit²))`, combined with a random component. This is an explanatory beam likelihood, not a claim that our code multiplies precisely this expression for every beam. The upstream likelihood-field implementation includes its own score aggregation. [代码/计算检查] (official AMCL likelihood-field method; parameter mapping verified locally)
**Speak:** “Scan endpoints close to expected walls support a particle, while mismatched endpoints reduce its plausibility.”

| Parameter in C/localization.yaml (`amcl.ros__parameters`) | Final value and unit | Reason and qualification |
|---|---|---|
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | Official model restored; custom signed model is not loaded. |
| `alpha1`, `alpha2` | 0.2, 0.2 noise coefficients | Rotation uncertainty contributions; adopted from Antonio/merged configuration, not independently identified. |
| `alpha3`, `alpha4` | 0.6, 0.3 noise coefficients | Larger translation uncertainty allows for wheel-slip uncertainty; not a claim to measure slip online. |
| `alpha5` | 0.2 coefficient | Present in config; standard differential model does not use the omnidirectional strafe-noise term. |
| `laser_model_type` | likelihood_field | Matches scan endpoints to static map. |
| `sigma_hit` | 0.08 m | Adopted endpoint-match scale; not an 8 cm position-accuracy guarantee. |
| `z_hit`, `z_rand` | 0.9, 0.1, dimensionless | Match/random mixture settings; not 90% LiDAR + 10% odometry fusion. |
| `max_beams` | 120 beams/update | Subsamples scan processing; not scanner hardware resolution. |
| `min_particles`, `max_particles` | 1500, 2500 particles | Adaptive population bounds; more particles are not automatically more accurate or faster. |
| `update_min_d` | 0.05 m | Motion threshold; not a 20 Hz update rate. |
| `update_min_a` | 0.1 rad = 5.7296° | Angular movement threshold, not the startup covariance threshold. |
| `use_sim_time` | false | Physical runtime uses ROS system time. |
| `base_frame_id`, `odom_frame_id`, `global_frame_id`, `scan_topic` | base_link, odom, map, scan | Match TF and topic names. |

All rows: [代码/计算检查] (C/localization.yaml; A/student_ws/src/planning_rpp/config/nav2_params.yaml; identical AMCL mapping adopted per H). Rationale for precise numeric optima is 未验证; retained values worked in the successful user-reported configuration, without an isolated parameter benchmark.

**Rate and recovery:** moving updates depend on scan arrival, movement thresholds, TF availability and processing. At 0.18 m/s, travelling 0.05 m takes 0.278 s; this is a rough straight-motion threshold interval, not a measured output period. Stationary scans trigger `/request_nomotion_update` only while the last command is zero, with at most one outstanding request. No command pauses AMCL during the 4 s goal hold. `robot.py` still contains an outdated comment saying movement thresholds are zero; the YAML above is authoritative. Recovery capabilities exist in AMCL, but no custom lost-localisation search is integrated. Recovery coefficients are not explicitly set in this final YAML; do not claim automatic global recovery is enabled or tested. [代码/计算检查] (robot.py, C/localization.yaml; arithmetic; official AMCL docs)
**Speak:** “AMCL is scan and motion driven; its particle display rate is not the EKF rate.”

**Failure:** wrong initial pose, repeated arena geometry, incorrect sensor TF, odometry errors, delayed scans or new unmapped objects can produce a biased or ambiguous estimate. The statement “the new obstacle pulled AMCL off” is a hypothesis unless before/after pose and ground truth establish it. [推测] (plausible failure modes; this project's causal isolation 未验证)
**Speak:** “A concentrated particle cloud can still represent the wrong location in a repetitive environment.”

### 2.4 Startup convergence

`Mission.initialize()` waits for map and initial-pose subscribers, rejects multiple publishers on `/cmd_vel`, `/amcl_pose`, `/odom`, publishes the supplied start centre and compass heading, then waits for fresh AMCL. Initial XY variance is `0.2²/3=0.0133333 m²` per axis; yaw variance `(pi/4)²/3=0.205617 rad²`. These are initial uncertainty settings, not measured initial error. Eight compass headings already exist in `robot.py`: E 0°, NE 45°, N 90°, NW 135°, W 180°, SW −135°, S −90°, SE −45°. A* remains four-neighbour. [代码/计算检查] (node.py initialize; robot.py compass)
**Speak:** “The starting compass direction initialises orientation; it does not add diagonal edges to A star.”

`CornerMission.initialize()` holds zero velocity for at least 3 s and counts five distinct newer AMCL timestamps. It uses the largest XY covariance eigenvalue:
`lambda_max=(xx+yy+sqrt((xx-yy)^2+4xy^2))/2`, `sigma_xy=sqrt(max(0,lambda_max))`; `xy=(c[1]+c[6])/2`. Require `sigma_xy≤0.05 m` and `sqrt(c[35])≤5°=0.0872665 rad`. Invalid/stale data resets the count. It does not depart after a deadline. A frozen wrong estimate with small covariance may pass; a valid but diffuse estimate may wait indefinitely. [代码/计算检查] (corner_rpp.py initialize)
**Speak:** “Five fresh low-covariance estimates must pass after the minimum stationary period.”

Frozen baseline aligns before a departing `line` action; an RPP-first action uses its own pursuit-bearing pivot. Nov 12 changed this to `if departure:` for either action, so diagonal starts first face the first cardinal segment after convergence. This is a real change, not something to retroactively claim for every successful baseline departure. [代码/计算检查] (S versus P corner_rpp.py)
**Speak:** “The newer start-direction change explicitly aligns the robot before either kind of first segment.”

### 2.5 58 cm coarse-grid A*

Centres are fixed, not regenerated from the robot pose. `course.yaml` uses x columns `(0.29,0.87,1.45,2.03,2.61,3.19) m`; rows A–F at y=2.03, G–L 1.45, M–R 0.87, S–X 0.29 m. Thus `x=0.29+0.58 c`, `y=0.29+0.58 r`, with bottom row r=0. For alphabetical row index j from top, `y=2.03−0.58 j`. N is +y, E is +x. [代码/计算检查] (C/course.yaml)
**Speak:** “Every letter has a precomputed centre, so tracking errors cannot move the planned reference line.”

`MapPlanner` inflates occupied values ≥50 and unknown values <0 by `stop_distance=0.22 m` using pixel-offset Euclidean distance. It checks centre connections every half pixel, about 0.0241667 m. Only cardinal centres exactly one 0.58 m step apart may connect. A* edge cost is 1 cell step and `h=(abs(dx)+abs(dy))/0.58`; it is admissible for this graph even when walls force detours. One blocked cell is skipped; a blocked start/goal returns no route. The 0.22 m clearance is not the robot physical radius: a 0.20 m square's half-diagonal is about 0.1414 m; extra margin and map discretisation remain design choices, not calibrated clearance guarantees. [代码/计算检查] (grid.py; control.yaml; arithmetic)
**Speak:** “The planner finds a shortest route in cell steps, not necessarily the fastest physical trajectory.”

`turn_windows()` groups each turn with its previous/next centre and merges overlapping windows. Other sections use straight control; the last half-cell switches to RPP if needed. The rounded curve does not pass through the original turning-cell centre: it is tangent to its incoming/outgoing centrelines. Goal centres remain the final targets. [代码/计算检查] (corner_geometry.py and corner_rpp.py run)
**Speak:** “We keep cell topology for planning, but round the motion reference inside turning cells.”

### 2.6 Straight control — exact form and adaptation

Let fixed path heading be `theta_r=atan2(y_end-y_start,x_end-x_start)` and `u=(cos(theta_r),sin(theta_r))`. Let robot TF position be p and heading theta. Then:

```
e_theta = wrap(theta_r - theta)
e_y = -sin(theta_r)*(x-x_start) + cos(theta_r)*(y-y_start)
e_front = e_y - l_front*sin(e_theta)
lateral = -K_s * atan2(k_cross*e_front, v_nominal + v_soft)
r_filtered += dt/(tau_D+dt) * (r_measured-r_filtered)
I_candidate = clip(I + K_i*e_theta*dt, -0.05, +0.05)
omega_request = clip(K_p*e_theta + I - K_d*r_filtered + lateral, -0.50, +0.50)
```

`I` is an angular-rate contribution, with conditional anti-windup. Positive `e_y` means left of the path; negative lateral command turns right toward it. The derivative uses measured filtered yaw rate, not a numerical derivative of a delayed AMCL heading. `atan2` has positive denominator here, so it matches `atan(numerator/denominator)`. [代码/计算检查] (node.py drive_line)
**Speak:** “Our lateral term uses Stanley’s error form, but commands body yaw rate rather than a steering-wheel angle.”

| Parameter in C/control.yaml unless specified | Baseline effective value | Selection reason / caveat |
|---|---|---|
| `drive_speed` | **0.18 m/s**, overridden by `launch/mission.launch.py` for `mission_corner_rpp` | YAML alone says 0.24; quoting only YAML would be wrong. Retained after physical speed trials. |
| `heading_kp` | 1.5 s⁻¹ | Heading error to yaw-rate response; empirical, not optimal-control result. |
| `heading_ki` | 0.03 s⁻² | Small bias correction; contribution clipped ±0.05 rad/s. |
| `heading_kd` | 0.08 dimensionless | Measured yaw-rate damping. |
| `heading_d_filter_time` | 0.20 s | Filters yaw-rate variation. |
| `stanley_angular_gain` | 1.5 s⁻¹ | Converts angular correction form into yaw rate. |
| `stanley_cross_gain` | 0.4 s⁻¹ | Lateral response; launch explicitly restores this value. |
| `stanley_front_offset` | 0.10 m | Virtual point ahead of body, not the physical front-wheel steering axle. |
| `stanley_soft_speed` | 0.10 m/s | Avoids strong low-speed division sensitivity. |
| `drive_turn_limit` | 0.50 rad/s before rate trim | Launch override agrees with YAML. Not the final motor angular command bound. |
| `slowdown_distance` / `approach_speed` | 0.15 m / **0.08 m/s launch override** | Legacy helper parameters; baseline CornerMission calls straight segments with end speed equal to cruise and uses RPP in the goal cell. They do **not** impose final-version goal deceleration. |
| `drive_line dt` minimum | 0.001 s | Numerical protection; loop pause default is 0.02 s, not a proven 50 Hz end-to-end rate. |

All rows: [代码/计算检查] (control.yaml, launch/mission.launch.py, node.py, corner_rpp.py). **Speak:** “The final baseline uses eighteen centimetres per second, with separate heading and lateral correction.”

This is a skid-steer-compatible command interface, not a derived four-wheel tyre model. Equal/opposite left/right wheel velocities are the ideal differential-drive interpretation of a pivot, but the host sends body twist and does not independently close four wheel-speed loops. Floor friction, firmware control and slip can change the real turning centre. [代码/计算检查] (base_driver.py; 推测: unmeasured physical mechanisms)
**Speak:** “We control body velocity; we do not claim to model or independently regulate every wheel’s contact motion.”

### 2.7 RPP geometry and control

For a right-angle vertex b, incoming unit direction u and outgoing unit direction v, `r=min(0.30 m, incoming_length/2, outgoing_length/2)=0.29 m` on normal edges. Circle centre `c=b-r*u+r*v`; path `p(phi)=c+r*(sin(phi)*u-cos(phi)*v)`, `0≤phi≤pi/2`. Entry is `b-r*u`; exit is `b+r*v`. Sampling uses `spacing=0.02 m`, `count=max(8,ceil(pi*r/(2*spacing)))`, giving 23 segments for r=0.29 m. Arc length is about 0.45553 m. Static-clearance rejection retains the original right-angle reference and logs a warning; it does not silently force an unsafe arc. [代码/计算检查] (corner_geometry.py smooth_corners; arithmetic)
**Speak:** “A tangent quarter circle replaces the sharp corner, with a twenty-nine-centimetre radius on our grid.”

The closest path projection gives progress s. The carrot is at `s+lookahead`, with `lookahead=min(0.20 m,cell_size/2)=0.20 m`. If beyond the endpoint, it is extended along the final centreline to prevent carrot collapse; the stopping target does not move. For target in body coordinates `(x_L,y_L)`, `kappa=2*y_L/(x_L²+y_L²)` in m⁻¹ and `omega=v*kappa` in rad/s. [代码/计算检查] (corner_geometry.py project/point_at/pursuit; corner_rpp.py)
**Speak:** “The lookahead point continues beyond the endpoint geometrically, but stopping still uses the original goal plane.”

Baseline speed regulation is `v=min(v_nominal, sqrt(a_lat/abs(kappa)), turn_speed/abs(kappa))`, `a_lat=v_nominal²/(0.58/2)=0.111724 m/s²`, `v_nominal=0.18 m/s`, `turn_speed=1.80 rad/s`. This is a design acceleration limit, not a measured grip limit. No later cross-error/cosine speed multiplier remains. If target-bearing magnitude exceeds 45°, translation is zero and requested turn is `sign(error)*1.8*min(1,abs(error)/(pi/2)) rad/s`. Pivoting is therefore still possible. [代码/计算检查] (corner_rpp.py follow_corner)
**Speak:** “Curvature reduces forward speed on tight tracking corrections, and large bearing errors may still require a pivot.”

### 2.8 Measured-rate trim and departure pivot — present even if omitted from slides

`RateTrim` applies `u=clip(w_ref+clip(0.25*(w_ref-w_meas)+I, ±0.5*abs(w_ref)), ±2.0)` rad/s. Integration uses gain 0.25 and integral time 1 s, with conditional anti-windup; sample dt capped at 0.2 s. It resets on zero request, sign change or stale feedback. `/odom` child frame must be `base_link`; acceptable feedback age is −0.1≤age<0.5 s. This uses EKF yaw rate, not four independent encoders. Original rate limits and gains are provisional design settings, not an identified motor model. [代码/计算检查] (turn_feedback.py RateTrim/command)
**Speak:** “A small measured yaw-rate correction helps track the requested turn, but it is not a calibrated wheel model.”

Absolute departure alignment uses TF heading, `omega=clip(1.5*heading_error, ±goal_turn_speed)` with `goal_turn_speed=0.8 rad/s`, stopping within 3°=0.05236 rad. There is no active 15° early stop in this helper. `turn_early_stop_degrees=15` remains for legacy relative turns; it is not the normal final RPP turn rule. [代码/计算检查] (turn_feedback.py face_heading; CornerMission override; node.py turn_relative)
**Speak:** “The final RPP version uses absolute departure alignment, rather than the old fifteen-degree early-stop turn.”

### 2.9 Arrival and four-second hold

For final tangent u and goal g, remaining distance is `(g-p)·u`. Final RPP returns when the nearest original-grid edge is the final edge and remaining≤0 m. Straight helper also uses projection crossing. This is an endpoint plane, not a circle of zero radius: lateral error can be nonzero at completion. `settle_goal()` calls `halt(4.0)` then logs TF-based Euclidean error if a fresh sample exists; it does not command correction. All four goal entries are processed, including a start letter repeated as the first goal. [代码/计算检查] (corner_rpp.py follow_corner/settle_goal/run; robot.py halt)
**Speak:** “We stop after crossing the goal’s longitudinal plane, then hold for four seconds without repeated corrective movements.”

No exact physical zero-error stop is guaranteed. Sensor latency, discrete control, braking and lateral deviation remain. In one historical diagnostic, TF said the endpoint was crossed while the latest AMCL message lagged by 0.161–0.271 s; at 0.18 m/s this corresponds to roughly 2.9–4.9 cm of travel before other differences. Comparing different-time poses is not a valid independent accuracy test. [实机验证] (E e2db1c39-9098-4abd-b3a9-916f29bd3778 and supplied diagnostic text; 代码/计算检查: arithmetic; 推测: exact share attributable to delay 未验证)
**Speak:** “Zero longitudinal threshold means no deliberate early acceptance; it does not mean zero physical stopping error.”

### 2.10 Obstacle detection — every decision threshold

`ExperimentalObstacleMission.read_scan` takes valid laser ranges, transforms them to `map` at scan time, and chooses the current cell nearest the latest AMCL pose. It considers only statically connected cardinal neighbours. `NewSurfaceObstacle` first removes points near mapped walls, then `CellObstacle` evaluates near faces. It does not require seeing all four sides of a box. [代码/计算检查] (experiment_obstacles.py, obstacle.py)
**Speak:** “We only need a stable visible front face, because the task tells us the obstacle occupies the cell.”

| Test / exact location | Frozen baseline | Nov 12 working trial | Meaning / reason |
|---|---|---|---|
| `experiment_obstacles.py read_scan` scan age | 0≤age<1.0 s | same | Reject old scans; exact scan-time TF required. |
| valid range | `msg.range_min≤range≤msg.range_max`, finite | same | Sensor-published bounds; concrete physical range runtime 未验证. |
| current cell | nearest AMCL cell centre | same | Selects up to four open neighbours; not continuous full-map monitoring. |
| `obstacle.py static_free` | reject any occupied≥50 or unknown<0/out-of-map pixel in ±2 pixels x/y | same | 5×5 neighbourhood; axial margin approximately 9.67 cm, not a precise radial 10 cm circle. |
| `visible_in_map` | sample every ≤resolution/2≈2.4167 cm | same | Reject ray passing through known wall/unknown map. |
| face axis | axis with larger absolute laser-to-cell-centre displacement | same | Select robot-facing cardinal side, not all four sides. |
| near face | centre coordinate ±0.29 m | same | Half-cell boundary. |
| `obstacle_boundary_tolerance` in control.yaml | ±0.20 m | same | Allows map/placement error; not a measured error bound. |
| transverse band, literal in `detect_cells` | ±0.29 m around cell centre | same | Keeps candidate within cell width. |
| depth band, literal | 0.12 m | same | Group broad returns at approximately one depth. |
| point count, literal | ≥5 points | same | Reject insufficient geometric evidence. |
| `obstacle_min_width` | 0.30 m visible span | **0.20 m** | Partial visibility; reduced to address L miss, effectiveness 未验证. |
| centre crossing | strict `min(lateral)<0<max(lateral)` | same | Face must cover both sides of centreline. |
| static-free support, literal | ≥5 points | same | Reject known-wall-supported fits. |
| fitted angle | `abs(slope)≤tan(10°)` | same | Task expects axis-aligned blocks; ≈0.176327 slope. |
| fit residual RMS | ≤0.025 m | same | Reject scattered/nonplanar depth fits. |
| `obstacle_confirm_time` | ≥0.5 s | **≥0.3 s** | Persistence across scans, not a timer that assumes detection. |
| `CellObstacle.observe count` | ≥4 distinct new scans | **≥3** | Repeated timestamp ignored; elapsed time and count both required. |
| continuity gap | >1.0 s resets candidate timing | same | Old evidence cannot bridge a large scan gap. |
| unreachable-candidate check | reject candidate if A* cannot reach active goal with it blocked | same | Arena assumption; dangerous if the only real passage is blocked. |
| stored dynamic obstacles | one cell | same | New confirmed cell replaces old; absence alone does not clear old cell. |

All rows: [代码/计算检查] (S and P versions of obstacle.py, experiment_obstacles.py, control.yaml). None of these geometric thresholds is a proven globally optimal setting. **Speak:** “The thresholds encode a large regular block near a neighbour boundary, not arbitrary obstacle shapes.”

At L in the Nov 12 log, R’s open neighbours included L; remaining non-wall points were 24 then 41, but candidate lists stayed empty. This proves no candidate reached confirmation at those logged instants; it does not identify which fit test failed. Filtering obstacle endpoints near side walls, insufficient visible width, orientation error and projection error are possibilities. The “stable frames” counter also increments for `None`, so a high count does not show a stable obstacle. Once the nearest current cell became L, neighbours were F/R and L itself was no longer tested. [实机验证] (E-L at 1780677843.791, 1780677844.912, 1780677846.032; 代码/计算检查: observe; 推测: exact rejection cause 未验证)
**Speak:** “The missed obstacle never became a candidate, so simply shortening confirmation time cannot guarantee detection.”

### 2.11 Replanning and failure behaviour

The current confirmed cell stays eligible while still visible. Otherwise the nearest eligible observed candidate is considered. A confirmed replacement affects control only if it is in the remaining route and, on the current segment, still ahead. A positive forward command then sends zero and raises `Replan`, caught by the route loop. A* calculates a detour from a safe centre before the blocked cell; the robot finishes the anchor connection, then uses the cached route if the obstacle has not changed. If the anchor lies behind, it turns and drives forward rather than reversing. [代码/计算检查] (experiment_obstacles.py; obstacle_node.py blocks_route/command/replan_at_centre; corner_rpp.py exception handling)
**Speak:** “A blocked route triggers a safe cell-centre connection before we follow the new A star route.”

`replan_at_centre` may hold 0.3 s after anchor arrival. If there is no route, the route loop sends zero and retries every 1.0 s without a mission timeout. However, the earlier unreachable-candidate veto often prevents such a real blockage from being registered at all. Do not promise “it always stops before any blocked only passage.” There is no separate general raw-scan emergency collision layer in this mission. [代码/计算检查] (obstacle_node.py; experiment_obstacles.py; corner_rpp.py)
**Speak:** “Our no-route behaviour waits and retries, but the detector’s reachability assumption is a separate limitation.”

### 2.12 Limits, watches, inactive parameters and reproducibility

`Robot.ready` requires −0.1≤AMCL age<1.0 s, map frame, finite pose and quaternion norm-squared between 0.5 and 1.5. `straight_sample` additionally requires TF and odom ages in the same interval, `base_link` child frame and finite values. Missing data issues zero commands and waits. The driver has `cmd_vel_timeout=0.8 s`, `feedback_timeout=1.0 s`, `heartbeat_rate=20 Hz`, requested `feedback_rate=100 Hz`, `feedback_interval_ms=10`, serial 115200 baud and `serial_timeout=1.0 s`; actual delivered rates must be measured. Driver command caps are 1.0 m/s and 2.0 rad/s. [代码/计算检查] (robot.py; driver source)
**Speak:** “We removed a mission-wide turn deadline, not the freshness checks and motor-command watchdogs.”

Exceptions in `node.main` log an error, send zero in `finally`, destroy the mission node and exit nonzero; Ctrl+C also stops and cleans up. `run.sh` holds a file lock and rejects a second launcher. A duplicate publisher can also reject startup. No claim that the task can never stop or exit is accurate. [代码/计算检查] (node.py main/initialize; run.sh)
**Speak:** “A fault can stop or terminate the mission; removing unnecessary exits did not remove all failure handling.”

Legacy-only YAML values include `arc_radius=0.18 m`, `arc_speed=0.06 m/s`, `arc_cross_gain=2.0 rad/(m·s)`, `arc_heading_gain=3.0 s⁻¹`, `arc_turn_limit=2.0 rad/s`, `arc_exit_heading_degrees=5°`, and `turn_min_speed=0`. They belong to the older `mission_arc` or relative-turn paths, not the final RPP reference radius. Parent driver `turn_wheel_gain=0.50`, `turn_wheel_limit=0.25`, `turn_wheel_min=0.12` are not active T13 calibration constants in `base_driver.py`'s override. [代码/计算检查] (control.yaml, arc_node.py, base_driver.py, driver source)
**Speak:** “A parameter left in the configuration is not necessarily used by the selected executable.”

### 2.13 Exact frozen configuration for another AI

These blocks are copied from the frozen source, not reconstructed from the report. Launch overrides take precedence. Legacy-only parameters remain in YAML; consult the active call path in Sections 2.6–2.12 before using them. [代码/计算检查] (S source files)
**Speak:** “The configuration must be interpreted together with the selected launcher and active methods.”

**Source:** `/home/rb3302-7/rb3302_fresh/backups/SUCCESS_RPP_20261111_173648/student_ws/src/rb3302_mission/config/control.yaml`. [代码/计算检查]

```yaml
rb3302_mission:
  ros__parameters:
    drive_speed: 0.24
    slowdown_distance: 0.15
    approach_speed: 0.07
    turn_speed: 1.80
    turn_early_stop_degrees: 15.0
    turn_min_speed: 0.0
    heading_kp: 1.5
    heading_ki: 0.03
    heading_kd: 0.08
    heading_d_filter_time: 0.20
    stanley_angular_gain: 1.5
    stanley_cross_gain: 0.4
    stanley_front_offset: 0.10
    stanley_soft_speed: 0.10
    drive_turn_limit: 0.50
    stop_distance: 0.22
    arc_radius: 0.18
    arc_speed: 0.06
    arc_cross_gain: 2.0
    arc_heading_gain: 3.0
    arc_turn_limit: 2.0
    arc_exit_heading_degrees: 5.0
    obstacle_confirm_time: 0.5
    obstacle_min_width: 0.30
    obstacle_boundary_tolerance: 0.20
```

**Source:** `/home/rb3302-7/rb3302_fresh/backups/SUCCESS_RPP_20261111_173648/student_ws/src/rb3302_mission/config/localization.yaml`. [代码/计算检查]

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.3
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /odom/raw
    odom0_config: [false, false, false, false, false, false, true, false, false, false, false, false, false, false, false]
    odom0_queue_size: 10
    imu0: /imu_data_ros
    imu0_config: [false, false, false, false, false, true, false, false, false, false, false, true, false, false, false]
    imu0_relative: true
    imu0_queue_size: 50
amcl:
  ros__parameters:
    use_sim_time: false
    base_frame_id: base_link
    odom_frame_id: odom
    global_frame_id: map
    scan_topic: scan
    robot_model_type: nav2_amcl::DifferentialMotionModel
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.6
    alpha4: 0.3
    alpha5: 0.2
    laser_model_type: likelihood_field
    sigma_hit: 0.08
    z_hit: 0.9
    z_rand: 0.1
    max_beams: 120
    min_particles: 1500
    max_particles: 2500
    update_min_d: 0.05
    update_min_a: 0.1
```

**Source:** `/home/rb3302-7/rb3302_fresh/backups/SUCCESS_RPP_20261111_173648/student_ws/src/rb3302_mission/config/course.yaml`. [代码/计算检查]

```yaml
landmarks:
  A:
  - 0.29
  - 2.03
  B:
  - 0.87
  - 2.03
  C:
  - 1.45
  - 2.03
  D:
  - 2.03
  - 2.03
  E:
  - 2.61
  - 2.03
  F:
  - 3.19
  - 2.03
  G:
  - 0.29
  - 1.45
  H:
  - 0.87
  - 1.45
  I:
  - 1.45
  - 1.45
  J:
  - 2.03
  - 1.45
  K:
  - 2.61
  - 1.45
  L:
  - 3.19
  - 1.45
  M:
  - 0.29
  - 0.87
  N:
  - 0.87
  - 0.87
  O:
  - 1.45
  - 0.87
  P:
  - 2.03
  - 0.87
  Q:
  - 2.61
  - 0.87
  R:
  - 3.19
  - 0.87
  S:
  - 0.29
  - 0.29
  T:
  - 0.87
  - 0.29
  U:
  - 1.45
  - 0.29
  V:
  - 2.03
  - 0.29
  W:
  - 2.61
  - 0.29
  X:
  - 3.19
  - 0.29
```

**Source:** `/home/rb3302-7/rb3302_fresh/backups/SUCCESS_RPP_20261111_173648/student_ws/src/rb3302_mission/launch/mission.launch.py`. [代码/计算检查]

```python
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rb3302_mission'))
    speed_parameters = ({'drive_speed': .18, 'approach_speed': .08,
                         'stanley_cross_gain': .4, 'drive_turn_limit': .50}
                        if os.environ.get('RB3302_MISSION') == 'mission_corner_rpp' else {})
    return LaunchDescription([
        DeclareLaunchArgument('start', description='Starting letter A-X'),
        DeclareLaunchArgument('bearing', description='N, NE, E, SE, S, SW, W or NW'),
        DeclareLaunchArgument('goals', description='Four ordered letters, comma separated'),
        DeclareLaunchArgument('course', default_value=str(share / 'config/course.yaml')),
        Node(package='rb3302_mission', executable=EnvironmentVariable('RB3302_MISSION', default_value='mission'), output='screen',
             parameters=[str(share / 'config/control.yaml'), speed_parameters,
                         # Prefix short enums: ROS YAML treats bare N/Y as booleans.
                         {key: ParameterValue([{'start': 'cell_', 'bearing': 'dir_'}.get(key, ''),
                                               LaunchConfiguration(key)], value_type=str)
                          for key in ('start', 'bearing', 'goals', 'course')}]),
    ])
```

## 3 开发迭代时间线（最重要）

### 3.1 How to present the chronology honestly

V1, V2 and V3 are narrative groupings, not three clean consecutive releases. Motion, localisation and obstacle work overlapped; many requested edits were only checked locally before being superseded. The dates here are dates of saved development entries/backups, not reconstructed robot-calendar timestamps. October 27, October 29–31, November 1, November 4, and November 7–8 have no independently dated entry in H establishing a specific change; activity on those days is **未验证**, not evidence that no work occurred. [代码/计算检查] (H dated entries)
**Speak:** “We grouped an interleaved development history into three stages: basic motion, controller refinement, and obstacle-aware replanning.”

Evidence abbreviations in the following table resolve through Section 1. `H(date)` means the archived dated development entry; it supports the recorded change, not automatically the claimed physical cause. Where physical results are absent, the result explicitly says 未验证. Each last-column sentence can be spoken in fewer than 25 words.

| Date / stage | Observed phenomenon | Cause assessment | Change | Result | Evidence | Retained in final baseline? / Speak |
|---|---|---|---|---|---|---|
| Oct 26, inherited start | Early Nav2-based runs were jerky, overshot and sometimes reported no route. | No single isolated cause; package names do not prove an algorithm defect. | Inspected supplied navigation and created a separate mission flow. | A simpler task-specific architecture emerged; comparative superiority 未验证. | [代码/计算检查] (H §§1–2; 实机验证: user early feedback, individual timing unavailable) | Architecture yes. “We simplified the task flow so individual planning and motion errors were easier to inspect.” |
| Oct 26, V1 | Fine planning did not match requested cell-centre behaviour. | Representation mismatch. | 24-node cardinal 0.60 m graph; compress collinear intermediate centres. | Code-level connected graph checks; early runtime quality mixed. | [代码/计算检查] (H §2) | Concept yes; spacing later 0.58 m. “We separated the fine map from a coarse letter-cell planning graph.” |
| Oct 26, V1 | Straight tracking and in-place turns behaved poorly. | Pose lag, physical response and feedback faults were not isolated. | Heading PID, lateral correction, relative turns and speed/steering ramps tried. | No single validated cure. | [代码/计算检查] (H §4; 实机验证: qualitative conversation) | Straight-plus-turn baseline historical. “A simple motion sequence exposed problems that a route planner alone could not solve.” |
| Oct 26, sensor handling | Robot repeatedly stopped after sensor warnings. | One freshness gate coupled raw IMU, base, laser and odom at 0.5 s. | Decoupled raw freshness checks; retained usable localisation requirements. | Structural improvement; all stop-start causes not proven removed. | [代码/计算检查] (H §5) | Principles yes; final freshness values differ. “We distinguished delayed raw messages from losing a usable pose.” |
| Oct 26, serial handling | Commands could be delayed. | Blocking serial readline under lock was a code risk. | Available-byte buffering and complete-line parsing. | Local code improvement; contribution to every jerk 未验证. | [代码/计算检查] (H §5) | Driver redesign retained. “Serial handling must not delay outgoing control while waiting for incomplete input.” |
| Oct 26, distance fusion | User trusted wheel travel more than AMCL distance. | A design preference, not calibrated sensor confidence. | Summed 0.7v odom and ±0.3v localisation contributions; odom latch. | Software tests of values/latch; not a statistical fusion method. | [代码/计算检查] (H §6) | No. “The seventy-thirty experiment combined speed contributions, not AMCL particles or probabilistic position estimates.” |
| Oct 26, reverse AMCL | Reverse motion appeared forward in particle display. | Upstream small-step motion prediction branch reproduced locally. | First signed small-step patch, then signed-vector custom plugin. | Local tests improved reverse prediction; robot symptom persisted. | [代码/计算检查] (H §§7–8; 实机验证: user reverse feedback, E 29836068-cd6e-45df-8264-02672d4f9d52 insufficient for complete sign chain) | No custom plugin in final. “A reproduced prediction issue did not prove the deployed robot’s complete localisation chain was fixed.” |
| Oct 28, custom model | Earlier synthetic test missed near-pure-turn behaviour. | Zero-clipped Gaussian translation caused positive magnitude bias. | Signed-vector-v2 removes clipping bias and pure-turn translation injection. | Reproduction: approximately 8.97 mm drift per tiny-translation/0.05 rad turn in old rewrite. | [代码/计算检查] (G/CODE_REVIEW_20261028.txt; H Oct 28) | No; official model restored. “Our custom model introduced another edge case, so a successful reverse test was not enough.” |
| Oct 28, driver | Repeated snapshots produced zero/spike odometry; yaw units inferred incorrectly. | Confirmed code-level timing/unit risks. | Publish only new feedback; explicit radians, nonzero covariance; measured yaw change. | Local regressions passed; firmware semantics not fully calibrated. | [代码/计算检查] (H Oct 28 review fixes; G/CODE_REVIEW_20261028.txt) | Driver changes retained. “We corrected feedback timing instead of forcing odometry to agree with the command.” |
| Oct 28, reverse fallback | Particle reverse behaviour still unreliable in user tests. | Entire runtime cause unresolved. | Removed reverse translation; face target then drive forward. | Operational simplification, not proof reverse AMCL was repaired. | [实机验证] (user repeated reverse observations; 代码/计算检查: H latest change) | Forward-only mission retained. “We removed reversing as an operational fallback rather than claiming the unresolved symptom was solved.” |
| Oct 28, launcher | Start direction N caused mission failure. | ROS YAML interpreted bare N as boolean. | Prefix `dir_`/`cell_` during launch and strip in parser. | Source-level fix; not navigation failure. | [代码/计算检查] (H startup entry; launch/mission.launch.py) | Yes. “Some failures came from parameter transport, not from the navigation algorithm.” |
| Oct 28, V3 early | Single close return repeatedly vetoed turns. | One-point obstacle criterion too sensitive. | Full-road barrier spanning ±0.25 m, ≥5 points, ≤0.08 m gaps, ≥4 frames/0.5 s; latest wall only. | Later replaced; broad-wall observation sometimes missed blocks. | [代码/计算检查] (H Oct 28; 实机验证: E 06ef782e-c482-4eec-a931-e98e76d7ad22) | No exact rule. “A single return was not enough evidence that the whole passage was blocked.” |
| Oct 28, goal correction | Repeated large turns chased centimetre-scale residuals. | 0.01 m acceptance too strict for noisy pose/low-speed motion. | Goal tolerance 0.05→0.01→0.05 m; stopped recheck tried. | User preferred restored wider tolerance. | [实机验证] (E 05dea704-18c7-4961-9601-bd0aeb50fc8d; 代码/计算检查: H) | No final radial tolerance. “A tighter numerical tolerance made behaviour worse instead of proving better physical accuracy.” |
| Oct 28, turn geometry | Turns appeared early or wide. | Physical pivot centre and pose reference uncertain. | Add incoming-path extension +0.10 m. | Some positive user feedback, not calibrated geometric correction. | [实机验证] (E 34520313-362d-46d0-9ed1-27f48a64a78f and subsequent user observation; 代码/计算检查: H) | No. “An endpoint extension was an empirical workaround, not a turning-centre calibration.” |
| Oct 28, recovery trial | Single turn lasted too long. | Odom-angle progress missing; cause unproven. | ≤0.4 s bursts; stable AMCL waits; 45 s turn and 15 s wait limits. | User reported poor result; rolled back. | [代码/计算检查] (H; 实机验证: E 4d6aa959-410c-4209-ab2a-a74e6dbbdebc and e2f13544-683a-4805-b805-ced89325f7eb) | No. “Adding more waiting rules did not resolve the underlying feedback problem.” |
| Oct 28, obstacle range | Full obstacle approached without stop. | 0.22 m detection range suspected too short. | Trial 0.60 m lookahead, then reverted at user request. | Deferred, not validated. | [代码/计算检查] (H Oct 28 final entries) | No. “We deferred obstacle changes until the motion baseline was usable again.” |
| Nov 2, V2 trials | Wanted alternatives to repeated fixed turns. | Need different tracking laws. | Separate PP, differential Stanley and DWA runners; no forced fixed 90° manoeuvre. | Ideal-model route tests recorded; clean physical ranking unavailable. | [代码/计算检查] (archive/historical_docs/CONTROLLER_TRIALS.txt; H) | Methods informed hybrid; runners inactive. “We compared several controller ideas, but did not establish a controlled real-robot ranking.” |
| Nov 2, deployment | Experimental modes reportedly did not move or had missing files. | Packaging/entrypoint and obstacle issues confounded comparison. | Returned focus to the legacy movement, rebuilt transfer archive. | User deferred other algorithms; no proof DWA intrinsically unsuitable. | [实机验证] (user file-not-found/nonmovement feedback; 代码/计算检查: H) | Deployment lesson retained. “A failed launch is not evidence that a control algorithm is fundamentally bad.” |
| Nov 2, V3 | N/S obstacle labels alternated, causing near-180° turns. | Blocked candidate excluded, any update interrupted route. | Keep currently observed blocked cell eligible; replan only on remaining-route blockage. | Regression checks passed; classification still unproven. | [实机验证] (E 59cbb1c7-832e-4d20-b4b4-908ef0226db4; 代码/计算检查: H) | Yes in evolved form. “Obstacle identity and route relevance must be separate decisions.” |
| Nov 2, V3 | Large box poorly represented by centre points. | Point-like assumption mismatched area. | Connected ≥0.20 m surface; central region; 0.05 m centroid stability; whole cell blocked. | Synthetic tests; later near-face model replaced centre-region rule. | [代码/计算检查] (H Nov 2) | Whole-cell occupancy yes; centre-region rules no. “We changed the planner state to an occupied cell, rather than treating a box as one point.” |
| Nov 2, rollback | Supposed restored motion still spun. | Copied motion matched baseline; angle feedback/runtime remained suspect. | Byte-for-byte baseline restore; obstacle temporarily disabled/restored. | No proof rollback fixed spinning. | [实机验证] (E 11973304-df85-414c-bf23-bf9e06e9fb44; 代码/计算检查: H) | Historical only. “Restoring identical code cannot by itself prove that live feedback is healthy.” |
| Nov 2, timed turn | Turn could reach 25 s timeout. | Fresh timestamp does not prove changing/correct yaw value. | Timeout 25→5 s; guessed 2.5 s/90°; then 0.4 s response calibration and one correction. | Fake-input checks, no exact physical-angle validation. | [代码/计算检查] (H Nov 2; 实机验证: earlier timeout log) | No. “A time-based turn needs a measured response model, and even then slip can change the result.” |
| Nov 2–3, Gazebo | Need repeatable testing without repeated hardware trials. | No simulator initially installed. | World from occupancy map, ROS bridges, same localisation chain; clean navigation experiment. | Nov 3 actual simulations completed; six-case precision 18/24 within 5 cm, max 6.55 cm. | [代码/计算检查] (G/matrix_tests/SUMMARY.json; H) | Test infrastructure; clean controller not final. “Gazebo helped test integration, but its wheel friction was not a calibrated replica of our rover.” |
| Nov 3, simplification | Exceptions/exits and turning logic became too complex. | Multiple experimental layers and strict checks. | Removed turn deadline and obsolete paths; AMCL-relative turns; streamlined mission. | User feedback continued; code simplification was not proof of accurate turning. | [代码/计算检查] (H Nov 3) | Some infrastructure retained; relative turns legacy. “We simplified the control path so the remaining behaviour could be understood and tested.” |
| Nov 3, arrival | Goal correction slow, repeated. | Centring routine and tight acceptance added extra motion. | Endpoint-plane crossing; goal centre without +0.10 m; 3 s hold. | Later RPP final approach and 4 s hold. | [代码/计算检查] (H Nov 3; 实机验证: E 8884178c-c692-47ab-adc9-b3f7126caccb) | Plane concept yes. “We separated reaching the goal plane from repeatedly chasing small residual errors.” |
| Nov 3, early stop | Physical turns looked overshot. | Estimated compensation, not measured stopping dynamics. | `turn_early_stop_degrees=15`. | Logs showed AMCL around 76° after nominal 90°, while user judged body orientation acceptable. | [实机验证] (E 4bef6c24-41e4-4676-a8fa-054d1258706d; 代码/计算检查: H) | Legacy only. “The fifteen-degree setting was an empirical workaround, not a verified physical overrun measurement.” |
| Nov 3, straight Stanley | Straight PID/lateral behaviour poor. | Heading alone or ad hoc target offset insufficient. | Stanley-style angular correction; later virtual point 0.10 m, heading gain 1→1.5, cap 0.35→0.50 rad/s. | XUNX showed lateral oscillation/saturation; parameter adaptation continued. | [实机验证] (E 84f31fe6-aa0e-443e-9c4d-abd44ddaa268; 代码/计算检查: H) | Stanley form yes. “We used lateral displacement explicitly instead of correcting only heading.” |
| Nov 5, PID+Stanley | Needed stronger and steadier straight tracking. | Pure heading/lateral proportional action insufficient for bias/damping. | Kp 1.5, Ki 0.03, Kd 0.08, D filter 0.20 s; then latest TF pose and fused yaw-rate D. | User said straight travel improved; no controlled RMS comparison. | [代码/计算检查] (H Nov 5; 实机验证: E f9dcfdc8-26fe-4497-8e0a-230587123d84 and user feedback) | Yes. “We kept the lateral law and added small integral correction with measured-rate damping.” |
| Nov 5, localisation density | Turning localisation looked delayed/imprecise. | CPU/rate bottleneck suspected, not measured. | EKF 30→50 Hz; AMCL 1000–3000 particles, 120 beams. | No hardware load proof; later AMCL rollback. | [代码/计算检查] (H) | EKF yes; particle settings later changed. “More particles were a trial, not evidence that localisation became faster.” |
| Nov 5, map geometry | Systematic apparent centre/wall mismatch. | User measured grid at 58 cm, not nominal 60. | Scale map 58/60, coordinates/grid/RViz together; outer walls adjusted one old pixel inward. | Same graph topology in local check. | [代码/计算检查] (H; course/map files; 实机验证: operator spacing report, ruler protocol unavailable) | Yes. “We corrected the map scale and target coordinates together.” |
| Nov 5, extension/slowdown | Cutting inner corners and stopping early. | Centre reference not yet resolved. | Extension 0.10→0.05→0 m; last 0.15 m slow to 0.04 m/s; later only goals slow. | Perceived error persisted; not an isolated control-gain problem. | [代码/计算检查] (H; 实机验证: user feedback across attachments) | Extension removed; slowdown not final active path. “Changing the endpoint cannot compensate reliably for an incorrect sensor reference.” |
| Nov 5, mission_arc | Rolling turns incomplete; still pivoted; rubbed inner wall. | Observed tracking failure; slip/pose/controller balance not isolated. | Radius 0.18 m, speed 0.06 m/s; turn cap 0.5→2 rad/s; heading gain 1.5→3; 5° exit check. | Exit position could trigger pivot alignment; user reverted to in-place version. | [实机验证] (E 6682d9d0-9953-4854-b6b6-e53115059630; 代码/计算检查: H, arc_node.py) | No. “The first arc controller still needed exit pivots, so it did not deliver the intended continuous turn.” |
| Nov 5, LiDAR offset | Body stopped short while its front appeared centred. | Wrong base-to-laser longitudinal transform was a concrete geometric error. | `lidar_from_imu_x` −0.052→+0.050 m; difference 0.102 m. | User X→R centre feedback positive; XUNX completion later reported good. | [实机验证] (E dea73374-eab6-499d-acec-b74544f8c89c; E f0555245-ddb7-492c-b313-e524ae4d0f96; 代码/计算检查: H) | Geometry correction yes; final +0.015 m. “The reference-point correction mattered more than repeatedly changing the arrival threshold.” |
| Nov 5, V3 near face | Box seen as one wall, not a square outline. | Requiring centre/square appearance was inappropriate. | Broad near-face detector; initially route cells within 1.5 m; width 0.30 m, depth band 0.12 m, 4 frames/0.5 s. | Later false walls required refinements. | [代码/计算检查] (H; 实机验证: E b1b628b1-8492-4bab-904d-9dffff0bc8cc) | Broad face yes. “A laser usually sees one face, so the detector should not require a complete box outline.” |
| Nov 5, turn extension | Proposed 17 cm to compensate body shift. | Uncalibrated workaround. | +0.17 m briefly added then cancelled before continuing obstacle work. | Physical benefit 未验证. | [代码/计算检查] (H Nov 5; user cancellation) | No. “We did not retain the proposed seventeen-centimetre turn extension.” |
| Nov 5, V3 geometry | Original walls falsely blocked cells. | Mapped-wall/visibility filtering and neighbour scope insufficient. | Axis-angle fit initially 8°, RMS≤0.025 m; ray visibility; wall margin 1→2 pixels; four open neighbours. | User later called obstacle handling good. | [代码/计算检查] (H; 实机验证: E d2ac22c7-7716-463e-9ee2-427f0e9a1177 plus user feedback) | Yes, angle later 10°. “We checked map consistency before treating a straight surface as a new obstacle.” |
| Nov 5, near boundary | False blocking away from expected box face. | Geometry admission too broad. | Near-face tolerance briefly 0.05 m, then ±0.10 m; no-route candidate veto. | Task-specific assumption retained, not universally safe. | [代码/计算检查] (H) | Veto yes; tolerance later ±0.20 m. “We restricted candidates to the neighbour boundary facing the robot.” |
| Nov 5–6, frozen turn guard | Physical robot turned while RViz heading appeared frozen. | Could be feedback/TF/AMCL chain; AMCL alone not proven guilty. | 1 s measured rotation≥12° with AMCL spread≤3° caused abort; user requested removal. | Guard detected mismatch but did not cure it. | [实机验证] (user frozen-heading observation; 代码/计算检查: H) | No. “A protection check detects a symptom; it does not repair the source of stale orientation.” |
| Nov 6, IMU parser | Potential stream freeze after CRC noise. | Local reproduction confirmed parser failed to resynchronise. | Header search, CRC skip, split/overflow/payload fixes. | Old decoder: 0/200 following valid frames; fixed: 200/200. Robot incident causality 未验证. | [代码/计算检查] (G/imu_stream_recovery_20261106/RESULT.txt) | Yes. “We fixed a reproducible stream-decoding defect without claiming it explained every earlier turn failure.” |
| Nov 6, LiDAR calibration | Some stopping offsets remained, about 3–4 cm by eye. | Earlier +5 cm placement estimate too large. | +0.050→+0.015 m. | User reported centred stops; no independent per-goal metrology. | [实机验证] (E c66a4f65-9246-41ce-b8b9-e1384e4901b8; E 3a18d084-94cb-4269-841f-436d51ba080e; 代码/计算检查: H) | Yes. “The final sensor offset came from physical inspection and robot feedback, not a formal calibration rig.” |
| Nov 6, obstacle tolerance | Nearby obstacle only partly visible. | Visible width/placement can differ from ideal face. | Boundary ±0.20 m; angle briefly 5°, then 10°; minimum visible width remains 0.30 m. | No independent detection recall estimate. | [代码/计算检查] (H; 实机验证: user feedback) | Yes in frozen baseline. “We relaxed placement and angle tolerance while preserving the large-face requirement.” |
| Nov 6, obstacle reachability | User worried about no-route waits. | Arena expected to leave a detour. | No-route veto removed then restored; distant observation tried. | Final experimental detector later uses only open neighbours. | [代码/计算检查] (H; experiment_obstacles.py) | Veto retained; far-cell observation not final. “Rejecting an impossible detour was an arena assumption, not proof that the observation was false.” |
| Nov 6, speed trials | Long missions, slow turns/goal approaches. | One supplied run exceeded 215.66 s without completion. | Drive .12→.14→.16→.24 m/s; turn .75→.90→1.05→1.20→1.80 rad/s; minimum turn removed. | Mixed user feedback; no universal speed guarantee. | [实机验证] (E cde4c858-4f70-4886-9b29-3171da19901f; 代码/计算检查: H) | Base parameters remain; final RPP cruise override .18. “Speed was increased experimentally, then reduced when the combined tracking behaviour was better.” |
| Nov 9, reference code | Antonio/merged robot motion looked promising. | Useful method reference, not proof of whole-stack superiority. | Two independent versions: measured-rate feedback; corner-window RPP. Copied calibration idea removed. | Local implementation; no Antonio runtime import or motor calibration dependency. | [代码/计算检查] (TWO_TURN_VERSIONS.txt; H) | Own RPP/rate trim retained. “We reused established ideas but wrote our own task-specific controller implementation.” |
| Nov 9, first simulation | RPP clear run completed but overlapped walls. | Endpoint carrot collapse associated with poor departure heading. | Extend carrot beyond final line, keep actual arrival plane. | 94.05 s/27 overlap frames rejected; 95.78 s clear retest no overlap. | [代码/计算检查] (G/two_turns_20261109_224435/final_report) | Yes. “Completion alone was not a pass; collision samples exposed a geometric tracking problem.” |
| Nov 9, simulated obstacles | Both versions falsely blocked other cells and exceeded 180 s. | Known-wall points participated in candidate fitting. | Filter static-wall-near points before fitting; only open neighbours. | Feedback 91.05 s, RPP 67.955 s with Q obstacle; zero sampled overlaps/false blocks. | [代码/计算检查] (G/two_turns_20261109_232925/final_report/RESULTS.txt, Gazebo only) | Yes. “Filtering known walls before fitting removed the false blocks in that simulation.” |
| Nov 10, RPP smoothing | RPP alternated pivot and travel on right-angle reference. | Sharp reference plus under-realised yaw rate; exact real cause not isolated. | Quintic Bézier trial, then tangent quarter circles nominal 0.30 m capped 0.29 m. | User later preferred turning; no quantitative comparison. | [实机验证] (E f99cc574-0c15-4fdf-84b6-f18eee07c322 and 1af1f42d-ceed-4391-b5e5-1c1baa98230f; 代码/计算检查: H) | Quarter circle yes; Bézier no. “RPP needs a usable reference curve; tracking raw right-angle points was not enough.” |
| Nov 10, goal alignment | Stops not physically centred; corrections slow. | Pose/actuation/slip effects confounded. | 2 cm point regulation, 5 cm RPP, then 3 cm acceptance; repeated settled rechecks; faster goal approach. | User said correction could make things worse. | [实机验证] (E 1a8a949e-c9e6-4574-ba39-960afd2fe9b5; conversation; 代码/计算检查: H) | No repeat correction. “We tried tighter goal alignment, but repeated small motions were not reliably beneficial.” |
| Nov 10, obstacle unification | V2 seemed to miss obstacles unlike V1. | Scan/replan anchoring differed between experiments. | V2 inherits same V1 experimental scanner and centre-anchor replan flow. | User later confirmed obstacle behaviour acceptable. | [代码/计算检查] (H Nov 10; 实机验证: conversation after unification) | Yes. “The two motion experiments should share the same obstacle policy.” |
| Nov 10, goal simplification | Extra centring did not improve observed stops. | Too many low-speed corrective movements. | Zero longitudinal offset; at most one correction trial; then no correction, 4 s hold. | User preferred one approach; no measured accuracy gain. | [实机验证] (user comparison; 代码/计算检查: H) | Hold-only yes. “We retained one approach and a stationary hold instead of repeated correction.” |
| Nov 10–11, tracking trials | Slow return to centreline after curves. | Lookahead, geometry and pose noise trade-off. | Added cross-error/cosine speed scaling; straight gain .4→.6, cap .5→.65. | Later reverted; isolated benefit 未验证. | [代码/计算检查] (H; 实机验证: user poor tracking feedback) | No. “Stronger correction and extra slowdown did not automatically improve the complete route.” |
| Nov 11, radius | User wanted more straight travel between bends. | Smaller radius shortens curved portion but increases curvature. | Radius .29→.20 m; then restoration/reversion sequence. | 0.20 m geometry gives 0.38 m exit-to-next-centre distance; physical superiority unproven. | [代码/计算检查] (H Nov 11; arithmetic) | Final .29 m. “A smaller radius leaves more straight distance but demands a tighter physical turn.” |
| Nov 11, lookahead | User asked stronger curve/line tracking. | Short lookahead may amplify pose noise. | .20→.05→.10 m. | Poor tracking reported; no isolated statistical comparison. | [代码/计算检查] (H; 实机验证: E e2db1c39-9098-4abd-b3a9-916f29bd3778) | Final .20 m. “We tested short lookaheads, then returned to the earlier twenty-centimetre setting.” |
| Nov 11, code review | Departures inherited old quarter-turn early stop; transitions paused. | Active-method dispatch and rate-trim scope defects. | Absolute heading helper; rate trim on straights; no extra align at continuous handoff; saturation consistency. | Targeted checks passed; not proof of physical perfection. | [代码/计算检查] (archive/historical_docs/CODE_REVIEW_20261111.txt) | Yes, with Nov 12 RPP-first extension separate. “We corrected which methods were actually called, not just parameter values.” |
| Nov 11, goal-cell RPP | Straight arrival still poor by eye. | Hypothesis that final tracking method matters. | Final half-cell switches to RPP at midpoint; goal plane and hold remain. | Later successful user run; isolated contribution unmeasured. | [代码/计算检查] (H; 实机验证: E-base and user success) | Yes. “RPP also handles the final goal cell, without changing the goal coordinates.” |
| Nov 11, particle latency | Particle cloud measured 1.3–1.5 Hz, max gap 5.337 s. | Workload/TF/motion gating possible; bottleneck unproven. | 1000–3000/120→500–2000/60; stationary-only no-motion requests; original AMCL restore. | Frequency improvement not measured in matched before/after run. | [实机验证] (user topic-hz output; 代码/计算检查: H) | Stationary service rule yes; final particle counts differ. “The particle display was slow, but we did not prove that particle count alone caused the delay.” |
| Nov 11, final rollback | Later changes degraded turning/straight tracking. | Multiple coupled changes. | Original .29 m quarter circle, .20 m lookahead, curvature-only regulation; straight gain .4/cap .5; Antonio AMCL mapping. | User: “这一版很好…完全成功的”; archived E-base. | [实机验证] (E-base and user confirmation; 代码/计算检查: H) | Yes. “We restored the combination that worked, rather than keeping every later experiment.” |
| Nov 11, startup + speed | Wanted initial convergence and faster completion. | Startup uncertainty observed; speed optimality unproven. | ≥3 s/five-sample gate; all speed ×1.5, then only straights ×1.5, then ×2. | User liked convergence; requested all original speeds restored. | [实机验证] (E-start, E 1439f3ba-bbd0-4eb2-8abf-b8664b78e7d6; 代码/计算检查: H) | Gate yes; multipliers no. “We kept startup convergence but rolled back the speed multipliers.” |
| Nov 11, freeze | Wanted reproducible successful source. | Repeated edits risked losing known behaviour. | Frozen `SUCCESS_RPP_20261111_173648`, checksums; report and slides later generated. | User subsequently confirmed another successful run; no separate full log supplied. | [代码/计算检查] (S/VERSION.txt, SHA256SUMS; 实机验证: final user success confirmation) | Report baseline. “The successful snapshot lets us separate confirmed behaviour from later experiments.” |

### 3.2 Controller trials: what we can and cannot claim

| Controller | Actual implementation / test | Why not the standalone final controller | Speak |
|---|---|---|---|
| Heading PID alone | Heading error, later measured-rate D; insufficient to fix a parallel lateral offset by itself. [代码/计算检查] (H; node.py) | Retained as one part, not abandoned. Physical quantitative improvement per PID term 未验证. | “PID remained useful for heading, but lateral offset needed its own correction.” |
| Standalone Stanley | Nov 2 virtual point 0.10 m ahead, cross gain .6 s⁻¹, heading multiplier 1.5 s⁻¹, cap .5 rad/s; later straight-specific variant. [代码/计算检查] (archived controllers/stanley.py) | User reported weak/oscillatory tracking at different settings; kept lateral form with PID. No rigorous all-controller hardware ranking. | “We retained Stanley’s lateral-error idea rather than using standalone Stanley for every manoeuvre.” |
| Pure Pursuit | Nov 2 lookahead .22 m, cruise cap .10 m/s, near-goal min .03 m/s, angular cap .5 rad/s, acceleration .20 m/s²; target behind allows pivot. [代码/计算检查] (archived controllers/pure_pursuit.py) | Not simply discarded: curvature pursuit became the foundation of final RPP. Early deployment issues confounded robot results. | “Pure pursuit evolved into the final curvature-regulated tracker.” |
| RPP | Own local pursuit with rounded path, curvature speed control and yaw-rate trim. [代码/计算检查] (final modules; 实机验证: E-base, user success) | Selected for corners/goals; ordinary straights retained familiar PID/Stanley. Not demonstrated universally best. | “The hybrid used each controller where its behaviour was easiest to make reliable.” |
| DWA | Samples 4 linear×11 angular=44 pairs, 1.5 s horizon; a_v=.20 m/s², a_w=1 rad/s², cruise .10 m/s, angular .5 rad/s. Score includes lateral, heading, progress and speed; endpoint term near .35 m. [代码/计算检查] (archived controllers/dwa.py) | Early nonmovement/file-not-found feedback did not isolate DWA. Work was deferred in favour of simpler motion; saying “DWA failed because of CPU” is unsupported. | “DWA was implemented and explored, but we did not complete a fair physical comparison.” |
| MPC | Discussed, user explicitly declined it. No active implementation/test established. [代码/计算检查] (conversation record; physical result 未验证) | Not selected; do not claim a failed MPC experiment. | “MPC was discussed, but it was not one of our implemented final candidates.” |

DWA prototype score is `6*lateral +1.2*abs(heading_error) −2*progress_gain −0.5*v`, plus `4*goal_distance` within .35 m of path end. These weights mix scaled quantities and were heuristic; no optimisation-derived weight selection was documented. A no-valid-candidate result is possible. [代码/计算检查] (archived controllers/dwa.py under `P/backups/history/before_clean_active_20261103_155038/rb3302_mission/controllers`)
**Speak:** “Our DWA weights were prototype tuning choices, not a learned or theoretically optimal cost function.”

### 3.3 Custom AMCL plugin: preserve the uncomfortable history

The Oct 26 record describes a local test of the upstream differential motion model: reverse odometry step −0.00375 m at 8 Hz predicted forward displacement in the small-translation branch, with zero motion noise. A custom signed update was tested with 1000 particles, 20 random seeds and alpha1–4=.2. Prediction-only first reverse leg was approximately +0.5990 m upstream versus −0.6006 m modified, for a true −0.600 m step sequence. These are archived software-test claims; they are **not physical localisation accuracy**. The first rewrite then introduced clipping bias near pure rotation; a later rewrite corrected that software issue. Robot reverse behaviour remained unconfirmed, reverse translation was removed, and official AMCL was eventually restored. [代码/计算检查] (H §§7–8 and G/CODE_REVIEW_20261028.txt; 实机验证: user continued reverse complaints; deployed class/TF sign chain 未验证)
**Speak:** “The custom motion-model work taught us to test edge cases and distinguish local algorithm tests from complete robot behaviour.”

### 3.4 Supplied Nav2 and Antonio/merged: relationship and attribution

The inspected `planning_astar` configuration in A/M selects `nav2_navfn_planner::NavfnPlanner` with `use_astar=true` and a Nav2 RPP controller. Therefore its name does not mean it has no local controller. `planning_rpp` in those reference repositories uses the Nav2 architecture; the later configuration selects `nav2_smac_planner::SmacPlanner2D`, RPP, costmaps, progress/goal checking and behaviour actions. `M/.../scripts/go_letters` sends navigation actions; `cell_obstacles` supplies obstacle integration. These local files have evolved: their present contents do not prove the exact original teacher release byte-for-byte. [代码/计算检查] (A/M student_ws/src/planning_astar and planning_rpp configs/scripts)
**Speak:** “The supplied packages were complete Nav2 configurations; our final mission replaced their planning and control orchestration.”

Our final stack still uses Nav2's AMCL and map server. It does **not** run the Nav2 controller server, BT navigator or Nav2 RPP plugin in the final all-launch. We wrote coarse cell A*, straight tracking, curve generation, task sequencing and obstacle-cell policy. Antonio/merged served as a behaviour/method reference; its AMCL configuration was deliberately adopted. Its motor calibration was not retained as an imported dependency. A fair performance benchmark showing the teammate stack was worse does not exist in the examined evidence; do not invent a failure reason. The selection reflects task-specific transparency and preservation of our working controller. [代码/计算检查] (final launch; TWO_TURN_VERSIONS.txt; H Nov 9/11; 推测: comparative superiority 未验证)
**Speak:** “We retained standard localisation components, adopted a reference configuration, and implemented our own task-specific navigation logic.”

### 3.5 Other work that should not be confused with final algorithm validation

Map alignment/letter display, CLI upload/build scripts, RViz costmap visualisation, driver and IMU debugging, multiple source backups, source manifests, Gazebo scenarios, evidence archiving, Git repository setup, folder organisation, report writing and a 37-slide study deck were also produced during the project. Git push authentication initially failed; this conversation does not establish a later successful push of the final frozen code. The pink RViz planning costmap is a display of static inflated geometry, not proof of a Nav2 live obstacle costmap. [代码/计算检查] (P scripts, archive/report folders, costmap_view.py; 实机验证: user terminal authentication failure and GUI observations)
**Speak:** “Reproducible deployment and evidence storage were part of the engineering work, alongside the navigation algorithms.”

### 3.6 Separate rewrites: clean/ and autonomous/ are not the final hybrid

The **clean** experiment and the later **autonomous** Nav2 rewrite are separate branches, not alternate names for the final `mission_corner_rpp`. The autonomous branch uses NavFn A* on a 0.025 m costmap, stock AMCL, EKF, Nav2 RPP, velocity smoother, live costmaps, a freshness gate and NavigateToPose mission actions. It also implemented bounded initial scan matching around the known starting cell: three scans; search ±0.20 m and ±45°; trim the worst 20% of endpoint residuals. This scan matcher is **not active in the final hybrid**, so the statement “corner/scan matching was never implemented anywhere” would be too broad. [代码/计算检查] (`P/autonomous/README.txt`, initial_alignment.py, navigation.launch.py)
**Speak:** “We also built a separate Nav2 rewrite, but its startup scan matcher is not part of the final hybrid.”

Autonomous branch motion was nominal 0.20 m/s, angular cap 0.60 rad/s, acceleration 0.30 m/s² and deceleration 0.50 m/s². Initial navigation acceptance was 0.025 m estimated error, then a ≥0.8 s stopped check against 0.05 m before 3 s dwell. Its mission had a 180 s overall deadline and retry after 0.5 s, unlike the final hybrid. These numbers belong to `autonomous/nav2.yaml`, `mission.py` and `motion_gate.py` as described in that branch README, not to C/control.yaml. [代码/计算检查] (autonomous README and configuration provenance; physical validation 未验证)
**Speak:** “The alternative rewrite had different planning, stopping and timeout policies and must not be mixed with the final controller.”

The autonomous six-case Gazebo suite recorded simulated times 61.2, 97.6, 108.1, 91.809, 97.9 and 108.6 s; maximum true goal error 0.0789 m; 20/24 goals within 0.05 m; all 24 centres under the body footprint. Zero sampled collision overlaps were reported. This differs from the earlier clean suite's 18/24 and 0.0655 m figures. An earlier concurrent simulation suffered a 4.7 s AMCL gap and continued simulated movement after zero ROS commands; the cause of the transport stall remained unresolved. [代码/计算检查] (`P/autonomous/evidence/RESULTS.txt`, `20261103_143020/SUMMARY.json`, earlier `20261103_022817` evidence; Gazebo only)
**Speak:** “The separate rewrite passed finite simulated cases, but that did not establish equivalent real-robot performance.”

The user reported that the independently designed version did not move on hardware and chose to continue the simpler legacy-derived controller. There is no controlled physical comparison isolating the autonomous failure mechanism, so “Nav2 is unsuitable” or “the clean rewrite was fully proven on the robot” is unsupported. [实机验证] (conversation: “你设计的也动不了大哥”); [推测] (specific cause 未验证)
**Speak:** “A simulator pass did not replace deployment and physical testing, and the user chose the working development branch.”

The simplified legacy Nov 3 Gazebo case X/N, goals X,O,Q,X completed in 126.495 s with reported actual goal errors approximately 0, 0.0670, 0.0343 and 0.0462 m. This predates later physical feedback and configuration changes. A separate old weighted-distance obstacle test took 239.811 s with collision and was not a pass. [代码/计算检查] (H Nov 3; `G/legacy_turn_20261103_150826/result.json` as archived record; simulated results)
**Speak:** “We retained failed simulation cases instead of treating every completion as a successful validation.”

## 4 五个关键挑战与教训

These are the five challenges in report Section 3.1, spread over slides 26–28. Numerical improvements are stated only where supported; the absence of a controlled before/after measurement is explicit.

### 4.1 Getting the robot reference point right

- **Phenomenon:** the user saw the front of the robot reach the centre while the body centre remained short; initial estimates were approximately 5–10 cm by eye. Source geometry had LiDAR x=−0.052 m, later corrected to +0.050 m, a 0.102 m change in represented sensor location. [实机验证] (conversation around E 392fcde2-0757-41bd-ada9-72edfe40300c; 代码/计算检查: H Nov 5)
  **Speak:** “The robot appeared to stop early because its sensor-to-body reference was wrong.”
- **Diagnosis and fix:** compare physical sensor placement with TF, rather than merely enlarging distance thresholds; +0.050 m later refined to +0.015 m after operator inspection. [代码/计算检查] (robot_model xacro; 实机验证: E dea73374-eab6-499d-acec-b74544f8c89c; E 3a18d084-94cb-4269-841f-436d51ba080e)
  **Speak:** “We corrected the laser transform, then refined its offset using physical placement and repeated runs.”
- **Improvement:** user described X→R as centred, then XUNX as good. The XUNX logged TF distances U=.022 m, N=.017 m, final X=.004 m are internal estimates. Physical improvement in centimetres or percentage is **未验证**, not “10 cm became 1 mm accuracy.” [实机验证] (E f0555245-ddb7-492c-b313-e524ae4d0f96; H verified-full entry)
  **Speak:** “The stopping position improved visibly, but our logs do not provide independent centimetre-level ground truth.”
- **Lesson / likely follow-up:** “Why does moving a sensor transform move the estimated base?” Because `T_map_base = T_map_laser * inverse(T_base_laser)`; a wrong mounting transform creates systematic body-pose error. Other causes such as scale and braking can coexist. [代码/计算检查] (rigid-transform relation)
  **Speak:** “Correct control equations cannot compensate reliably for a wrong coordinate transform.”

### 4.2 Accounting for skid-steer motion during turns

- **Phenomenon:** the body translated during nominal in-place turns, sometimes described as moving back roughly a body length. That is an operator description, not measured displacement. User also reported inner-wall rubbing and repeated spin episodes. [实机验证] (conversation around E b1b628b1-8492-4bab-904d-9dffff0bc8cc and 6682d9d0-9953-4854-b6b6-e53115059630)
  **Speak:** “An in-place command did not guarantee that the physical body centre stayed fixed.”
- **Diagnosis:** skid steering requires slip, but slip alone does not explain all persistent spinning; TF/IMU updates, low-speed response and command tracking also mattered. The IMU stream failure was locally reproduced; individual robot causality remained **未验证**. [代码/计算检查] (G/imu_stream_recovery_20261106; 推测: physical mechanism attribution)
  **Speak:** “We separated the rover’s turning geometry from failures in the orientation-feedback chain.”
- **Fix and improvement:** later RPP used 0.29 m quarter circles and 0.20 m lookahead, reducing reliance on repeated ideal pivots. User reported smoother turns; no measured reduction in turning-centre displacement is available. Pivots remain for large pursuit bearings and departures. [代码/计算检查] (corner_geometry.py; 实机验证: E-base plus user successful-run feedback)
  **Speak:** “Curved path tracking reduced dependence on perfect pivots, but it did not eliminate wheel slip.”
- **Likely follow-up:** “Why not use equal and opposite wheel speeds?” Ideal differential-drive kinematics gives `v=(v_R+v_L)/2`, `omega=(v_R-v_L)/b`; effective b and slip on a four-wheel skid-steer base vary. No physically identified b was used in our host controller. [代码/计算检查] (ideal kinematic relation; 推测: actual slip values 未验证)
  **Speak:** “Equal wheel commands imply zero translation only in an ideal model, not necessarily on our real floor.”

### 4.3 Distinguishing new obstacles from existing walls

- **Phenomenon:** early obstacle labels switched and produced unnecessary near-180° route changes; later simulation confirmed spurious blocks. Existing walls and obstacle faces both form straight returns. [实机验证] (E 59cbb1c7-832e-4d20-b4b4-908ef0226db4; 代码/计算检查: Nov 9 simulation records)
  **Speak:** “A straight laser surface is not automatically a new obstacle because the arena walls look the same.”
- **Fix:** remove map-wall-near returns before fitting, reject occluded rays, inspect only open cardinal neighbours, require stable broad near-face geometry, remember one cell and replan only on route conflict. [代码/计算检查] (experiment_obstacles.py and obstacle.py)
  **Speak:** “We combined map consistency, cell geometry and persistence instead of trusting one geometric feature.”
- **Measured improvement:** Nov 9 Gazebo post-fix cases had only Q confirmed, zero sampled overlap frames; feedback 91.05 s, RPP 67.955 s. These are simulation results, not physical percentages. Physical E-base confirms B detection and continuation to mission complete; general recall/false-positive rates are **未验证**. [代码/计算检查] (G/two_turns_20261109_232925/final_report/RESULTS.txt; 实机验证: E-base)
  **Speak:** “The corrected simulation removed false blocked cells in the tested case, and robot logs also recorded successful obstacle-triggered replanning.”
- **Likely follow-up:** “Can filtering hide a real block beside a wall?” Yes. Nov 12 L miss shows the final detector is not universally reliable; exact failed condition was not logged. New width/time trial remains unvalidated. [实机验证] (E-L; 代码/计算检查: filters; 推测: particular width cause)
  **Speak:** “Filtering known walls reduces false positives but can remove part of a real obstacle’s visible face.”

### 4.4 Avoiding counterproductive goal corrections

- **Phenomenon:** 1 cm goal tolerance led to repeated large turns; later RPP goal corrections with 2/3/5 cm settings were reported to worsen stops. Different trials used different arrival methods, so these tolerances are not interchangeable experiments. [实机验证] (E 05dea704-18c7-4961-9601-bd0aeb50fc8d; E 1a8a949e-c9e6-4574-ba39-960afd2fe9b5; user feedback; 代码/计算检查: H)
  **Speak:** “Small estimated errors triggered extra movements that sometimes made the physical position worse.”
- **Fix:** one RPP goal-cell approach, longitudinal crossing, 4 s hold and no renewed centring. [代码/计算检查] (corner_rpp.py)
  **Speak:** “We removed repeated centring and kept one continuous approach followed by a stable stop.”
- **Improvement / limitation:** user preferred observed behaviour; reduction in physical error and total seconds attributable to this change is **未验证**. E-base held X at estimated .0082 m and A at .0145 m; E-start includes H at .0786 m. Do not claim every final stop lies within 3 cm. [实机验证] (E-base, E-start; no ruler measurement)
  **Speak:** “The simpler method was more consistent by observation, but it still allows lateral stopping error.”
- **Likely follow-up:** “What if it crosses the plane far from the centre?” The code can finish; there is no lateral goal gate. This is a deliberate simplification with a documented limitation. [代码/计算检查] (final arrival condition)
  **Speak:** “Our goal criterion checks longitudinal progress, so exact two-dimensional centring is not guaranteed.”

### 4.5 Waiting for a usable initial estimate

- **Phenomenon:** user requested particle settling before departure and later said convergence was good. No controlled count of failed starts before/after exists. [实机验证] (startup-feedback conversation; E-start partial run)
  **Speak:** “We saw value in letting localisation settle before introducing motion.”
- **Fix:** ≥3 s stationary; five fresh AMCL estimates; largest XY standard deviation≤.05 m; yaw standard deviation≤5°. The Nov 12 log shows READY at 1780677833.588 after earlier waiting entries at 1780677829.789, demonstrating actual gating in that run. [代码/计算检查] (initialize; 实机验证: E-L)
  **Speak:** “Departure requires several fresh low-uncertainty estimates, not just a fixed sleep.”
- **Improvement / limitation:** user retained it after restoring baseline speeds. Improvement in startup accuracy or success rate is **未验证**. Low covariance can still be confidently wrong; wait may be unbounded. [实机验证] (user retention/success confirmation; 代码/计算检查: condition)
  **Speak:** “Covariance checks improve our departure criterion, but they do not certify the true pose.”
- **Likely follow-up:** “Why exactly five frames and three seconds?” They are conservative empirical design choices to avoid accepting one transient small-covariance update; no optimisation study establishes them as best. [代码/计算检查] (source; 推测: optimality 未验证)
  **Speak:** “We chose a short settling period and repeated confirmation; we did not derive an optimal convergence duration.”

## 5 结果

### 5.1 What counts as success

A `MISSION COMPLETE` line establishes software completion of the requested list for that run, not collision-free motion, physical centre accuracy or a three-minute pass. An operator's “成功了” is qualitative real-robot evidence, not a reconstructed log. A filtered terminal extract may omit earlier goals. A goal repeated in grep output must not be counted as a second visit. The inventory below preserves gaps rather than inventing routes. [代码/计算检查] (raw-log structure and mission code)
**Speak:** “We distinguish mission completion, physical performance and timing instead of treating one success message as all three.”

### 5.2 Best-supported named runs

| Run | Route / obstacle | Time definition | Stopping errors | Evidence / interpretation |
|---|---|---|---|---|
| Nov 5 LiDAR-offset check | X→R; repeated R goals in supplied log | Full startup-to-finish timing not claimed here | Reported TF endpoint distance .001 m; physical operator said centred | [实机验证] (E dea73374-eab6-499d-acec-b74544f8c89c; H verified_center entry) Estimate, not 1 mm physical accuracy. |
| Nov 5 complete V1 | X,U,N,X; no confirmed dynamic obstacle claimed from the selected evidence | 167.128 s from first GOAL X to final completion, **excluding initialisation and first goal hold** | TF U .022 m, N .017 m, final X .004 m; intermediate M .056 m | [实机验证] (E f0555245-ddb7-492c-b313-e524ae4d0f96; arithmetic/H verified-full entry) Not final RPP. |
| Nov 11 successful pre-gate RPP extract | Complete list unavailable in extract; visible GOAL X then A; B obstacle confirmed | Last visible X event to completion: 67.549854 s; **partial mission only** | Held TF X .0082 m; A .0145 m; anchor C .013 m at stop | [实机验证] (E-base) User called version successful; do not invent the missing two goals. |
| Nov 11 convergence/speed feedback extract | Visible X,H,A; full ordered four-goal list not proven by extract; B confirmed | Visible X event to completion: 65.948355 s; **partial mission only** | Held TF X .0346 m, H .0786 m, A .0469 m | [实机验证] (E-start) Not proof final restored-speed snapshot had these exact errors. |
| Final original-speed + gate confirmation | Route, obstacle and time not supplied with final “成功了” message | 未验证 | No attached per-goal values | [实机验证] (user confirmation after “来跑一次成功版”; 代码/计算检查: S/VERSION.txt records earlier evidence only) |
| Submitted video | Exact letter route and linkage to frozen version **未验证**; user confirmed obstacle rerouting in discussion | File duration **132.766667 s**, not independently verified complete mission stopwatch | Qualitatively close to centres; no independent metrology | [实机验证] (V and user video discussion; 代码/计算检查: frame count/fps) Do not identify this as XUNX or X,P,G,X without matching evidence. |
| Nov 12 speed trial | X→R→L→F path toward next goal; user says obstacle at L | Run interrupted; no success time | L missed; no completed next goal | [实机验证] (E-L) This is a later failure, not a reclassification of earlier successes. |

**Speak:** “The video is about two minutes thirteen seconds long, but its exact full-mission route has not been established.”

### 5.3 Simulation results — separate evidence category

- Nov 3 six-case Gazebo matrix: recorded 24 goal completions, no geometric overlaps, but only 18/24 true goal errors≤.05 m; max .0655 m. This used the separate clean experiment, not the final Nov 11 controller. [代码/计算检查] (G/matrix_tests/SUMMARY.json; H)
  **Speak:** “Completing all simulated goals did not mean every goal met a five-centimetre precision criterion.”
- Nov 9 final post-filter Gazebo pair: X/N, goals X,P,G,X, Q obstacle .58×.58 m. Feedback version 91.05 s, max goal error .0864494 m; RPP 67.955 s, max .0755106 m. Zero sampled planar body overlaps and no false blocked cells in these cases. Goal acceptance meant the goal lay under the body footprint. [代码/计算检查] (G/two_turns_20261109_232925/final_report/RESULTS.txt)
  **Speak:** “The simulation confirmed integration on that scenario, with ground truth used only for evaluation.”
- Simulation used real AMCL/EKF software but modelled physics; 5 ms physics step, .20 m square body. Friction/actuation were not system-identified against hardware. Therefore these times and true simulation errors must not be merged with real-robot statistics. [代码/计算检查] (same RESULTS.txt)
  **Speak:** “Running real localisation software in Gazebo does not make the simulated wheel response identical to the real rover.”

### 5.4 Complete available completion-evidence inventory

The machine-extracted inventory at the end of this section lists every attached extract with an actual timestamped completion or fourth-goal event found during this review. It is an inventory of **available evidence**, not every trial ever run. Routes missing from the extract remain unavailable. Durations are labelled as visible-log spans, not whole-mission times. All position residuals are internal estimates unless explicitly described otherwise. [代码/计算检查] (extraction of local E files)
**Speak:** “We can account for the surviving evidence, but we cannot recover missing trial measurements from memory.”


### 5.5 Raw completion extracts: full available inventory

Each item contains only timestamped ROS output; copied terminal grep commands are not completion evidence. Repeated identical lines are deduplicated. A complete event with an incomplete extract does not establish the missing goal sequence.

#### Extract E `8884178c-c692-47ab-adc9-b3f7126caccb`

- Visible GOAL events in excerpt order: X. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/8884178c-c692-47ab-adc9-b3f7126caccb/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780680047.875906414] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780680032.058391 → 1780680047.875906, span 15.818 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `eda43c63-7059-409c-a007-1043c92bf019`

- Visible GOAL events in excerpt order: unavailable. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/eda43c63-7059-409c-a007-1043c92bf019/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780677307.347291574] [rb3302_mission]: 目标 4/4：X，已停三秒。`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780677274.379449 → 1780677307.917940, span 33.538 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `d49fe7d1-b56b-4f46-80d2-402dfe51b275`

- Visible GOAL events in excerpt order: F, R, A. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/d49fe7d1-b56b-4f46-80d2-402dfe51b275/pasted-text.txt)
- Recorded completion: `[mission_corner_rpp-9] [INFO] [1780674974.964690328] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780674900.373707 → 1780674974.964690, span 74.591 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission_corner_rpp-9] [INFO] [1780674915.820163320] [rb3302_mission]: 停车诊断：目标=(3.190,2.030)m，路线朝向=+0.0度；TF控制位置=(3.192,2.048)m，朝向=+0.8度，纵向剩余=-0.002m，横向=+0.018m，距目标=0.018m；AMCL最新位置=(3.150,2.053)m，朝向=+1.4度，纵向剩余=+0.040m，横向=+0.023m，消息年龄=0.166s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_corner_rpp-9] [INFO] [1780674919.836259047] [rb3302_mission]: GOAL HOLD F: 已停4秒，二维误差=0.0365m；不纠正
[mission_corner_rpp-9] [INFO] [1780674931.884207949] [rb3302_mission]: 停车诊断：目标=(3.190,0.870)m，路线朝向=-90.0度；TF控制位置=(3.208,0.851)m，朝向=-87.7度，纵向剩余=-0.019m，横向=+0.018m，距目标=0.026m；AMCL最新位置=(3.199,0.917)m，朝向=-87.3度，纵向剩余=+0.047m，横向=+0.009m，消息年龄=0.188s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_corner_rpp-9] [INFO] [1780674935.900107687] [rb3302_mission]: GOAL HOLD R: 已停4秒，二维误差=0.0136m；不纠正
[mission_corner_rpp-9] [INFO] [1780674970.945696476] [rb3302_mission]: 停车诊断：目标=(0.290,2.030)m，路线朝向=+180.0度；TF控制位置=(0.289,2.055)m，朝向=+177.7度，纵向剩余=-0.001m，横向=-0.025m，距目标=0.025m；AMCL最新位置=(0.358,2.057)m，朝向=+176.2度，纵向剩余=+0.068m，横向=-0.027m，消息年龄=0.259s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_corner_rpp-9] [INFO] [1780674974.955643141] [rb3302_mission]: GOAL HOLD A: 已停4秒，二维误差=0.0445m；不纠正
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `f9dcfdc8-26fe-4497-8e0a-230587123d84`

- Visible GOAL events in excerpt order: X, X, U, N. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/f9dcfdc8-26fe-4497-8e0a-230587123d84/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780675116.621835483] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780674956.872639 → 1780675116.621835, span 159.749 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `d2ac22c7-7716-463e-9ee2-427f0e9a1177`

- Visible GOAL events in excerpt order: X, A, H, O. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/d2ac22c7-7716-463e-9ee2-427f0e9a1177/pasted-text.txt)
- Recorded completion: `[mission_obstacle-9] [INFO] [1780690938.539386947] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780690741.339759 → 1780690938.539387, span 197.200 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 1. Cells/events are reproduced below; physical placement is not independently measured. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission_obstacle-9] [WARN] [1780690760.963280869] [rb3302_mission]: 确认整格障碍：B，替换旧障碍None；原路线可走，继续执行。
[mission_obstacle-9] [INFO] [1780690920.304828517] [rb3302_mission]: 停车诊断：目标=(3.290,0.870)m，路线朝向=+0.0度；TF控制位置=(3.294,0.882)m，朝向=-5.8度，纵向剩余=-0.004m，横向=+0.012m，距目标=0.013m；AMCL最新位置=(3.260,0.883)m，朝向=-6.9度，纵向剩余=+0.030m，横向=+0.013m，消息年龄=0.214s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690935.199890386] [rb3302_mission]: 停车诊断：目标=(3.190,0.290)m，路线朝向=-90.0度；TF控制位置=(3.171,0.289)m，朝向=-92.1度，纵向剩余=-0.001m，横向=-0.019m，距目标=0.019m；AMCL最新位置=(3.172,0.313)m，朝向=-93.8度，纵向剩余=+0.023m，横向=-0.018m，消息年龄=0.266s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690762.930763420] [rb3302_mission]: 停车诊断：目标=(0.290,2.030)m，路线朝向=+90.0度；TF控制位置=(0.304,2.031)m，朝向=+87.5度，纵向剩余=-0.001m，横向=-0.014m，距目标=0.014m；AMCL最新位置=(0.304,2.009)m，朝向=+87.2度，纵向剩余=+0.021m，横向=-0.014m，消息年龄=0.225s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690796.988569761] [rb3302_mission]: 停车诊断：目标=(0.290,0.190)m，路线朝向=-90.0度；TF控制位置=(0.377,0.188)m，朝向=-95.1度，纵向剩余=-0.002m，横向=+0.087m，距目标=0.087m；AMCL最新位置=(0.381,0.229)m，朝向=-94.5度，纵向剩余=+0.039m，横向=+0.091m，消息年龄=0.230s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690815.696461265] [rb3302_mission]: 停车诊断：目标=(1.550,0.290)m，路线朝向=+0.0度；TF控制位置=(1.550,0.277)m，朝向=-3.9度，纵向剩余=-0.000m，横向=-0.013m，距目标=0.013m；AMCL最新位置=(1.507,0.276)m，朝向=-4.4度，纵向剩余=+0.043m，横向=-0.014m，消息年龄=0.183s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690835.945406778] [rb3302_mission]: 停车诊断：目标=(1.450,1.550)m，路线朝向=+90.0度；TF控制位置=(1.468,1.558)m，朝向=+89.6度，纵向剩余=-0.008m，横向=-0.018m，距目标=0.020m；AMCL最新位置=(1.473,1.512)m，朝向=+89.1度，纵向剩余=+0.038m，横向=-0.023m，消息年龄=0.181s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690851.223814532] [rb3302_mission]: 停车诊断：目标=(0.870,1.450)m，路线朝向=+180.0度；TF控制位置=(0.869,1.456)m，朝向=-173.0度，纵向剩余=-0.001m，横向=-0.006m，距目标=0.006m；AMCL最新位置=(0.881,1.458)m，朝向=-172.2度，纵向剩余=+0.011m，横向=-0.008m，消息年龄=0.235s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690874.802504523] [rb3302_mission]: 停车诊断：目标=(1.550,1.450)m，路线朝向=+0.0度；TF控制位置=(1.552,1.447)m，朝向=-3.9度，纵向剩余=-0.002m，横向=-0.003m，距目标=0.003m；AMCL最新位置=(1.531,1.447)m，朝向=-4.1度，纵向剩余=+0.019m，横向=-0.003m，消息年龄=0.170s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_obstacle-9] [INFO] [1780690891.146251653] [rb3302_mission]: 停车诊断：目标=(1.450,0.870)m，路线朝向=-90.0度；TF控制位置=(1.485,0.870)m，朝向=-103.6度，纵向剩余=-0.000m，横向=+0.035m，距目标=0.035m；AMCL最新位置=(1.486,0.896)m，朝向=-105.4度，纵向剩余=+0.026m，横向=+0.036m，消息年龄=0.270s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `4bef6c24-41e4-4676-a8fa-054d1258706d`

- Visible GOAL events in excerpt order: Q, X. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/4bef6c24-41e4-4676-a8fa-054d1258706d/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780680920.146358217] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780680830.771921 → 1780680920.146358, span 89.374 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `392fcde2-0757-41bd-ada9-72edfe40300c`

- Visible GOAL events in excerpt order: X, R, R, R. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/392fcde2-0757-41bd-ada9-72edfe40300c/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780682394.835705206] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780682371.606315 → 1780682394.835705, span 23.229 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `e2db1c39-9098-4abd-b3a9-916f29bd3778`

- Visible GOAL events in excerpt order: A, A, F, R. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/e2db1c39-9098-4abd-b3a9-916f29bd3778/pasted-text.txt)
- Recorded completion: `[mission_corner_rpp-9] [INFO] [1780674199.840480884] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780674123.212484 → 1780674199.840481, span 76.628 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission_corner_rpp-9] [INFO] [1780674199.830165676] [rb3302_mission]: GOAL HOLD A: 已停4秒，二维误差=0.0323m；不纠正
[mission_corner_rpp-9] [INFO] [1780674127.124078100] [rb3302_mission]: GOAL HOLD A: 已停4秒，二维误差=0.0494m；不纠正
[mission_corner_rpp-9] [INFO] [1780674148.110696321] [rb3302_mission]: GOAL HOLD F: 已停4秒，二维误差=0.0290m；不纠正
[mission_corner_rpp-9] [INFO] [1780674162.956545221] [rb3302_mission]: GOAL HOLD R: 已停4秒，二维误差=0.0812m；不纠正
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `d321b4b0-f509-4acf-ae5d-6169f27ae2d2`

- Visible GOAL events in excerpt order: X, A. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/d321b4b0-f509-4acf-ae5d-6169f27ae2d2/pasted-text.txt)
- Recorded completion: `[mission_corner_rpp-9] [INFO] [1780676698.402603629] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780676626.117940 → 1780676698.402604, span 72.285 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 1. Cells/events are reproduced below; physical placement is not independently measured. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission_corner_rpp-9] [WARN] [1780676657.106873801] [rb3302_mission]: 确认整格障碍：B，替换旧障碍None；挡住剩余路线，准备重新规划。
[mission_corner_rpp-9] [INFO] [1780676630.848756832] [rb3302_mission]: GOAL HOLD X: 已停4秒，二维误差=0.0082m；不纠正
[mission_corner_rpp-9] [INFO] [1780676657.746277343] [rb3302_mission]: 停车诊断：目标=(1.450,2.030)m，路线朝向=+180.0度；TF控制位置=(1.448,2.043)m，朝向=-178.7度，纵向剩余=-0.002m，横向=-0.013m，距目标=0.013m；AMCL最新位置=(1.515,2.040)m，朝向=-179.4度，纵向剩余=+0.065m，横向=-0.010m，消息年龄=0.533s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission_corner_rpp-9] [INFO] [1780676698.393873681] [rb3302_mission]: GOAL HOLD A: 已停4秒，二维误差=0.0145m；不纠正
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `c66a4f65-9246-41ce-b8b9-e1384e4901b8`

- Visible GOAL events in excerpt order: K, K, K, Q. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/c66a4f65-9246-41ce-b8b9-e1384e4901b8/pasted-text.txt)
- Recorded completion: `[mission_obstacle-9] [INFO] [1780675336.948077222] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780675317.583225 → 1780675336.948077, span 19.365 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 3. Cells/events are reproduced below; physical placement is not independently measured. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission_obstacle-9] [WARN] [1780675319.040936968] [rb3302_mission]: 确认整格障碍：P，替换旧障碍None；原路线可走，继续执行。
[mission_obstacle-9] [WARN] [1780675323.212549987] [rb3302_mission]: 确认整格障碍：V，替换旧障碍P；原路线可走，继续执行。
[mission_obstacle-9] [WARN] [1780675324.484529101] [rb3302_mission]: 确认整格障碍：P，替换旧障碍V；原路线可走，继续执行。
[mission_obstacle-9] [INFO] [1780675327.508651861] [rb3302_mission]: 停车诊断：目标=(2.610,1.450)m，路线朝向=+90.0度；TF控制位置=(2.627,1.453)m，朝向=+82.8度，纵向剩余=-0.003m，横向=-0.017m，距目标=0.017m；AMCL最新位置=(2.624,1.421)m，朝向=+82.3度，纵向剩余=+0.029m，横向=-0.014m，消息年龄=0.244s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `f0555245-ddb7-492c-b313-e524ae4d0f96`

- Visible GOAL events in excerpt order: X, X, U, N. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/f0555245-ddb7-492c-b313-e524ae4d0f96/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780684042.029022703] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780683874.891966 → 1780684042.029023, span 167.137 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission-9] [INFO] [1780684038.705791558] [rb3302_mission]: 停车诊断：目标=(3.190,0.290)m，路线朝向=+0.0度；TF控制位置=(3.190,0.294)m，朝向=-0.7度，纵向剩余=-0.000m，横向=+0.004m，距目标=0.004m；AMCL最新位置=(3.177,0.292)m，朝向=-0.6度，纵向剩余=+0.013m，横向=+0.002m，消息年龄=0.254s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission-9] [INFO] [1780683908.486693795] [rb3302_mission]: 停车诊断：目标=(1.450,0.290)m，路线朝向=+180.0度；TF控制位置=(1.450,0.268)m，朝向=+175.4度，纵向剩余=-0.000m，横向=+0.022m，距目标=0.022m；AMCL最新位置=(1.459,0.263)m，朝向=+175.3度，纵向剩余=+0.009m，横向=+0.027m，消息年龄=0.234s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission-9] [INFO] [1780683924.255579675] [rb3302_mission]: 停车诊断：目标=(0.290,0.290)m，路线朝向=+180.0度；TF控制位置=(0.289,0.290)m，朝向=-174.9度，纵向剩余=-0.001m，横向=-0.000m，距目标=0.001m；AMCL最新位置=(0.336,0.297)m，朝向=-175.8度，纵向剩余=+0.046m，横向=-0.007m，消息年龄=0.266s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission-9] [INFO] [1780683940.591507169] [rb3302_mission]: 停车诊断：目标=(0.290,0.870)m，路线朝向=+90.0度；TF控制位置=(0.346,0.872)m，朝向=+91.4度，纵向剩余=-0.002m，横向=-0.056m，距目标=0.056m；AMCL最新位置=(0.348,0.842)m，朝向=+90.9度，纵向剩余=+0.028m，横向=-0.058m，消息年龄=0.235s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission-9] [INFO] [1780683956.173718256] [rb3302_mission]: 停车诊断：目标=(0.870,0.870)m，路线朝向=+0.0度；TF控制位置=(0.871,0.887)m，朝向=-3.0度，纵向剩余=-0.001m，横向=+0.017m，距目标=0.017m；AMCL最新位置=(0.855,0.890)m，朝向=-2.4度，纵向剩余=+0.015m，横向=+0.020m，消息年龄=0.208s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission-9] [INFO] [1780683982.235705955] [rb3302_mission]: 停车诊断：目标=(0.290,0.870)m，路线朝向=+180.0度；TF控制位置=(0.290,0.862)m，朝向=-174.3度，纵向剩余=-0.000m，横向=+0.008m，距目标=0.008m；AMCL最新位置=(0.312,0.861)m，朝向=-172.8度，纵向剩余=+0.022m，横向=+0.009m，消息年龄=0.204s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
[mission-9] [INFO] [1780683997.685824126] [rb3302_mission]: 停车诊断：目标=(0.290,0.290)m，路线朝向=-90.0度；TF控制位置=(0.293,0.288)m，朝向=-81.2度，纵向剩余=-0.002m，横向=+0.003m，距目标=0.003m；AMCL最新位置=(0.291,0.307)m，朝向=-80.4度，纵向剩余=+0.017m，横向=+0.001m，消息年龄=0.172s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `34520313-362d-46d0-9ed1-27f48a64a78f`

- Visible GOAL events in excerpt order: unavailable. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/34520313-362d-46d0-9ed1-27f48a64a78f/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780680816.986688516] [rb3302_mission]: 目标 4/4：X，已停三秒。`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780680781.861828 → 1780680816.986689, span 35.125 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission-9] [INFO] [1780680781.861828322] [rb3302_mission]: 到点停车：中心误差 x=-0.015m，y=+0.012m，odom剩余=-0.035m。
[mission-9] [INFO] [1780680813.959255497] [rb3302_mission]: 到点停车：中心误差 x=+0.017m，y=+0.013m，odom剩余=-0.022m。
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `dea73374-eab6-499d-acec-b74544f8c89c`

- Visible GOAL events in excerpt order: X, R, R, R. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/dea73374-eab6-499d-acec-b74544f8c89c/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780683386.482832797] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780683362.282611 → 1780683386.482833, span 24.200 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission-9] [INFO] [1780683377.119382592] [rb3302_mission]: 停车诊断：目标=(3.190,0.870)m，路线朝向=+90.0度；TF控制位置=(3.190,0.871)m，朝向=+90.8度，纵向剩余=-0.001m，横向=+0.000m，距目标=0.001m；AMCL最新位置=(3.190,0.857)m，朝向=+90.5度，纵向剩余=+0.013m，横向=-0.000m，消息年龄=0.232s。正剩余=尚未走到；负剩余=已越过；正横向=路线左侧。TF与AMCL为各自最新值，并非同一采样时刻。
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `85aa8ed8-e728-4eac-a4c3-5dde7a2018eb`

- Visible GOAL events in excerpt order: unavailable. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/85aa8ed8-e728-4eac-a4c3-5dde7a2018eb/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780679323.619960961] [rb3302_mission]: 目标 4/4：X，已停三秒。`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780679288.553360 → 1780679324.416839, span 35.863 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `941764cd-7237-407f-bf0e-66c0304ceaf9`

- Visible GOAL events in excerpt order: X, H, A. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/941764cd-7237-407f-bf0e-66c0304ceaf9/pasted-text.txt)
- Recorded completion: `[mission_corner_rpp-9] [INFO] [1780678165.787067132] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780678091.641402 → 1780678165.787067, span 74.146 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 1. Cells/events are reproduced below; physical placement is not independently measured. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission_corner_rpp-9] [WARN] [1780678131.775293968] [rb3302_mission]: 确认整格障碍：B，替换旧障碍None；挡住剩余路线，准备重新规划。
[mission_corner_rpp-9] [INFO] [1780678099.835322001] [rb3302_mission]: GOAL HOLD X: 已停4秒，二维误差=0.0346m；不纠正
[mission_corner_rpp-9] [INFO] [1780678122.710825221] [rb3302_mission]: GOAL HOLD H: 已停4秒，二维误差=0.0786m；不纠正
[mission_corner_rpp-9] [INFO] [1780678165.776702601] [rb3302_mission]: GOAL HOLD A: 已停4秒，二维误差=0.0469m；不纠正
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `2ac85a59-b126-4617-8b30-a0798dac2434`

- Visible GOAL events in excerpt order: X. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/2ac85a59-b126-4617-8b30-a0798dac2434/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780680047.875906414] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780680032.058391 → 1780680047.875906, span 15.818 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `84f31fe6-aa0e-443e-9c4d-abd44ddaa268`

- Visible GOAL events in excerpt order: X, X, U, N. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/84f31fe6-aa0e-443e-9c4d-abd44ddaa268/pasted-text.txt)
- Recorded completion: `[mission-9] [INFO] [1780681675.448283554] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780681508.046035 → 1780681675.448284, span 167.402 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: not present in the available extract; per-goal errors 未验证. [实机验证] (same extract)

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

#### Extract E `1a8a949e-c9e6-4574-ba39-960afd2fe9b5`

- Visible GOAL events in excerpt order: A, H, U, A. Full ordered mission / start orientation: 未验证 unless independently established in Section 5.2. [实机验证] (/home/rb3302-7/.codex/attachments/1a8a949e-c9e6-4574-ba39-960afd2fe9b5/pasted-text.txt)
- Recorded completion: `[mission_corner_rpp-9] [INFO] [1780676898.239590793] [rb3302_mission]: MISSION COMPLETE`. [实机验证] (same extract)
- Earliest to latest visible mission timestamp: 1780676820.922198 → 1780676898.239591, span 77.317 s. This is **not** a full mission duration and may include repeated pasted snippets. [代码/计算检查] (timestamp arithmetic on same real log)
- Confirmed-obstacle lines: 0. No confirmation line in this extract; this does not prove no physical obstacle was present. [实机验证] (same extract)
- Goal/stop residual evidence: listed verbatim below; do not assign an anchor stop to a requested goal. [实机验证] (same extract)

```text
[mission_corner_rpp-9] [INFO] [1780676820.922197853] [rb3302_mission]: GOAL ALIGN A: dx=+0.001 dy=+0.003m; within 3cm
[mission_corner_rpp-9] [INFO] [1780676820.949508843] [rb3302_mission]: GOAL HOLD A: 误差=0.3cm，持续复核=0.0s
[mission_corner_rpp-9] [INFO] [1780676821.959884832] [rb3302_mission]: GOAL HOLD A: 误差=0.7cm，持续复核=1.0s
[mission_corner_rpp-9] [INFO] [1780676823.074026498] [rb3302_mission]: GOAL HOLD A: 误差=1.8cm，持续复核=2.1s
[mission_corner_rpp-9] [INFO] [1780676842.860545553] [rb3302_mission]: GOAL ALIGN H: dx=-0.013 dy=-0.027m; within 3cm
[mission_corner_rpp-9] [INFO] [1780676842.888012428] [rb3302_mission]: GOAL HOLD H: 误差=2.9cm，持续复核=0.0s
[mission_corner_rpp-9] [INFO] [1780676843.893478730] [rb3302_mission]: GOAL HOLD H: 误差=2.7cm，持续复核=1.0s
[mission_corner_rpp-9] [INFO] [1780676844.904587636] [rb3302_mission]: GOAL HOLD H: 误差=2.6cm，持续复核=2.0s
[mission_corner_rpp-9] [INFO] [1780676861.890093775] [rb3302_mission]: GOAL ALIGN U: dx=+0.029 dy=-0.006m; within 3cm
[mission_corner_rpp-9] [INFO] [1780676861.916445390] [rb3302_mission]: GOAL HOLD U: 误差=2.8cm，持续复核=0.0s
[mission_corner_rpp-9] [INFO] [1780676863.024891014] [rb3302_mission]: GOAL HOLD U: 误差=1.7cm，持续复核=1.1s
[mission_corner_rpp-9] [INFO] [1780676864.029273306] [rb3302_mission]: GOAL HOLD U: 误差=2.1cm，持续复核=2.1s
[mission_corner_rpp-9] [INFO] [1780676890.499030171] [rb3302_mission]: GOAL ALIGN A: dx=-0.025 dy=+0.014m; within 3cm
[mission_corner_rpp-9] [INFO] [1780676890.528234285] [rb3302_mission]: GOAL HOLD A: 误差=2.8cm，持续复核=0.0s
[mission_corner_rpp-9] [INFO] [1780676891.559302774] [rb3302_mission]: GOAL HOLD A: 误差=2.6cm，持续复核=1.0s
[mission_corner_rpp-9] [INFO] [1780676892.812380170] [rb3302_mission]: GOAL ALIGN A: dx=-0.029 dy=+0.000m; within 3cm
[mission_corner_rpp-9] [INFO] [1780676892.841861524] [rb3302_mission]: GOAL HOLD A: 误差=2.8cm，持续复核=0.0s
[mission_corner_rpp-9] [INFO] [1780676893.912000482] [rb3302_mission]: GOAL HOLD A: 误差=2.8cm，持续复核=1.1s
[mission_corner_rpp-9] [INFO] [1780676895.146551835] [rb3302_mission]: GOAL ALIGN A: dx=-0.030 dy=-0.002m; within 3cm
[mission_corner_rpp-9] [INFO] [1780676895.174191731] [rb3302_mission]: GOAL HOLD A: 误差=3.0cm，持续复核=0.0s
[mission_corner_rpp-9] [INFO] [1780676896.272518606] [rb3302_mission]: GOAL HOLD A: 误差=2.0cm，持续复核=1.1s
[mission_corner_rpp-9] [INFO] [1780676897.382665376] [rb3302_mission]: GOAL HOLD A: 误差=1.8cm，持续复核=2.2s
```

**Speak:** “This extract proves the recorded events, but missing route and accuracy measurements remain unverified.”

## 6 已知局限和老实话

| Limitation | What the code/evidence actually says | English answer to a lecturer | Evidence |
|---|---|---|---|
| “No timeouts at all” is false | No overall mission/turn-completion deadline; startup/no-route waits can be unbounded. Freshness checks and .8 s command /1 s feedback watchdogs remain. | “There is no mission-wide deadline, but stale data and missing motor commands still cause stops.” | [代码/计算检查] (robot.py; driver; corner_rpp.py) |
| Unique passage blocked | Before confirmation, the candidate is rejected if blocking it destroys all paths to the goal. If a blockage is already retained and planning fails, wait/retry. This is not a guaranteed safe response to a truly blocked only route. | “We assumed the task leaves a detour; that assumption is a limitation when the only passage is physically blocked.” | [代码/计算检查] (experiment_obstacles.py; obstacle_node.py) |
| Restricted object geometry | Requires broad near-boundary axis-aligned face, ≥5 points, centre crossing and stable scans. Small/irregular/moving objects can be missed. | “Our detector is specialised for large regular cell obstacles, not general pedestrian or small-object avoidance.” | [代码/计算检查] (obstacle.py) |
| Only one blocked cell | New confirmed cell replaces old. Two simultaneous true obstacles are not represented together; old obstacle is not cleared just because it disappears. | “The state model remembers one obstacle because that matches our chosen task assumption.” | [代码/计算检查] (observe) |
| Nearby own-cell blind spot | Only neighbours of nearest estimated cell are considered; after entering the obstructed cell it may no longer be a candidate. | “Missing the first detection can leave a blind spot after the current-cell assignment changes.” | [实机验证] (E-L; 代码/计算检查: read_scan) |
| Map-wall filtering trade-off | 5×5 pixel rejection can remove a real obstacle’s endpoints near static walls; remaining span may fail. | “Stricter wall rejection reduces false alarms but can reduce the visible evidence for a real block.” | [代码/计算检查] (static_free; 推测: exact L cause 未验证) |
| No general collision guarantee | Static clearance and cell replan are not a swept-volume dynamic emergency-stop system. Pivots and anchor connections do not have a separate universal raw-scan collision veto. | “We demonstrated the task-specific replanning behaviour, not a general collision-avoidance guarantee.” | [代码/计算检查] (controller/obstacle call chain) |
| New objects can disturb localisation | They create scan returns inconsistent with map; actual bias depends on scene and sensor model. No quantified bias measured here. | “Unmapped objects can weaken scan-to-map agreement, but we did not isolate their localisation bias experimentally.” | [推测] (plausible AMCL effect, physical magnitude 未验证) |
| Lost localisation | No integrated custom corner-matching or active recovery manoeuvre in the final hybrid. AMCL has recovery facilities, but runtime recovery configuration/performance not verified. | “Large localisation disturbances were difficult to recover from quickly; we did not validate an autonomous recovery policy.” | [代码/计算检查] (launch/config; 实机验证: user qualitative limitation) |
| Particle concentration ≠ accuracy | Gate checks covariance, not an external reference; repeated geometry can support a wrong mode. | “A small covariance is confidence in an estimate, not proof that the estimate is correct.” | [代码/计算检查] (gate; 推测: wrong-mode risk) |
| No independent physical truth | Most residuals are TF/AMCL estimates; visual judgement is not a ruler or motion-capture measurement. | “Our physical accuracy evidence is qualitative; logged residuals are internal localisation errors.” | [实机验证] (attached diagnostics and video) |
| Final lateral goal tolerance absent | Endpoint-plane crossing can finish with cross-track offset; hold does not move again. | “Our stop criterion does not enforce zero lateral error, even when longitudinal remaining distance reaches zero.” | [代码/计算检查] (follow_corner/settle_goal) |
| Discrete actuation | Stop command is not instantaneous physical braking; feedback latency and slip remain. No measured stopping-distance model. | “The controller commands a stop at its estimated crossing, but braking and estimation delay still affect the physical result.” | [代码/计算检查] (command rule; 推测: measured contribution 未验证) |
| Rates and compute | EKF50Hz, callback requests and particle count are configuration, not guaranteed realised rates. CPU bottleneck not established. | “We measured a slow particle-cloud stream once, but did not prove whether CPU, TF or motion gating was dominant.” | [实机验证] (1.3–1.5Hz user output; 代码/计算检查: config) |
| Four-wheel mechanics | T13 twist interface; no four independent host-side wheel-speed loops or identified slip model. | “We adapted angular commands to the rover interface without claiming a full four-wheel dynamics model.” | [代码/计算检查] (base_driver.py) |
| Software can exit | Exceptions/duplicate publishers/launcher lock errors can terminate startup or mission. No global ignore-errors guarantee. | “We retain explicit failure handling; the mission is not designed to continue blindly after every fault.” | [代码/计算检查] (main, initialize, run.sh) |
| Version identity | A source snapshot and a successful build do not prove robot installed-file identity. Earlier upload had Permission denied. | “We must distinguish the saved source version from what was actually uploaded and launched.” | [实机验证] (Nov 12 terminal upload failure; 代码/计算检查: S/VERSION) |
| Completion-time claim | Video length and partial log spans do not prove all routes meet180s. Some earlier simulated/physical runs exceeded that. | “We demonstrated successful runs, but did not establish a worst-case three-minute guarantee.” | [实机验证] (selected logs/video; 代码/计算检查: simulated results) |

## 7 Q&A 题库

Answers are designed for approximately 15–30 seconds at normal speech pace. Each includes a shorter **Speak** line (≤25 words) and a Chinese memory cue. Evidence applies to the answer and short line. Do not present an unverified explanation as a measured result.

### Algorithm selection

**Q1. Why AMCL instead of SLAM?**
We already had a map. AMCL estimates pose against that map, while SLAM would also estimate or update the map. We did not run a controlled trial proving AMCL more accurate than SLAM. [代码/计算检查] (final localisation launch; comparison 未验证)
**Speak:** “We chose localisation in an existing map rather than rebuilding the map during the task.”
中文要点：已有地图，用定位；不声称比SLAM更准。

**Q2. Why use both EKF and AMCL?**
They solve different parts of the problem. EKF supplies continuous local motion from wheel velocity and IMU. AMCL anchors that motion to the static map using laser observations. [代码/计算检查] (localization.yaml)
**Speak:** “EKF supplies local motion, and AMCL supplies the map correction.”
中文要点：局部连续运动与全局地图修正互补。

**Q3. Why not use wheel odometry only?**
Odometry accumulates errors and skid steering can slip. The map-based laser observation provides an external geometric constraint. We did not measure a universal odometry drift rate. [代码/计算检查] (architecture; 推测: unmeasured drift magnitude)
**Speak:** “Wheel motion alone cannot reliably tell us where the body is after slip.”
中文要点：轮子转动不等于车身真实位移。

**Q4. Why coarse-grid A*?**
The arena goals and obstacles are naturally described by letter cells. A small graph makes cell blocking and route inspection straightforward. Fine occupancy still checks walls and localisation geometry. [代码/计算检查] (grid.py)
**Speak:** “The coarse graph matches the task while the fine map preserves wall geometry.”
中文要点：规划粗图，几何和定位细图。

**Q5. Is A* optimal here?**
With unit cardinal edge costs and Manhattan heuristic, it returns a shortest valid cell-step route. That does not minimise real travel time, turning effort or localisation risk. [代码/计算检查] (grid.py)
**Speak:** “A star is optimal for our cell-step cost, not for every physical performance measure.”
中文要点：最少格数，不一定最快。

**Q6. Why four neighbours rather than eight?**
The task route uses cardinal passages and fixed cell-centre references. Diagonal starts initialise heading but do not create diagonal planning edges. Curved motion is generated after planning. [代码/计算检查] (grid.py; robot.py; corner_geometry.py)
**Speak:** “The route graph is cardinal, but the controller can generate continuous curved motion.”
中文要点：起点八朝向≠A*八邻接。

**Q7. Why combine PID and Stanley?**
Heading error and lateral displacement are different. PID regulates orientation, while the Stanley-style term steers a parallel but displaced robot back toward the line. [代码/计算检查] (node.py)
**Speak:** “Facing the right direction is not the same as being on the centreline.”
中文要点：朝向对了也可能平行偏移。

**Q8. Is this standard Stanley?**
No. Standard Stanley produces steering angle for a steered vehicle. We adapt its lateral-error form to body angular velocity and combine it with heading PID. [代码/计算检查] (node.py; report formula)
**Speak:** “It is a Stanley-inspired yaw-rate correction, not an Ackermann steering controller.”
中文要点：借横向公式，输出角速度。

**Q9. Why RPP at corners?**
Rounded references let the robot turn while progressing, reducing repeated stop-and-pivot transitions. The physical baseline was preferred by the user, but there is no controlled proof that RPP always outperforms other controllers. [代码/计算检查] (corner modules; 实机验证: E-base/user feedback)
**Speak:** “RPP gave us a useful way to track rounded corners with curvature-based speed regulation.”
中文要点：平滑弯道适合，不夸大算法排名。

**Q10. Is your RPP the Nav2 plugin?**
No. It is our local curvature-regulated pure-pursuit implementation. We use Nav2 AMCL and map server, but not the Nav2 controller server or its complete collision/recovery machinery. [代码/计算检查] (all.launch.py; corner_rpp.py)
**Speak:** “We use the RPP method, not the full Nav2 RPP controller plugin.”
中文要点：方法相同，执行栈不同。

**Q11. Why not DWA?**
We implemented a small sampled-velocity prototype and recorded ideal-model trials. Early hardware/deployment problems prevented a clean comparison, and we focused on the simpler hybrid. We cannot claim a demonstrated intrinsic DWA failure. [代码/计算检查] (CONTROLLER_TRIALS and archived dwa.py; 实机验证: user nonmovement report)
**Speak:** “DWA was explored, but we did not complete a fair real-robot comparison.”
中文要点：不能把启动失败说成算法不行。

**Q12. Why not MPC?**
It was discussed but not selected or demonstrated as an implemented experiment. We should not describe an MPC failure or benchmark that never happened. [代码/计算检查] (conversation scope; hardware result 未验证)
**Speak:** “MPC was considered in discussion, but it was not implemented in the final development path.”
中文要点：讨论过，不等于做过。

### Parameters and formulas

**Q13. Why 58 cm instead of 60 cm?**
The assignment describes approximately 60 cm cells. The operator reported 58 cm centre spacing, so the map scale and all letter coordinates were updated together. A full surveyed arena calibration was not documented. [代码/计算检查] (H Nov 5; 实机验证: user spacing report)
**Speak:** “We used the measured working spacing rather than treating the approximate assignment dimension as exact.”
中文要点：名义60，工作值58。

**Q14. Why a 29 cm radius?**
The generator requests at most 30 cm and caps it at half each adjacent edge. Half of 58 cm is 29 cm, preventing adjacent corner fillets from overlapping. [代码/计算检查] (smooth_corners)
**Speak:** “The half-cell cap makes the actual radius twenty-nine centimetres.”
中文要点：30标称，半格限制后29。

**Q15. Why not a 20 cm radius?**
It leaves more straight path but requires higher curvature. We tried it, then restored the earlier 29 cm geometry as part of the combination the user preferred. We did not isolate a statistical optimum. [代码/计算检查] (H Nov 11; 实机验证: user rollback/success)
**Speak:** “A smaller arc is not automatically easier for a skid-steer robot to track.”
中文要点：更小半径要求更狠转向。

**Q16. Why 20 cm lookahead?**
It was restored after 5 and 10 cm trials. Short lookahead increases response to local pose variation; long lookahead can cut corners. The final value was an empirical compromise. [代码/计算检查] (H and final code; 推测: isolated sensitivity contribution)
**Speak:** “Twenty centimetres was the retained compromise, not a theoretically unique best value.”
中文要点：实测保留，不说数学最优。

**Q17. What is the curvature equation?**
For a lookahead point in body coordinates, curvature is twice its lateral coordinate divided by squared target distance. Angular velocity is linear velocity times curvature. [代码/计算检查] (pursuit)
**Speak:** “The lateral carrot position determines curvature, and velocity converts curvature into yaw rate.”
中文要点：κ=2y/L²，ω=vκ。

**Q18. How are PID gains chosen?**
The retained values are Kp1.5, Ki0.03 and Kd0.08, with 0.20 s rate filtering and bounded integral. They were adjusted during development, not identified using a formal plant model or optimisation. [代码/计算检查] (control.yaml/H)
**Speak:** “The gains are practical tuning choices with damping and anti-windup, not a formal optimal-controller result.”
中文要点：经验调参，有滤波和防积分饱和。

**Q19. Why a front reference point?**
A virtual point 10 cm ahead reflects both lateral displacement and body orientation, helping the correction anticipate the front's movement. It is not a physical steering axle. [代码/计算检查] (front_cross formula)
**Speak:** “The virtual front point includes heading in the lateral error without changing the fixed path.”
中文要点：虚拟前视参考点，不是车轴模型。

**Q20. Why 1500–2500 particles?**
Those bounds were adopted from Antonio/merged AMCL settings and retained with the successful configuration. More particles trade computation for representation capacity; we did not measure a best count. [代码/计算检查] (localisation mappings; 实机验证: successful configuration feedback)
**Speak:** “The particle bounds came from the retained reference configuration, not a measured optimal count.”
中文要点：参考方案配置，不编最优依据。

**Q21. Does sigma_hit=0.08 mean eight-centimetre accuracy?**
No. It sets the laser likelihood-field match scale. Robot pose accuracy also depends on map geometry, transforms, observations and motion uncertainty. [代码/计算检查] (AMCL configuration/method)
**Speak:** “A sensor-model width is not the robot’s measured position accuracy.”
中文要点：模型宽度≠定位精度。

**Q22. Why five startup samples and three seconds?**
They prevent departing on one transient concentrated estimate and allow a short stationary settling period. No experiment proves these exact values optimal, and low covariance can still be wrong. [代码/计算检查] (initialize; optimality 未验证)
**Speak:** “We require repeated fresh evidence instead of accepting one apparently stable update.”
中文要点：多帧确认，不保绝对正确。

**Q23. Why is the planning margin 22 cm?**
It is a static-map clearance setting, not the body radius. The 20 cm square has a 14.14 cm half-diagonal; the remaining margin is a design allowance, not a measured guarantee. [代码/计算检查] (grid.py/control.yaml; arithmetic)
**Speak:** “The clearance parameter includes a design margin beyond the nominal body geometry.”
中文要点：规划安全边界，不是车半径。

**Q24. Are all final speeds 0.24 m/s?**
No. The RPP launcher overrides cruise to 0.18 m/s. The latest 1.2-times experiment requests 0.216 m/s, while the locked report version remains at the original speed. [代码/计算检查] (mission.launch.py; current versus snapshot)
**Speak:** “The selected launch configuration overrides the generic YAML speed.”
中文要点：看有效覆盖值，别只看yaml。

### Iteration history and challenges

**Q25. What were V1, V2 and V3?**
V1 established coarse planning with straight travel and pivots. V2 explored controllers and arrived at the hybrid. V3 refined obstacle detection and replanning. The actual work overlapped. [代码/计算检查] (H)
**Speak:** “The three versions organise overlapping development work rather than three isolated releases.”
中文要点：叙事分组，不是假装完全顺序。

**Q26. Why did early turns spin indefinitely?**
The controller could wait for a heading change that was not reflected correctly in feedback. Several causes were considered. A parser defect was reproduced, but no single explanation was proven for every physical incident. [代码/计算检查] (IMU reproduction/H; 实机验证: user frozen-RViz reports)
**Speak:** “The symptom was missing usable heading progress; the exact cause was not identical or proven in every run.”
中文要点：不把所有转圈都归咎AMCL。

**Q27. Did timeout protection fix localisation?**
No. A timeout or mismatch guard can stop a bad motion but cannot repair its sensor or estimation cause. That distinction motivated further inspection rather than declaring the problem solved. [代码/计算检查] (H guard addition/removal)
**Speak:** “Stopping on a fault is protection, not a localisation repair.”
中文要点：保护不等于修复。

**Q28. What was wrong with the sensor offset?**
The encoded laser position disagreed with its position relative to the body. Changing that transform shifted the interpreted base position. It was first corrected forward, then refined to 1.5 cm. [代码/计算检查] (xacro/H; 实机验证: operator centre feedback)
**Speak:** “The localisation reference must represent the body centre consistently with the sensor mount.”
中文要点：定位点与车体中心的几何关系。

**Q29. Why abandon timed turns?**
A fixed duration assumes repeatable angular response. A brief calibration was also tried, but slip and delayed observations make an exact-angle claim unjustified. Final RPP uses pose and rate feedback. [代码/计算检查] (H; final code)
**Speak:** “Time alone cannot guarantee an angle when the rover’s physical turning response changes.”
中文要点：时间转向依赖稳定转速。

**Q30. Why did mission_arc fail?**
It still had an exit-heading condition that could stop translation and pivot. The operator reported insufficient turning and wall rubbing. The precise physical cause was not isolated, so we should not claim one proven formula error. [实机验证] (E 6682d9d0-9953-4854-b6b6-e53115059630; 代码/计算检查: arc source/H)
**Speak:** “The first arc design did not achieve continuous reliable cornering, and its exit check could still cause a pivot.”
中文要点：出口补转是代码行为，滑移原因未隔离。

**Q31. What happened to the custom AMCL model?**
It addressed a locally reproduced reverse prediction issue, then required another correction for near-turn bias. Robot reverse behaviour was not conclusively fixed. Final configuration restored the official model. [代码/计算检查] (H/G review; 实机验证: repeated reverse complaints)
**Speak:** “The custom model was an experiment, not part of the final successful localisation configuration.”
中文要点：插件试验后回退官方。

**Q32. Why remove goal corrections?**
Small correction motions sometimes worsened observed position because pose variation and physical motion were comparable to the requested adjustment. We retained one approach and a four-second stationary hold. [实机验证] (user feedback; 代码/计算检查: settle_goal)
**Speak:** “Repeated small corrections were not reliably better than holding the first completed approach.”
中文要点：纠偏反而更差，保留一次到点。

**Q33. Why restore original speeds after doubling?**
The user preferred the overall behaviour at baseline speed. The 1.5 and 2-times trials were not controlled experiments demonstrating a precise failure threshold. Startup convergence was retained separately. [代码/计算检查] (H Nov 11; 实机验证: user feedback)
**Speak:** “We kept the useful startup change while reverting the speed experiment.”
中文要点：解耦保留与回退。

### Results and limitations

**Q34. What accuracy did you achieve?**
Some logged held TF residuals were small, but those are internal estimates. We did not collect independent per-goal physical ground truth. The defensible physical statement is that stops looked generally close to centres in successful trials. [实机验证] (E-base/video/user; ground truth 未验证)
**Speak:** “We report estimated residuals separately from the visual physical assessment.”
中文要点：没有独立厘米精度结论。

**Q35. Was the video mission 132.77 seconds?**
The file is 132.77 seconds long. Its full start-to-completion coverage and exact letter-route mapping have not been established from the available evidence, so it is not automatically an official mission time. [代码/计算检查] (video metadata; whole-mission claim 未验证)
**Speak:** “That number is the recording duration, not an independently verified complete-run stopwatch measurement.”
中文要点：视频长≠任务总时长。

**Q36. Did obstacle replanning work physically?**
Yes, the retained baseline extract records B confirmed on the route, a safe-centre connection and continued execution to completion. It does not prove every obstacle arrangement is detected. [实机验证] (E-base)
**Speak:** “The robot log demonstrates a real obstacle-triggered detour, but not universal detection reliability.”
中文要点：有真实成功，不保证全场景。

**Q37. Can you guarantee completion within three minutes?**
No worst-case guarantee was established. Routes, detours and localisation waits change time; some earlier tests exceeded the limit. The successful evidence must be reported with its exact timing boundary. [代码/计算检查] (simulation results; 实机验证: partial long-run logs)
**Speak:** “We have successful demonstrations, not a proven worst-case time bound.”
中文要点：不能拿局部时长做全局保证。

**Q38. How realistic was Gazebo?**
It used real localisation software and the map, with simulated sensors and dynamics. Ground truth evaluated errors, but wheel response and friction were not calibrated to the robot. [代码/计算检查] (simulation RESULTS)
**Speak:** “Gazebo tested integration and geometry, while real hardware remained necessary for slip and turning behaviour.”
中文要点：软件真实，物理模型近似。

**Q39. Did you quantitatively prove smoother motion?**
No acceleration or jerk metric was collected for the final physical comparison. Smoothness is an operator/video assessment supported by observed behaviour, not a measured percentage improvement. [实机验证] (user/video; quantitative metric 未验证)
**Speak:** “Our smoothness result is qualitative rather than a measured jerk reduction.”
中文要点：丝滑是观察，不编百分比。

### What if…?

**Q40. What if a new obstacle blocks an unrelated branch?**
It may replace the remembered cell after confirmation, but the mission only replans if it obstructs the remaining route. This prevents needless interruptions for unrelated observations. [代码/计算检查] (observe/blocks_route)
**Speak:** “Detection updates state; route relevance decides whether replanning is necessary.”
中文要点：更新障碍≠立刻改路。

**Q41. What if two obstacles exist?**
Only one confirmed cell is stored. A new one replaces the previous one, so the planner cannot jointly represent two persistent dynamic obstacles. This is a task-assumption limitation. [代码/计算检查] (obstacle.py)
**Speak:** “The current state representation is limited to one remembered obstacle cell.”
中文要点：两个同时存在可能遗忘旧的。

**Q42. What if the only passage is blocked?**
The detector currently rejects a candidate that makes the goal unreachable. If an existing block leaves no route, the mission waits and retries. Therefore safe handling of a truly blocked unique passage is not guaranteed. [代码/计算检查] (scanner veto and route loop)
**Speak:** “Our reachability assumption must not be mistaken for a general safety guarantee.”
中文要点：无路否决候选是明确局限。

**Q43. What if the obstacle is removed?**
The current blocked cell is not cleared simply because no candidate is seen. It remains until another confirmed cell replaces it. Earlier versions had clearing/expiry rules, but those are not the final behaviour. [代码/计算检查] (observe; H)
**Speak:** “Absence of a detection does not automatically clear the stored obstacle.”
中文要点：无检测不代表自动解除封格。

**Q44. What if the robot is pushed or kidnapped?**
Localisation may become inconsistent or converge incorrectly. We did not integrate or validate a custom active recovery sequence. AMCL facilities alone are not a guarantee of quick recovery under our configuration. [代码/计算检查] (final launch/config; 实机验证: user limitation)
**Speak:** “Large unexpected disturbances can be difficult to recover from quickly.”
中文要点：可恢复能力不等于当前一定恢复。

**Q45. What if localisation stops updating?**
Freshness checks command zero and wait when pose data is unusable. A frozen value with a fresh timestamp is a different fault; the removed turn guard would have addressed only a particular mismatch symptom. [代码/计算检查] (robot.py/H)
**Speak:** “Fresh timestamps and correct changing values are different requirements.”
中文要点：时间新不代表值对。

**Q46. What if the robot is sideways at a goal?**
The final arrival test is longitudinal crossing on the last edge. It can stop with lateral error, then hold without correction. There is no final mandatory yaw tolerance at the goal. [代码/计算检查] (follow_corner/settle_goal)
**Speak:** “The final goal rule does not enforce exact lateral position or a prescribed final heading.”
中文要点：到点平面判据，不做姿态精调。

**Q47. What if a smooth arc is too close to a wall?**
The static-map clearance test rejects the arc and keeps the original grid corner. RPP may then pivot if the target bearing is large. It does not secretly keep shrinking the radius. [代码/计算检查] (smooth_corners/follow_corner)
**Speak:** “Rejected arcs fall back to the grid reference, so continuous turning is not guaranteed at every corner.”
中文要点：不安全弧线会退回直角。

**Q48. What if the start points northeast?**
The compass maps northeast to 45 degrees. In the Nov 12 variant, after convergence the robot pivots to the first planned cardinal direction, then starts the route. This does not change obstacle neighbours. [代码/计算检查] (current robot.py/corner_rpp.py)
**Speak:** “Diagonal starting orientation is handled before travel without changing the four-neighbour planner.”
中文要点：先收敛再转正，不改避障方向。

**Q49. Why did L fail to be detected in the faster run?**
R included L as an open neighbour, but candidate lists stayed empty despite remaining non-wall points. Confirmation was never reached in logged samples. The exact failed geometric condition is unknown, so speed alone is not established as the cause. [实机验证] (E-L)
**Speak:** “The evidence points to candidate rejection before stability confirmation.”
中文要点：先漏候选，不是只差等0.5秒。

**Q50. Does lowering confirmation to 0.3 seconds solve L?**
Not by itself if the candidate remains empty. The new trial also lowers visible width to 20 cm and requires three fresh frames. Its hardware effectiveness has not yet been demonstrated. [代码/计算检查] (current changes; physical validation 未验证)
**Speak:** “The relaxed detector is a targeted trial, not a confirmed fix.”
中文要点：最新试验未验证。

### Teamwork and provenance

**Q51. What did each member do?**
All three discussed solutions. DENG YUNTIAN focused on implementation and algorithm optimisation, ZENG JINWEN on debugging and fixes, and PIA TALESARA on observing problems and suggesting improvements. Individual feature ownership beyond those stated roles is not documented. [代码/计算检查] (user's explicit contribution statement)
**Speak:** “We shared solution ideas, with different emphasis on implementation, debugging and operational feedback.”
中文要点：按真实角色，不编某人独写某模块。

**Q52. What did you learn from Antonio’s code?**
We inspected its navigation behaviour and feedback ideas, then wrote our own task-specific movement. We adopted its AMCL parameter mapping, but the final controller does not import its RPP implementation or calibration file. [代码/计算检查] (H/TWO_TURN_VERSIONS/final source)
**Speak:** “We acknowledge the reference method and configuration without claiming those established ideas are new.”
中文要点：方法借鉴，AMCL参数采用，控制自己实现。

**Q53. What was your own main contribution?**
Use your real role: for ZENG JINWEN, debugging and problem fixes with repeated physical feedback and shared solution discussion. Do not claim sole authorship of every algorithm or attribute unlogged commits to another member. [代码/计算检查] (user role statement; detailed personal commit attribution 未验证)
**Speak:** “My focus was diagnosing observed failures and helping turn the test feedback into corrections.”
中文要点：讲实际debug工作，别夸大个人作者。

**Q54. What is the strongest lesson?**
A mathematically reasonable controller can still fail when sensor geometry, timing or the physical motion assumptions are wrong. Correcting the laser reference and separating code evidence from real behaviour mattered as much as selecting algorithms. [代码/计算检查] (chronology; 实机验证: offset/run feedback)
**Speak:** “Correct geometry and trustworthy feedback are prerequisites for meaningful controller tuning.”
中文要点：先坐标和反馈，再调控制。

**Q55. What would you improve next?**
A proposed next step is timestamp-aligned recording with independent physical reference, followed by isolated tests of stopping, turning and obstacle misses. That would separate localisation error from tracking error before adding more controller complexity. [推测] (proposal, not implemented)
**Speak:** “I would measure the error sources separately before adding another layer of control.”
中文要点：后续建议，不伪装已实现。

## 8 团队分工

The authoritative contribution description is the user's explicit statement in this conversation, not a reconstructed Git authorship analysis. All three contributed ideas for solving encountered problems. [代码/计算检查] (conversation statement)
**Speak:** “All three members contributed ideas throughout the iterative problem-solving process.”

| Member | Confirmed role | Concrete way to explain it without inventing authorship | Evidence boundary / Speak |
|---|---|---|---|
| DENG YUNTIAN | Implement ideas; optimise algorithms | Describe implementation/optimisation work within the shared controller and obstacle iteration process. Do not assert exclusive authorship of RPP, EKF or A* without another record. | [代码/计算检查] (user role statement; feature-by-feature attribution 未验证) “Deng focused on implementing proposed solutions and improving the algorithms.” |
| ZENG JINWEN | Debug issues and make fixes | Discuss the observed stopping, turning and obstacle problems and the debugging cycle. Chat contains repeated operator feedback, but does not independently label every message by a named author. | [代码/计算检查] (user role statement; exact per-fix authorship 未验证) “Zeng focused on debugging failures and fixing issues revealed by testing.” |
| PIA TALESARA | Find problems; give suggestions | Explain operational observation and proposed improvements. Do not invent that Pia measured a particular sensor offset or wrote a particular module. | [代码/计算检查] (user role statement; individual dated examples 未验证) “Pia focused on identifying problems during operation and suggesting improvements.” |

A safe concrete **team** example is the reference-point issue: observed early body stopping → discuss sensor/body relationship → change transform → physical retest. Another is obstacle false blocking → inspect map filtering → change candidate logic → observe replanning. These are project examples, not proven person-specific commit assignments. [实机验证] (offset and obstacle logs; 代码/计算检查: H)
**Speak:** “We used observation, discussion, implementation and retesting as a shared cycle.”

## 9 术语表

Each entry gives two plain-English explanatory sentences, followed by a short speaking line. Definitions are explanatory rather than claims that every feature is implemented.

### EKF

Extended Kalman Filter predicts a changing state and corrects it using uncertain measurements. It linearises nonlinear relations locally; in our project it produces local odometry. [代码/计算检查] (C/localization.yaml; physical magnitude where applicable 未验证)
**Speak:** “EKF combines motion measurements while accounting for their uncertainty.”

### IMU

An Inertial Measurement Unit measures inertial motion quantities such as angular velocity and acceleration. Our filter selects its yaw and yaw rate rather than treating it as an absolute position sensor. [代码/计算检查] (C/localization.yaml; physical magnitude where applicable 未验证)
**Speak:** “The IMU supplies orientation and turn-rate information, not direct global position.”

### Odometry

Odometry estimates how the robot moved relative to a local starting frame. Wheel-based estimates can drift or disagree with body travel when wheels slip. [代码/计算检查] (robot.py and localisation configuration; physical magnitude where applicable 未验证)
**Speak:** “Odometry measures accumulated local motion and can drift.”

### AMCL

Adaptive Monte Carlo Localisation maintains multiple candidate poses in a known map. Odometry predicts motion and laser observations help choose the better candidates. [代码/计算检查] (C/localization.yaml; official AMCL reference; physical magnitude where applicable 未验证)
**Speak:** “AMCL estimates where we are in an existing map.”

### Particle

A particle is one candidate position and heading with a weight. Many particles describe uncertainty or competing explanations of a scan. [代码/计算检查] (AMCL method; physical magnitude where applicable 未验证)
**Speak:** “Each particle is a possible robot pose.”

### Covariance

Covariance describes estimated uncertainty and correlations between state components. A small value means a concentrated estimate, not independently proven accuracy. [代码/计算检查] (corner_rpp.py initialize; physical magnitude where applicable 未验证)
**Speak:** “Covariance expresses estimated uncertainty, not guaranteed correctness.”

### Likelihood field

A likelihood field associates map locations with distance to known obstacles. Scan endpoints closer to expected obstacles generally support a pose more strongly. [代码/计算检查] (AMCL configured laser model; physical magnitude where applicable 未验证)
**Speak:** “The likelihood field scores how well laser endpoints agree with mapped walls.”

### Resampling

Resampling selects a new particle population according to particle weights. Strongly supported regions usually receive more representation, while diversity can be lost. [代码/计算检查] (AMCL method; physical magnitude where applicable 未验证)
**Speak:** “Resampling concentrates particles where the observations fit better.”

### A*

A star searches using cost already spent plus estimated remaining cost. Our implementation uses unit cardinal steps and a Manhattan heuristic. [代码/计算检查] (grid.py; physical magnitude where applicable 未验证)
**Speak:** “A star balances travelled cost with a lower estimate of what remains.”

### Heuristic

A heuristic estimates the remaining search cost. Manhattan distance never overestimates cardinal cell-step distance in our graph. [代码/计算检查] (grid.py; physical magnitude where applicable 未验证)
**Speak:** “The heuristic guides search without pretending to know the complete route.”

### Manhattan distance

Manhattan distance adds absolute differences along two grid axes. Our implementation divides metre differences by the 0.58 m cell spacing. [代码/计算检查] (grid.py; physical magnitude where applicable 未验证)
**Speak:** “Manhattan distance counts the minimum cardinal steps before considering walls.”

### PID

Proportional, Integral and Derivative control combines present error, accumulated error and damping. Our derivative action uses filtered measured yaw rate on a fixed straight reference. [代码/计算检查] (node.py; physical magnitude where applicable 未验证)
**Speak:** “PID controls heading with immediate response, bias correction and damping.”

### Stanley

Standard Stanley combines heading and lateral error to command steering angle. Our adaptation uses the lateral-error form to contribute body yaw rate. [代码/计算检查] (node.py; physical magnitude where applicable 未验证)
**Speak:** “Our Stanley-style term corrects sideways displacement from the reference line.”

### Cross-track error

Cross-track error is signed sideways distance from the reference path. It differs from heading error and from distance remaining to the goal. [代码/计算检查] (node.py; physical magnitude where applicable 未验证)
**Speak:** “Cross-track error tells us how far sideways we are from the line.”

### Pure pursuit

Pure pursuit chooses a forward point on a path and curves toward it. The point is a geometric steering reference, not necessarily a stopping target. [代码/计算检查] (corner_geometry.py; physical magnitude where applicable 未验证)
**Speak:** “Pure pursuit steers toward a point ahead on the reference path.”

### RPP

Regulated Pure Pursuit adds speed regulation to pursuit tracking. Our version regulates by curvature and does not include all Nav2 RPP features. [代码/计算检查] (corner_rpp.py; physical magnitude where applicable 未验证)
**Speak:** “RPP adjusts speed while following the pursuit curvature.”

### Lookahead

Lookahead is the reference-path distance from projected progress to the tracking point. Smaller values react more locally but can increase sensitivity to pose variation. [代码/计算检查] (corner_geometry.py; tuning rationale; physical magnitude where applicable 未验证)
**Speak:** “Lookahead controls how far ahead the tracker aims.”

### Curvature

Curvature measures how sharply a path turns and has units of inverse metres. For a circle it equals one divided by radius, with sign indicating turn direction. [代码/计算检查] (pursuit and omega=v*kappa; physical magnitude where applicable 未验证)
**Speak:** “Curvature converts forward speed into the required yaw rate.”

### Skid-steer

A skid-steer vehicle turns by driving its two sides differently while tyres slip against the floor. The actual turning centre can move and need not match ideal differential-drive geometry. [代码/计算检查] (hardware description; physical magnitude unmeasured; physical magnitude where applicable 未验证)
**Speak:** “Skid steering makes tyre-floor interaction important during turns.”

### Differential drive

Ideal differential-drive kinematics relates body motion to left and right wheel speeds. It is a useful command abstraction but not a complete four-wheel slip model. [代码/计算检查] (base_driver interface; ideal kinematic explanation; physical magnitude where applicable 未验证)
**Speak:** “Differential-drive equations describe an ideal relationship between wheel speeds and body motion.”

### TF

TF is ROS transform infrastructure connecting coordinate frames over time. A timestamp matters because the robot and its sensor frames move. [代码/计算检查] (robot.py; scan-time transform; physical magnitude where applicable 未验证)
**Speak:** “TF lets us compare sensor and robot positions in a common frame.”

### map frame

The map frame is the global reference associated with the static arena map. AMCL estimates the correction between map and locally accumulated odometry. [代码/计算检查] (localization.yaml; physical magnitude where applicable 未验证)
**Speak:** “The map frame anchors the robot to the known arena.”

### odom frame

The odom frame tracks continuous local motion and may drift globally. EKF supplies the transform from odom to the robot body frame. [代码/计算检查] (localization.launch.py; physical magnitude where applicable 未验证)
**Speak:** “Odom is locally continuous but not globally drift-free.”

### base_link

base_link is the robot body reference frame used by control. Its relationship to the laser must match the physical mounting geometry. [代码/计算检查] (robot_model and robot.py; physical magnitude where applicable 未验证)
**Speak:** “Base link defines the body reference we are trying to position.”

### Occupancy grid

An occupancy grid labels small map cells as free, occupied or unknown. It is distinct from our larger letter-cell planning graph. [代码/计算检查] (grid.py and map YAML; physical magnitude where applicable 未验证)
**Speak:** “The occupancy map stores wall geometry at a finer scale than the route graph.”

### Inflation / clearance

Inflation expands map obstacles by a design distance before checking paths. It helps reserve space around the robot but is not proof of collision-free tracking. [代码/计算检查] (grid.py stop_distance; physical magnitude where applicable 未验证)
**Speak:** “Inflation provides planning clearance around known obstacles.”

### Replanning

Replanning searches for a new route after relevant state changes. Our mission only interrupts when the confirmed blocked cell affects the remaining path. [代码/计算检查] (obstacle_node.py; physical magnitude where applicable 未验证)
**Speak:** “Replanning changes the route when the current route becomes blocked.”

### Endpoint plane

An endpoint plane is the line through a two-dimensional goal perpendicular to the approach direction. Crossing it proves longitudinal progress, not zero lateral error. [代码/计算检查] (corner_rpp.py arrival; physical magnitude where applicable 未验证)
**Speak:** “The endpoint plane checks how far along the approach we have travelled.”

### Anti-windup

Anti-windup prevents an integral controller accumulating excessive correction while its output is saturated. Our heading and rate loops bound and condition their integral updates. [代码/计算检查] (node.py and turn_feedback.py; physical magnitude where applicable 未验证)
**Speak:** “Anti-windup stops the integrator from fighting an unreachable command.”

### Watchdog

A watchdog detects missing timely input or commands and triggers a defined response. Our base driver sends stop commands after command or feedback expiry. [代码/计算检查] (waverover_driver.py; physical magnitude where applicable 未验证)
**Speak:** “The watchdog stops the base when control communication becomes stale.”

### Ground truth

Ground truth is a separate reference used to evaluate an estimate. A TF residual measured by the same localisation system is not independent physical truth. [代码/计算检查] (results methodology; physical magnitude where applicable 未验证)
**Speak:** “Independent ground truth is needed to measure absolute accuracy.”

### Nav2

Nav2 is a ROS 2 navigation framework containing localisation, planning, control and behaviour components. Using its AMCL does not mean our final mission uses its whole navigation stack. [代码/计算检查] (all.launch.py; physical magnitude where applicable 未验证)
**Speak:** “We use selected Nav2 components rather than its complete navigation pipeline.”

### DWA

Dynamic Window Approach samples reachable velocity commands and evaluates predicted motion. Our prototype used a short horizon and heuristic tracking costs. [代码/计算检查] (archived controllers/dwa.py; physical magnitude where applicable 未验证)
**Speak:** “DWA chooses among feasible short-term velocity candidates.”

### SLAM

Simultaneous Localisation and Mapping estimates a map and pose together. Our final online navigation uses a supplied derived map instead of a SLAM node. [代码/计算检查] (final launch; physical magnitude where applicable 未验证)
**Speak:** “SLAM builds the map while localising, which differs from our final fixed-map setup.”

### Yaw

Yaw is planar rotation about the vertical axis. Heading error is wrapped to avoid a discontinuity between plus and minus pi. [代码/计算检查] (grid.py wrap; robot.py yaw; physical magnitude where applicable 未验证)
**Speak:** “Yaw describes which way the robot faces in the plane.”

### Dynamic replanning versus dynamic obstacle

Dynamic replanning means updating a route during execution. The object causing that update can be a stationary box that was absent from the map. [代码/计算检查] (obstacle policy; physical magnitude where applicable 未验证)
**Speak:** “A static new box can still trigger dynamic replanning.”

## 10 报告和幻灯片里的每一句话是否属实

### 10.1 Scope and most important corrections

This audit covers the complete extracted body text of the current Word report, every visible PowerPoint text item, and all speaker-note text. Layout/page breaks are not being redesigned. Titles and names are checked as labels, not experimental claims. Original report/slides are left unchanged. The audit refers to the frozen November 11 baseline; it does not silently substitute the later trial parameters. [代码/计算检查] (extraction from R and T)

The principal corrections are:

1. **Video timing:** replace a suggestion of a measured complete under-three-minute mission with “The supplied recording is 132.77 s long; the complete mission timing boundary is not independently established.” [代码/计算检查] (video metadata; whole mission 未验证)
   **Speak:** “The recording duration and the full mission duration must be distinguished.”
2. **Physical causality:** change “tyre slip caused the offset” to “The observed body shift is consistent with skid-steer slip; its contribution was not separately measured.” [实机验证] (body shift reported; 推测: isolated cause)
   **Speak:** “We observed body shift, but did not measure how much each physical effect contributed.”
3. **Accuracy:** keep “visually close to centres” and explicitly label TF residuals as estimates; do not promise zero error, 3 cm accuracy or ruler-verified accuracy. [实机验证] (logs/video)
   **Speak:** “Our estimated residuals are not independent accuracy measurements.”
4. **Controller comparison:** “explored” is defensible; “proved best” or “DWA failed because it was too slow” is not. [代码/计算检查] (archived trials; equal-condition benchmark 未验证)
   **Speak:** “We selected a working combination without claiming a controlled ranking of all controllers.”
5. **Obstacle assumptions:** disclose one-cell memory and unreachable-candidate rejection; the detector is not general collision avoidance. [代码/计算检查] (obstacle code)
   **Speak:** “The obstacle method relies on the course assumptions and has known blind spots.”
6. **Startup:** covariance gates indicate concentration, not guaranteed correct location; no forced-departure timeout does not mean no watchdogs. [代码/计算检查] (initialize/driver)
   **Speak:** “Convergence checks are useful but are not a certificate of localisation accuracy.”
7. **Version labels:** report slide 19 values 30 cm/0.5 s/four scans describe the frozen baseline; 20 cm/0.3 s/three scans describe the unverified November 12 detector trial. [代码/计算检查] (snapshot versus working files)
   **Speak:** “These two parameter sets belong to different versions.”
8. **Goal handling:** goal-cell RPP does not mean a repeated precision-alignment stage; stopping is longitudinal crossing and hold-only. [代码/计算检查] (follow_corner/settle_goal)
   **Speak:** “The final approach follows RPP once, then holds without renewed centring.”

### 10.2 Report body audit — complete extracted text

Paragraph identifiers R1–R77 refer to extraction order, not page numbers. A paragraph may contain several sentences; the assessment explicitly covers all sentences in it, with questionable assertions qualified below. Mathematical expressions are kept exactly as extracted; missing superscript appearance in plain text is not evidence of a broken Word equation.

#### R1 — LABEL

> RB3302 Autonomous Navigation Project Report

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R2 — LABEL

> DENG YUNTIAN  •  PIA TALESARA  •  ZENG JINWEN

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R3 — LABEL

> 1 Task and system overview

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R4 — QUALIFY

> We developed a robot navigation system for a structured arena containing fixed walls and large obstacles. The arena has 24 lettered cells arranged in six columns and four rows, with approximately 0.58 m between adjacent cell centres. The robot must visit the requested goals in order, stop at each goal and find another route when a new obstacle blocks its path. The task also requires completion within three minutes.

**Audit:** The arena and ordered-goal task are supported. State that 58 cm is the operator-reported working spacing; the assignment says approximately 60 cm. The three-minute statement is a requirement, not a demonstrated bound. [代码/计算检查] (D/Project_Description.pdf; C/course.yaml; H; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R5 — KEEP

> The robot uses a four-wheel skid-steer base: different speeds on the left and right sides produce a turn. A laser scanner measures the surrounding geometry, wheel odometry estimates motion, and an inertial measurement unit (IMU) supplies orientation and angular velocity. ROS 2 connects these inputs to localisation, planning and control.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (all.launch.py; hardware/task description; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R6 — KEEP

> At startup, the robot waits for localisation to settle. It then plans a route, follows it using straight-line or curved-path control, and checks incoming scans for obstacles. A confirmed obstruction updates the planner when necessary. At each goal, the robot stops for four seconds before continuing to the next destination.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (all.launch.py; hardware/task description; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R7 — LABEL

> 2 Final methods

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R8 — LABEL

> 2.1 Localisation and startup

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R9 — KEEP

> An extended Kalman filter (EKF) combines forward velocity from the base with IMU yaw and yaw rate. It predicts the robot state and corrects that prediction using the measurements, producing continuous local odometry. This estimate is useful for short-term motion, but errors can accumulate as the robot travels.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (localization.yaml; robot.py; standard filter explanation; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R10 — KEEP

> Adaptive Monte Carlo Localisation (AMCL) corrects the robot pose against the known map. Its particles represent possible positions and headings. Odometry moves the particles, laser-to-map agreement determines their weights, and resampling concentrates them around better explanations of the scan. The controller obtains its map-frame pose through the transform chain, combining AMCL’s map correction with the EKF odometry.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (localization.yaml; robot.py; standard filter explanation; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R11 — QUALIFY

> Before departure, the robot remains stationary for at least three seconds. We require five consecutive fresh AMCL estimates with position standard deviation no greater than 0.05 m and heading standard deviation no greater than 5°. These covariance checks indicate a concentrated estimate and prevent departure while the initial particle distribution is still unsettled.

**Audit:** The numeric gate is correct, using largest XY covariance eigenvalue. Replace “prevent departure ... unsettled” with “gate departure on estimated concentration”; a confidently wrong mode may pass. [代码/计算检查] (corner_rpp.py initialize; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R12 — LABEL

> 2.2 Coarse grid planning

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R13 — KEEP

> We keep a fine occupancy map for localisation and wall geometry, and a separate coarse graph for route planning. Cell centres are calculated in advance:

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (grid.py; course.yaml; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R14 — KEEP

> xi = x0 + i d,     yj = y0 + j d,     d = 0.58 m

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (grid.py; course.yaml; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R15 — KEEP

> Here i and j are column and row indices. Each free cell connects to its four cardinal neighbours where no fixed wall blocks the connection. A* selects nodes using f(n) = g(n) + h(n), where g is the accumulated cost and h = |iₙ − iɢ| + |jₙ − jɢ| is the Manhattan distance to the goal, in cell steps. A newly blocked cell is excluded from the search. The resulting sequence is divided into straight sections, corners and the final approach. This representation matches the arena and makes obstacle updates straightforward.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (grid.py; course.yaml; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R17 — LABEL

> 2.3 Motion control

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R18 — KEEP

> The final controller uses heading PID with a Stanley-style lateral correction on ordinary straight sections. Regulated Pure Pursuit (RPP) handles corners and the goal cell. Both controllers use the same planned reference, so a tracking error does not redefine the centreline.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (node.py; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R19 — LABEL

> Heading and lateral correction on straight sections

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R20 — KEEP

> Let θᵣ be the reference direction and θ the robot heading. The heading error is wrapped to the interval [−π, π]. For a reference line starting at (xₛ, yₛ), the signed lateral error is calculated from the robot position (x, y):

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (node.py; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R21 — KEEP

> eθ = wrap(θr − θ)

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (node.py; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R22 — KEEP

> ey = −sin(θr)(x − xs) + cos(θr)(y − ys)

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (node.py; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R23 — QUALIFY

> Positive lateral error means the robot is left of the line. We use a point l_f = 0.10 m ahead of the body reference to account for the direction in which the front of the robot is moving:

**Audit:** 0.10 m is a virtual control point. Avoid implying a measured front-axle model or a four-wheel tyre model. [代码/计算检查] (node.py front_cross; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R24 — KEEP

> ef = ey − lf sin(eθ)

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (node.py; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R25 — LABEL

> The commanded yaw rate combines heading regulation and lateral correction:

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R26 — QUALIFY

> ω = Kp eθ + Ki ∫ eθ dt − Kd θ̇ − Ks arctan(k ef / (v + vs))

**Audit:** Correct high-level requested yaw law. Add that filtering, anti-windup, saturation and a measured-rate trim follow; this is not necessarily the final actuator command. [代码/计算检查] (node.py; turn_feedback.py; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R27 — QUALIFY

> The proportional term responds to heading error; the integral term compensates for persistent bias; and the derivative term, implemented using filtered yaw rate, damps rotation. The final term steers the robot back towards the centreline. Its negative sign makes a leftward displacement produce a rightward correction. The softening speed vₛ prevents excessive correction at low speed, while k and Kₛ set the lateral response.

**Audit:** Term roles and signs are correct. “Compensates persistent bias” and “damps rotation” describe design intent, not individually measured improvements. [代码/计算检查] (node.py; isolated gain effect 未验证; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R28 — KEEP

> Standard Stanley control produces a front-wheel steering angle. Our implementation uses its lateral-error form to produce body angular velocity, which suits the left–right differential drive interface. It is an adaptation of the control law, rather than a detailed model of four-wheel tyre slip. Heading correction and lateral correction must work together: facing the correct direction alone does not remove an existing sideways offset.

**Audit:** All sentences correctly distinguish a Stanley-inspired yaw-rate adaptation from a full tyre-slip model. [代码/计算检查] (node.py/base_driver.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R29 — LABEL

> RPP at corners and in the goal cell

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R30 — KEEP

> The corner reference is rounded before tracking. RPP chooses a point ahead along that path and expresses it in robot coordinates as (x_L, y_L). The required curvature and yaw rate are:

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (corner_geometry.py; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R31 — KEEP

> κ = 2yL / (xL2 + yL2),     ω = vκ

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (corner_geometry.py; corner_rpp.py; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R32 — QUALIFY

> The sign of y_L determines the turning direction. Curvature-based speed regulation helps the robot follow tighter sections. The final lookahead is 0.20 m. A longer lookahead can cut corners or delay return to the reference; a very short one makes steering more sensitive to pose fluctuations. We selected the setting from the robot’s actual tracking behaviour.

**Audit:** 0.20 m lookahead and curvature regulation are supported. Sensitivity/corner-cutting statements are trade-offs, not isolated project measurements; say retained after trials rather than optimal. [代码/计算检查] (corner_rpp.py; H Nov 11; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R33 — QUALIFY

> The same RPP approach is used in the goal cell. Arrival is judged from progress along the final reference direction: the robot stops once the remaining longitudinal distance reaches zero or becomes negative. It then holds for four seconds while localisation continues to update. We removed repeated small alignment movements because they often introduced more error than they removed. This stopping rule does not require the lateral error to be exactly zero.

**Audit:** Correct baseline rule; add last-edge condition. Repeated corrections worsening performance is operator feedback. No lateral-zero guarantee is correctly disclosed. [代码/计算检查] (corner_rpp.py; user goal-correction feedback; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R35 — LABEL

> 2.4 Obstacle detection and replanning

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R36 — QUALIFY

> The task obstacles are large blocks occupying a whole cell. The laser usually sees the near face, rather than the full outline, so detection is based on a broad surface near the expected cell boundary. We first remove returns associated with existing map walls. We then inspect only the four adjacent cells that are connected to the current cell by an open passage.

**Audit:** Near-face/four-open-neighbour rule is correct. Whole-cell large-block geometry is the operator task specification, not a survey of every assignment obstacle. [代码/计算检查] (experiment_obstacles.py; user obstacle description; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R37 — KEEP FOR BASELINE

> A candidate surface must have at least 0.30 m of visible lateral span, cover both sides of the cell centreline, and lie within ±0.20 m of the near boundary. Its direction must be within 10° of that boundary. Straight-line fitting rejects scattered points, and the same candidate must persist for at least four fresh scans over at least 0.5 s before confirmation. The 0.30 m requirement refers to the visible surface, not the physical obstacle width; the confirmed obstacle blocks the whole cell.

**Audit:** All numeric values match frozen success. Add ≥5 points and ≤2.5 cm fit RMS if explaining exact implementation. Do not apply these values to November 12 trial. [代码/计算检查] (S obstacle.py/control.yaml; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R38 — QUALIFY

> We retain one latest confirmed obstacle rather than accumulating old blocked cells. A new observation updates this state, but only triggers A* when it blocks the remaining route. Replanning uses a safe grid-centre connection so that the new route remains consistent with the cell layout. If the original path is still usable, the robot continues along it. This keeps obstacle recognition separate from route selection and motion execution.

**Audit:** Main flow correct, but add that candidates making the goal unreachable are rejected, and old blocks are not cleared on absence alone. [代码/计算检查] (experiment_obstacles.py; obstacle.py; obstacle_node.py; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R39 — LABEL

> 3 Development and iteration

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R40 — KEEP

> The project went through many revisions and physical tests. We encountered problems in localisation, planning, control and obstacle recognition, often working on more than one at a time. The three versions below group the main design changes. We then discuss a few of the most useful challenges, rather than listing every parameter adjustment or code revision.

**Audit:** Supported as a summary of extensive interleaved revisions; do not imply every edit had a physical test. [代码/计算检查] (H; attached test records; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R41 — LABEL

> V1 Coarse planning with straight motion and in-place turns

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R42 — KEEP

> Problem and change. We first needed a clear route representation and a working connection between planning and the base. The initial controller followed coarse-grid centre lines, stopped at a corner and turned in place before starting the next straight section.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (H V1 entries; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R43 — QUALIFY

> Observed behaviour. This made individual movements easy to inspect. We could check the planned cell sequence, the stopping position and the requested turn separately, and establish basic navigation between goals.

**Audit:** Inspection benefits are design rationale; basic navigation has physical examples, not a measured comparison of inspectability. [代码/计算检查] (H; Nov 5 XUNX evidence; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R44 — REWRITE CAUSAL CLAIM

> Remaining problem. The real base did not rotate around a perfectly fixed centre. Skid steering displaced the body during turns, leaving the next straight section offset from its reference. Repeated stopping also made the motion less continuous. These effects motivated a change in the controller, rather than further reliance on ideal in-place rotation.

**Audit:** Observed body drift and discontinuous motion are supported. Replace “Skid steering displaced the body” with “Observed body drift was consistent with skid-steer slip; other feedback effects were not isolated.” [代码/计算检查] (user turn observations; H; cause [推测); real-robot statements only where explicitly cited above]
**Speak:** “This wording must retain its evidence limits.”

#### R45 — LABEL

> V2 Controller trials and the final hybrid method

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R46 — KEEP WITH SCOPE

> Problem and change. We explored PID, Stanley, Pure Pursuit/RPP and DWA while improving straight tracking, turning and goal approach. The final design kept PID heading regulation with Stanley-style lateral correction on straight sections, and used RPP at corners and in the goal cell.

**Audit:** Explored all listed methods is supported; do not claim equal-condition real-robot evaluation of each. [代码/计算检查] (CONTROLLER_TRIALS.txt; H; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R47 — QUALIFY

> Observed behaviour. The combination gave smoother transitions and better passage through successive corners in physical tests. Adding lateral correction addressed an error that heading control alone could not remove: a robot can face along a corridor while remaining displaced from its centreline.

**Audit:** Smoother/better physical passage is qualitative user feedback, not a measured improvement. The heading-versus-lateral distinction is mathematically supported. [代码/计算检查] (E-base/user feedback; node.py; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R48 — REWRITE CAUSAL CLAIM

> Remaining problem. Increasing speed or correction strength did not always help. Aggressive settings amplified pose fluctuations and the effects of slip. We therefore selected parameters from overall route completion and tracking quality, rather than treating a larger gain or shorter lookahead as automatically better.

**Audit:** Speed/gain rollbacks are documented. “Amplified pose fluctuations and slip” was not isolated; replace with “were associated with less consistent observed tracking.” [代码/计算检查] (H Nov 10/11; cause [推测); real-robot statements only where explicitly cited above]
**Speak:** “This wording must retain its evidence limits.”

#### R50 — LABEL

> V3 Obstacle recognition and route updates

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R51 — QUALIFY

> Problem and change. Early detection could mistake existing walls for new obstacles or miss a large block when only part of its surface was visible. We replaced broad geometric assumptions with the task-specific checks described in Section 2.4: exclude mapped walls, inspect open adjacent cells, and confirm a stable surface near the relevant boundary.

**Audit:** False/missed detections are supported, but these changes did not eliminate all misses; November 12 L is a counterexample to any universal implication. [代码/计算检查] (H; E-L; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R52 — QUALIFY

> Observed behaviour. The final system detected newly blocked cells, replanned when the active route was affected, and continued towards the goals. Separating obstacle updates from replanning reduced unnecessary route changes caused by observations away from the remaining path.

**Audit:** At least one real detour and mission completion is logged. “Reduced unnecessary route changes” follows code gating and observed early issues; no reduction rate was measured. [代码/计算检查] (E-base; blocks_route; H; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R53 — KEEP

> Remaining limitation. These rules are suited to large, regular blocks in this arena. They are not a general detector for small objects, irregular obstacles or rapidly moving objects.

**Audit:** All sentences/expressions supported at code or clearly stated explanatory level; no independent physical precision implied. [代码/计算检查] (obstacle.py limitations; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R54 — LABEL

> 3.1 Selected challenges and improvements

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R55 — QUALIFY

> Getting the robot reference point right. Early runs sometimes stopped before the body reached the cell centre. The laser mounting transform affected which physical point the localisation estimate represented. We checked the sensor offset and coordinate relationship, then corrected the transform. The stopping position improved substantially. This showed why a control error should not always be addressed by changing the arrival condition: the reference geometry itself may be wrong.

**Audit:** Transform changes and qualitative centre improvement are supported. “Improved substantially” must remain qualitative; no independent before/after physical metric. [代码/计算检查] (H Nov 5/6; E dea73374 and 3a18d084; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R56 — REWRITE CAUSAL CLAIM

> Accounting for the base during turns. We initially treated a turn as a change of heading with no translation. On the four-wheel skid-steer base, tyre slip moved the effective turning centre, and a correct final angle could still leave the body displaced. RPP reduced dependence on that idealised motion by following a curved reference. The physical behaviour of the base was therefore part of the controller choice, not just a tuning detail.

**Audit:** Replace categorical slip attribution with observed body displacement consistent with slip. RPP reduces dependence on fixed-centre pivots by design; it still permits pivots. [代码/计算检查] (corner geometry; user turning observations; physical cause [推测); real-robot statements only where explicitly cited above]
**Speak:** “This wording must retain its evidence limits.”

#### R57 — QUALIFY

> Distinguishing new obstacles from old walls. A straight group of laser returns is not sufficient evidence of a new obstruction. Fixed walls produce the same shape. Map-based exclusion removed those returns before the adjacent-cell checks. Allowing a partial visible face also addressed missed detections when the scanner could not see the full obstacle width. Spatial checks and confirmation across scans worked together to reduce false decisions.

**Audit:** Filter order and partial-face design are correct. Say “designed to reduce misses and false decisions”; do not imply all partial-view misses were solved. [代码/计算检查] (obstacle.py; Nov 9 simulation; E-L; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R58 — QUALIFY

> Avoiding counterproductive goal corrections. We tried repeated small movements to reduce the final position error. Low-speed response, localisation fluctuations and slip sometimes turned a correction into another offset. The final version makes one approach and holds the stop, without repeated alignment. This gave more consistent behaviour and avoided spending time on movements that did not improve the physical result.

**Audit:** Removal and operator preference are supported. Low-speed/slip/noise explanation is plausible, not isolated; improved consistency/time saving were not quantified. [代码/计算检查] (H Nov 10/11; user feedback; cause [推测); real-robot statements only where explicitly cited above]
**Speak:** “This wording must retain its evidence limits.”

#### R59 — QUALIFY

> Waiting for a usable initial estimate. Starting while the AMCL particles were still spread out introduced an initial pose error into subsequent tracking. We added a stationary settling period and consecutive covariance checks before departure. Physical testing showed a better startup, so we retained this change when restoring the successful motion speeds.

**Audit:** The gate and positive startup feedback are supported. “Introduced an initial pose error” is a hypothesis without independent pose truth. [代码/计算检查] (corner_rpp.py; E-start and E-L; initial-error cause [推测); real-robot statements only where explicitly cited above]
**Speak:** “This wording must retain its evidence limits.”

#### R60 — KEEP

> Across these changes, physical tests were essential. A plausible formula or clean software structure did not by itself establish that the robot would follow the intended line. We used the observed body position, turning behaviour, scan alignment and runtime records together to decide which changes to keep.

**Audit:** Methodological lesson is consistent with the record; does not provide a quantified causal result. [代码/计算检查] (H and physical feedback; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R62 — LABEL

> 4 Final results and limitations

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R63 — LABEL

> 4.1 Route completion and motion

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R64 — QUALIFY

> The final physical test completed the requested goal sequence. The robot travelled through straight passages and successive corners, then stopped close to the target cell centres. Motion was generally smooth, with more natural transitions than the earlier stop-and-turn controller. The final run did not show the persistent spinning that had affected earlier versions.

**Audit:** User confirmed final success and smooth near-centre behaviour. No separate complete post-freeze log proves every clause for an identified run; specify qualitative operator assessment. [代码/计算检查] (user post-freeze confirmation; E-base is earlier partial extract; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R65 — QUALIFY

> Heading and lateral corrections kept straight travel broadly aligned with the intended route. RPP allowed the robot to negotiate the bends and approach the goals without repeated alignment manoeuvres. Small deviations remained on some straight sections, and some corners were taken closer to the inner wall. 

**Audit:** Supported as qualitative observation, not an RMS tracking or clearance measurement. [代码/计算检查] (user/video feedback; logged lateral residuals; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R66 — LABEL

> 4.2 Dynamic obstacle replanning

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R67 — KEEP WITH SOURCE

> Physical testing demonstrated dynamic replanning around a newly detected obstacle. After confirming the obstacle face, the system marked the corresponding cell as blocked. When that cell lay on the remaining route, A* generated an alternative path and the robot followed it to continue the mission.

**Audit:** Actual B confirmation and detour continuation in E-base support dynamic replanning. Do not use only video wall-following as proof; cite the log. [代码/计算检查] (E-base; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R68 — QUALIFY

> The test exercised the complete sequence from laser observation to route execution. Map-wall exclusion helped prevent false blocking, and route checks avoided unnecessary replanning.

**Audit:** Integration events are logged. Filtering benefit has simulated and code evidence; it is not a measured general real-world false-positive rate. [代码/计算检查] (E-base; G post-filter results; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R69 — LABEL

> 4.3 Time and stopping performance

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R70 — REWRITE

> The submitted test video is 132.77 s long, or approximately 2 min 13 s, providing a recorded demonstration shorter than the three-minute task limit. The timing is taken from the recording. We retained the successful baseline speeds after faster trials gave less consistent tracking.

**Audit:** Recording duration 132.77 s is verified. Replace “demonstration shorter than ... task limit” with “The supplied recording lasts 132.77 s; complete mission timing coverage is not independently established.” Speed rollback is documented, comparative precision unmeasured. [代码/计算检查] (V metadata; H Nov 11; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R71 — QUALIFY

> Visual inspection of the physical run showed stopping positions generally close to the cell centres. Correcting the sensor reference position and simplifying the final approach both contributed to this result. This is a visual assessment; logged pose residuals are not independent ground-truth measurements.

**Audit:** Visual/estimated-versus-truth distinction is correct. Sensor and approach changes were associated with improvement; independent causal contribution of each is unmeasured. [代码/计算检查] (user feedback; logs; no physical truth; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

#### R72 — LABEL

> 4.4 Limitations

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R73 — QUALIFY

> Localisation error and wheel slip still affect corner clearance and stopping position. The obstacle detector also relies on the known arena layout and the large cell-sized blocks specified for the task. A sudden disturbance that causes a large localisation error can be difficult to recover from quickly, interrupting otherwise stable navigation.

**Audit:** Scope and difficult recovery are defensible limitations. Do not imply final AMCL auto-recovery settings or new-obstacle-induced bias were measured. [代码/计算检查] (final code; user limitation; dynamics effects [推测); real-robot statements only where explicitly cited above]
**Speak:** “This wording must retain its evidence limits.”

#### R74 — LABEL

> 5 Team contributions and collaboration

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R75 — KEEP AS DECLARED ROLES

> All three members contributed ideas for solving the problems encountered during development. DENG YUNTIAN focused on turning these ideas into working code and improving the algorithms. ZENG JINWEN focused on debugging and fixing problems. PIA TALESARA focused on identifying problems during operation and suggesting improvements. We used discussion and feedback from physical tests to decide which changes to retain.

**Audit:** Matches the user-provided contribution statement. It is not a verified per-commit authorship audit. [代码/计算检查] (conversation team-role declaration; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R76 — LABEL

> 6 Conclusion

**Audit:** Heading, label or declared member names; no performance assertion. [代码/计算检查] (R structure / user team list; real-robot statements only where explicitly cited above)
**Speak:** “This describes the selected baseline at the stated evidence level.”

#### R77 — QUALIFY

> We implemented autonomous navigation for a structured grid arena using AMCL localisation, coarse-grid A* planning, PID with Stanley-style lateral correction, and RPP at corners and in goal cells. The system also detects large cell obstacles and replans around them. After repeated physical tests and revisions, the final version completed navigation smoothly and stopped close to the intended cell centres. The main lesson was that algorithm choice must be supported by correct sensor geometry, consistent coordinate frames, an understanding of the robot’s motion characteristics, and coordination between localisation and control.

**Audit:** Architecture and existence of real obstacle replanning supported. Smooth/near-centre outcome is qualitative and version-specific; retain no numerical accuracy implication. [代码/计算检查] (final source; E-base; user success; real-robot statements only where explicitly cited above)
**Speak:** “This wording must retain its evidence limits.”

### 10.3 Slides — every visible text item

Each table preserves all visible text strings, including labels and numbers. Equations split over lines are evaluated as parts of the same equation. KEEP means supported for the baseline, not a new physical experiment.

#### Slide 1

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| RB3302 | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Autonomous Navigation | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Project | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| DENG YUNTIAN | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| PIA TALESARA | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| ZENG JINWEN | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 2

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| 01 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Task and system overview | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| 2 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 3

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| The navigation task | KEEP: baseline description. | [代码/计算检查] (course.yaml; Project_Description.pdf; 实机验证 only as specified) |
| 3 | LABEL: page number. | [代码/计算检查] (course.yaml; Project_Description.pdf; 实机验证 only as specified) |
| 24 lettered cells in a 6 × 4 arena | KEEP: baseline description. | [代码/计算检查] (course.yaml; Project_Description.pdf; 实机验证 only as specified) |
| 58 cm between adjacent cell centres | QUALIFY: operator-reported working spacing; assignment says approximately 60 cm. | [代码/计算检查] (course.yaml; Project_Description.pdf; 实机验证 only as specified) |
| Visit goals in the requested order | KEEP: baseline description. | [代码/计算检查] (course.yaml; Project_Description.pdf; 实机验证 only as specified) |
| Replan around new cell-sized obstacles | KEEP: baseline description. | [代码/计算检查] (course.yaml; Project_Description.pdf; 实机验证 only as specified) |
| Stop at goals and meet the 3-minute limit | KEEP: baseline description. | [代码/计算检查] (course.yaml; Project_Description.pdf; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 4

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Robot and sensors | KEEP: baseline description. | [代码/计算检查] (hardware description; localization.yaml; 实机验证 only as specified) |
| 4 | LABEL: page number. | [代码/计算检查] (hardware description; localization.yaml; 实机验证 only as specified) |
| Four-wheel skid-steer base | KEEP: baseline description. | [代码/计算检查] (hardware description; localization.yaml; 实机验证 only as specified) |
| Laser scanner: environment geometry | KEEP: baseline description. | [代码/计算检查] (hardware description; localization.yaml; 实机验证 only as specified) |
| Wheel odometry: forward motion | KEEP: baseline description. | [代码/计算检查] (hardware description; localization.yaml; 实机验证 only as specified) |
| IMU: heading and angular velocity | KEEP: baseline description. | [代码/计算检查] (hardware description; localization.yaml; 实机验证 only as specified) |
| ROS 2: localisation, planning and control | KEEP: baseline description. | [代码/计算检查] (hardware description; localization.yaml; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 5

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Mission flow | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| 5 | LABEL: page number. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| 1  Wait for localisation to settle | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| 2  Plan a route with A* | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| 3  Track the route and monitor obstacles | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| 4  Replan when the remaining route is blocked | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| 5  Hold 4 seconds at each goal, then continue | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 6

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| 02 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Final methods | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| 6 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 7

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Local motion and map localisation | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| 7 | LABEL: page number. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| EKF | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| Wheel velocity + IMU | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| Continuous local odometry | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| Short-term motion estimate | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| AMCL | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| Laser scan + known map | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| Correct map position and heading | QUALIFY: estimates/corrects pose; not guaranteed correct ground truth. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |
| Control uses the combined TF pose | KEEP: baseline description. | [代码/计算检查] (localization.yaml; robot.py; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 8

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| How AMCL updates the pose | KEEP: baseline description. | [代码/计算检查] (AMCL configured method; 实机验证 only as specified) |
| 8 | LABEL: page number. | [代码/计算检查] (AMCL configured method; 实机验证 only as specified) |
| Particles represent possible robot poses | KEEP: baseline description. | [代码/计算检查] (AMCL configured method; 实机验证 only as specified) |
| Odometry predicts their movement | KEEP: baseline description. | [代码/计算检查] (AMCL configured method; 实机验证 only as specified) |
| Laser-to-map agreement sets particle weights | KEEP: baseline description. | [代码/计算检查] (AMCL configured method; 实机验证 only as specified) |
| Resampling concentrates the better matches | KEEP: baseline description. | [代码/计算检查] (AMCL configured method; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 9

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Startup convergence | KEEP: covariance gate, not actual-accuracy guarantee; largest XY standard deviation. | [代码/计算检查] (corner_rpp.py initialize; 实机验证 only as specified) |
| 9 | LABEL: page number. | [代码/计算检查] (corner_rpp.py initialize; 实机验证 only as specified) |
| Keep the robot stationary for at least 3 s | KEEP: covariance gate, not actual-accuracy guarantee; largest XY standard deviation. | [代码/计算检查] (corner_rpp.py initialize; 实机验证 only as specified) |
| Require 5 consecutive fresh AMCL estimates | KEEP: covariance gate, not actual-accuracy guarantee; largest XY standard deviation. | [代码/计算检查] (corner_rpp.py initialize; 实机验证 only as specified) |
| Position standard deviation ≤ 5 cm | KEEP: covariance gate, not actual-accuracy guarantee; largest XY standard deviation. | [代码/计算检查] (corner_rpp.py initialize; 实机验证 only as specified) |
| Heading standard deviation ≤ 5° | KEEP: covariance gate, not actual-accuracy guarantee; largest XY standard deviation. | [代码/计算检查] (corner_rpp.py initialize; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 10

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Two map representations | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| 10 | LABEL: page number. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Fine occupancy map | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Laser localisation | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Known-wall filtering | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Geometric clearance checks | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Coarse cell graph | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Fixed centres at 58 cm spacing | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Four cardinal neighbours | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |
| Cell occupancy for route planning | KEEP: baseline description. | [代码/计算检查] (grid.py; map files; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 11

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Cell centres and A* | KEEP: baseline description. | [代码/计算检查] (grid.py; course.yaml; 实机验证 only as specified) |
| 11 | LABEL: page number. | [代码/计算检查] (grid.py; course.yaml; 实机验证 only as specified) |
| xᵢ = x₀ + i d       yⱼ = y₀ + j d       d = 0.58 m | KEEP: define origin as bottom-left centre; alphabetical rows run opposite increasing y. | [代码/计算检查] (grid.py; course.yaml; 实机验证 only as specified) |
| f(n) = g(n) + h(n) | KEEP: baseline description. | [代码/计算检查] (grid.py; course.yaml; 实机验证 only as specified) |
| h(n) = \|i − i_goal\| + \|j − j_goal\| | KEEP: baseline description. | [代码/计算检查] (grid.py; course.yaml; 实机验证 only as specified) |
| g: accumulated cost       h: remaining grid distance | KEEP: baseline description. | [代码/计算检查] (grid.py; course.yaml; 实机验证 only as specified) |
| Extract straight sections, corners and the final approach | KEEP: baseline description. | [代码/计算检查] (grid.py; course.yaml; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 12

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| The final hybrid controller | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| 12 | LABEL: page number. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| Ordinary straight sections | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| Heading PID | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| Stanley-style lateral correction | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| Fixed cell-centre reference line | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| Corners and the goal cell | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| RPP path tracking | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| Curvature-based speed regulation | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |
| Continuous pose updates | KEEP: baseline description. | [代码/计算检查] (node.py; corner_rpp.py; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 13

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Straight tracking errors | KEEP: baseline description. | [代码/计算检查] (node.py error formulas; 实机验证 only as specified) |
| 13 | LABEL: page number. | [代码/计算检查] (node.py error formulas; 实机验证 only as specified) |
| eθ = wrap(θ_reference − θ_robot) | KEEP: baseline description. | [代码/计算检查] (node.py error formulas; 实机验证 only as specified) |
| e_y = −sin(θ_reference)(x − x_start) | KEEP: baseline description. | [代码/计算检查] (node.py error formulas; 实机验证 only as specified) |
|           + cos(θ_reference)(y − y_start) | KEEP: baseline description. | [代码/计算检查] (node.py error formulas; 实机验证 only as specified) |
| e_f = e_y − l_f sin(eθ)       l_f = 0.10 m | KEEP: baseline description. | [代码/计算检查] (node.py error formulas; 实机验证 only as specified) |
| Heading error eθ      Lateral error e_y      Front error e_f | KEEP: baseline description. | [代码/计算检查] (node.py error formulas; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 14

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| PID plus Stanley-style correction | KEEP: baseline description. | [代码/计算检查] (node.py and turn_feedback.py; 实机验证 only as specified) |
| 14 | LABEL: page number. | [代码/计算检查] (node.py and turn_feedback.py; 实机验证 only as specified) |
| ω = Kₚeθ + Kᵢ∫eθ dt − Kdθ̇ | QUALIFY: requested yaw law before measured-rate trim and output saturation. | [代码/计算检查] (node.py and turn_feedback.py; 实机验证 only as specified) |
|        − Kₛ arctan(k e_f / (v + vₛ)) | QUALIFY: requested yaw law before measured-rate trim and output saturation. | [代码/计算检查] (node.py and turn_feedback.py; 实机验证 only as specified) |
| PID: face along the route | KEEP: baseline description. | [代码/计算检查] (node.py and turn_feedback.py; 实机验证 only as specified) |
| Lateral term: return to the centreline | KEEP: baseline description. | [代码/计算检查] (node.py and turn_feedback.py; 实机验证 only as specified) |
| vₛ softens low-speed correction | KEEP: baseline description. | [代码/计算检查] (node.py and turn_feedback.py; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 15

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Adapting Stanley to this robot | KEEP: baseline description. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |
| 15 | LABEL: page number. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |
| Standard Stanley | KEEP: baseline description. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |
| Front-wheel steering angle | KEEP: baseline description. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |
| Heading + lateral correction | KEEP: baseline description. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |
| Our implementation | KEEP: baseline description. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |
| Body angular velocity | KEEP: baseline description. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |
| Heading PID + lateral term | KEEP: baseline description. | [代码/计算检查] (node.py adapted lateral law; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 16

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| RPP at corners and goals | KEEP: baseline description. | [代码/计算检查] (corner_geometry.py and corner_rpp.py; 实机验证 only as specified) |
| 16 | LABEL: page number. | [代码/计算检查] (corner_geometry.py and corner_rpp.py; 实机验证 only as specified) |
| Rounded reference path + forward tracking point | KEEP: baseline description. | [代码/计算检查] (corner_geometry.py and corner_rpp.py; 实机验证 only as specified) |
| κ = 2yL / (xL² + yL²) | KEEP: baseline description. | [代码/计算检查] (corner_geometry.py and corner_rpp.py; 实机验证 only as specified) |
| ω = vκ | KEEP: baseline description. | [代码/计算检查] (corner_geometry.py and corner_rpp.py; 实机验证 only as specified) |
| (xL, yL): lookahead point in robot coordinates | KEEP: baseline description. | [代码/计算检查] (corner_geometry.py and corner_rpp.py; 实机验证 only as specified) |
| Final lookahead: 20 cm | KEEP: baseline description. | [代码/计算检查] (corner_geometry.py and corner_rpp.py; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 17

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Tracking and arrival | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| 17 | LABEL: page number. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| Long lookahead can cut corners | QUALIFY: general control trade-off, not isolated measured project effect. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| Short lookahead reacts more strongly to pose noise | QUALIFY: general control trade-off, not isolated measured project effect. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| Use RPP inside the goal cell | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| Stop when longitudinal remaining distance reaches zero | QUALIFY: last-edge endpoint plane; no zero-lateral-error guarantee. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |
| Hold 4 s, keep localisation updating, no repeated alignment | KEEP: baseline description. | [代码/计算检查] (corner_rpp.py; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 18

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Recognising a new obstacle | KEEP: baseline description. | [代码/计算检查] (experiment_obstacles.py; 实机验证 only as specified) |
| 18 | LABEL: page number. | [代码/计算检查] (experiment_obstacles.py; 实机验证 only as specified) |
| Exclude returns from existing map walls | KEEP: baseline description. | [代码/计算检查] (experiment_obstacles.py; 实机验证 only as specified) |
| Inspect only the four open adjacent cells | KEEP: baseline description. | [代码/计算检查] (experiment_obstacles.py; 实机验证 only as specified) |
| Look near the boundary facing the robot | KEEP: baseline description. | [代码/计算检查] (experiment_obstacles.py; 实机验证 only as specified) |
| Detect a broad face, not a complete box outline | KEEP: baseline description. | [代码/计算检查] (experiment_obstacles.py; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 19

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Obstacle confirmation | KEEP FOR FROZEN BASELINE; latest experimental width/time/count differ. | [代码/计算检查] (S obstacle.py/control.yaml; 实机验证 only as specified) |
| 19 | LABEL: page number. | [代码/计算检查] (S obstacle.py/control.yaml; 实机验证 only as specified) |
| Visible face span ≥ 30 cm | KEEP FOR FROZEN BASELINE; latest experimental width/time/count differ. | [代码/计算检查] (S obstacle.py/control.yaml; 实机验证 only as specified) |
| Direction error ≤ 10° | KEEP FOR FROZEN BASELINE; latest experimental width/time/count differ. | [代码/计算检查] (S obstacle.py/control.yaml; 实机验证 only as specified) |
| Distance from expected boundary ≤ 20 cm | KEEP FOR FROZEN BASELINE; latest experimental width/time/count differ. | [代码/计算检查] (S obstacle.py/control.yaml; 实机验证 only as specified) |
| At least 4 fresh scans over ≥ 0.5 s | KEEP FOR FROZEN BASELINE; latest experimental width/time/count differ. | [代码/计算检查] (S obstacle.py/control.yaml; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 20

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Updating the route | KEEP: baseline description. | [代码/计算检查] (obstacle state/replan code; 实机验证 only as specified) |
| 20 | LABEL: page number. | [代码/计算检查] (obstacle state/replan code; 实机验证 only as specified) |
| Mark the confirmed obstacle cell as blocked | QUALIFY: candidates making the active goal unreachable are rejected before confirmation. | [代码/计算检查] (obstacle state/replan code; 实机验证 only as specified) |
| Keep only the latest confirmed obstacle | KEEP: replacement only; absence does not automatically clear previous block. | [代码/计算检查] (obstacle state/replan code; 实机验证 only as specified) |
| Check whether it blocks the remaining route | KEEP: baseline description. | [代码/计算检查] (obstacle state/replan code; 实机验证 only as specified) |
| If blocked, run A* and connect at a safe grid centre | KEEP: baseline description. | [代码/计算检查] (obstacle state/replan code; 实机验证 only as specified) |
| Otherwise, continue the existing route | KEEP: baseline description. | [代码/计算检查] (obstacle state/replan code; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 21

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| 03 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Development and iteration | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| 21 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 22

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Many revisions, three main stages | KEEP: baseline description. | [代码/计算检查] (H; 实机验证 only as specified) |
| 22 | LABEL: page number. | [代码/计算检查] (H; 实机验证 only as specified) |
| V1: coarse planning + straight / in-place turns | KEEP: baseline description. | [代码/计算检查] (H; 实机验证 only as specified) |
| V2: controller trials + hybrid motion | KEEP: baseline description. | [代码/计算检查] (H; 实机验证 only as specified) |
| V3: obstacle detection + dynamic replanning | KEEP: baseline description. | [代码/计算检查] (H; 实机验证 only as specified) |
| Selected changes from many physical tests | KEEP: baseline description. | [代码/计算检查] (H; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 23

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| V1  Basic grid navigation | KEEP: baseline description. | [代码/计算检查] (H V1 and user physical observations; 实机验证 only as specified) |
| 23 | LABEL: page number. | [代码/计算检查] (H V1 and user physical observations; 实机验证 only as specified) |
| Change: follow centres, stop, turn in place | KEEP: baseline description. | [代码/计算检查] (H V1 and user physical observations; 实机验证 only as specified) |
| Result: clear planning and movement sequence | QUALIFY: operator/design observation, no measured before-after accuracy. | [代码/计算检查] (H V1 and user physical observations; 实机验证 only as specified) |
| Problem: turning moved the body centre | QUALIFY: operator/design observation, no measured before-after accuracy. | [代码/计算检查] (H V1 and user physical observations; 实机验证 only as specified) |
| Next step: improve tracking and motion continuity | KEEP: baseline description. | [代码/计算检查] (H V1 and user physical observations; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 24

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| V2  Motion control trials | KEEP: baseline description. | [代码/计算检查] (H/controller trials; 实机验证 only as specified) |
| 24 | LABEL: page number. | [代码/计算检查] (H/controller trials; 实机验证 only as specified) |
| Explored PID, Stanley, PP/RPP and DWA | KEEP: implementation/exploration, not controlled physical benchmark of every controller. | [代码/计算检查] (H/controller trials; 实机验证 only as specified) |
| Final: PID + lateral correction on straights | KEEP: baseline description. | [代码/计算检查] (H/controller trials; 实机验证 only as specified) |
| RPP at corners and goals | KEEP: baseline description. | [代码/计算检查] (H/controller trials; 实机验证 only as specified) |
| Result: smoother transitions and successive turns | QUALIFY: qualitative user preference, no smoothness metric. | [代码/计算检查] (H/controller trials; 实机验证 only as specified) |
| Trade-off: stronger correction was not always better | KEEP: baseline description. | [代码/计算检查] (H/controller trials; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 25

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| V3  Obstacle handling | KEEP: baseline description. | [代码/计算检查] (H, E-base and Gazebo post-filter records; 实机验证 only as specified) |
| 25 | LABEL: page number. | [代码/计算检查] (H, E-base and Gazebo post-filter records; 实机验证 only as specified) |
| Problem: old walls triggered false blocks | KEEP: baseline description. | [代码/计算检查] (H, E-base and Gazebo post-filter records; 实机验证 only as specified) |
| Problem: partial views caused missed detections | KEEP: baseline description. | [代码/计算检查] (H, E-base and Gazebo post-filter records; 实机验证 only as specified) |
| Change: mapped-wall filtering + stable near-face checks | KEEP: baseline description. | [代码/计算检查] (H, E-base and Gazebo post-filter records; 实机验证 only as specified) |
| Result: successful rerouting around blocked cells | KEEP WITH SOURCE: real B-detour extract; not universal detection reliability. | [代码/计算检查] (H, E-base and Gazebo post-filter records; 实机验证 only as specified) |
| Scope: large regular obstacles in this arena | KEEP: baseline description. | [代码/计算检查] (H, E-base and Gazebo post-filter records; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 26

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Geometry and turning challenges | KEEP: baseline description. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |
| 26 | LABEL: page number. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |
| Early stopping | QUALIFY: observed body offset, not proof every early stop shared one cause. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |
| Sensor reference mismatch | KEEP: baseline description. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |
| Correct the mounting transform | KEEP: baseline description. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |
| Body drift during turns | KEEP: baseline description. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |
| Skid-steer tyre slip | REWORD: “Body drift consistent with skid-steer slip”; causal contribution not independently measured. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |
| Use RPP path tracking | KEEP: baseline description. | [代码/计算检查] (H sensor correction, user turn observations; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 27

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Detection and alignment challenges | KEEP: baseline description. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |
| 27 | LABEL: page number. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |
| False obstacles | KEEP: baseline description. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |
| Original walls resemble obstacle faces | KEEP: baseline description. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |
| Exclude mapped walls first | KEEP: baseline description. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |
| Repeated goal corrections | KEEP: baseline description. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |
| Small moves sometimes made errors worse | KEEP AS OPERATOR FEEDBACK; no measured error reduction. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |
| One approach + stationary hold | KEEP: baseline description. | [代码/计算检查] (H; obstacle filters; user alignment feedback; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 28

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Startup and parameter choices | KEEP: baseline description. | [代码/计算检查] (H Nov 11; user startup feedback; 实机验证 only as specified) |
| 28 | LABEL: page number. | [代码/计算检查] (H Nov 11; user startup feedback; 实机验证 only as specified) |
| Wait for initial particle convergence | KEEP: baseline description. | [代码/计算检查] (H Nov 11; user startup feedback; 实机验证 only as specified) |
| Retain the successful baseline speeds | KEEP: baseline description. | [代码/计算检查] (H Nov 11; user startup feedback; 实机验证 only as specified) |
| Balance lookahead, correction strength and slip | KEEP: baseline description. | [代码/计算检查] (H Nov 11; user startup feedback; 实机验证 only as specified) |
| Judge changes using physical runs and logs | KEEP: baseline description. | [代码/计算检查] (H Nov 11; user startup feedback; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 29

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| 04 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Results and limitations | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| 29 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 30

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Final physical performance | QUALIFY: user-confirmed successful run; no complete independent post-freeze measurement set. | [代码/计算检查] (user successful-run feedback; E-base; 实机验证 only as specified) |
| 30 | LABEL: page number. | [代码/计算检查] (user successful-run feedback; E-base; 实机验证 only as specified) |
| Requested goal sequence completed | QUALIFY: user-confirmed successful run; no complete independent post-freeze measurement set. | [代码/计算检查] (user successful-run feedback; E-base; 实机验证 only as specified) |
| Smooth straight and curved motion | QUALIFY: user-confirmed successful run; no complete independent post-freeze measurement set. | [代码/计算检查] (user successful-run feedback; E-base; 实机验证 only as specified) |
| Stops generally close to cell centres | QUALIFY: user-confirmed successful run; no complete independent post-freeze measurement set. | [代码/计算检查] (user successful-run feedback; E-base; 实机验证 only as specified) |
| No persistent spinning in the final run | QUALIFY: user-confirmed successful run; no complete independent post-freeze measurement set. | [代码/计算检查] (user successful-run feedback; E-base; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 31

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Dynamic replanning and time | KEEP WITH LOG: B confirmation/replan/continuation; exact video-to-route mapping unknown. | [代码/计算检查] (E-base obstacle events; V metadata; 实机验证 only as specified) |
| 31 | LABEL: page number. | [代码/计算检查] (E-base obstacle events; V metadata; 实机验证 only as specified) |
| Detected a new cell-sized obstacle | KEEP WITH LOG: B confirmation/replan/continuation; exact video-to-route mapping unknown. | [代码/计算检查] (E-base obstacle events; V metadata; 实机验证 only as specified) |
| Updated occupancy and generated a detour | KEEP WITH LOG: B confirmation/replan/continuation; exact video-to-route mapping unknown. | [代码/计算检查] (E-base obstacle events; V metadata; 实机验证 only as specified) |
| Continued the goal sequence | KEEP WITH LOG: B confirmation/replan/continuation; exact video-to-route mapping unknown. | [代码/计算检查] (E-base obstacle events; V metadata; 实机验证 only as specified) |
| Test video: 132.77 s, approximately 2 min 13 s | REWORD: “Recording duration: 132.77 s; complete mission timing not independently established.” | [代码/计算检查] (E-base obstacle events; V metadata; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 32

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Remaining limitations | KEEP AS LIMITATION; do not assert automatic recovery enabled/tested. | [代码/计算检查] (limitations in Section 6; 实机验证 only as specified) |
| 32 | LABEL: page number. | [代码/计算检查] (limitations in Section 6; 实机验证 only as specified) |
| Small stopping and corner offsets | KEEP AS LIMITATION; do not assert automatic recovery enabled/tested. | [代码/计算检查] (limitations in Section 6; 实机验证 only as specified) |
| Wheel slip affects actual turning motion | KEEP AS LIMITATION; do not assert automatic recovery enabled/tested. | [代码/计算检查] (limitations in Section 6; 实机验证 only as specified) |
| Detection assumes large regular cell obstacles | KEEP AS LIMITATION; do not assert automatic recovery enabled/tested. | [代码/计算检查] (limitations in Section 6; 实机验证 only as specified) |
| Large localisation disturbances can be hard to recover from | KEEP AS LIMITATION; do not assert automatic recovery enabled/tested. | [代码/计算检查] (limitations in Section 6; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 33

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| 05 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Team contributions | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| 33 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 34

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Shared ideas and implementation | KEEP AS USER-DECLARED ROLE; no exclusive feature authorship established. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |
| 34 | LABEL: page number. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |
| DENG YUNTIAN | KEEP AS USER-DECLARED ROLE; no exclusive feature authorship established. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |
| ZENG JINWEN | KEEP AS USER-DECLARED ROLE; no exclusive feature authorship established. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |
| PIA TALESARA | KEEP AS USER-DECLARED ROLE; no exclusive feature authorship established. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |
| Implementation and optimisation | KEEP AS USER-DECLARED ROLE; no exclusive feature authorship established. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |
| Debugging and fixes | KEEP AS USER-DECLARED ROLE; no exclusive feature authorship established. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |
| Finding problems and suggesting changes | KEEP AS USER-DECLARED ROLE; no exclusive feature authorship established. | [代码/计算检查] (user role declaration; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 35

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| 06 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| Conclusion | LABEL: no measured-performance claim. | [代码/计算检查] (deck structure; 实机验证 only as specified) |
| 35 | LABEL: page number. | [代码/计算检查] (deck structure; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 36

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| What we achieved | KEEP: baseline description. | [代码/计算检查] (final code; qualitative success feedback; 实机验证 only as specified) |
| 36 | LABEL: page number. | [代码/计算检查] (final code; qualitative success feedback; 实机验证 only as specified) |
| AMCL localisation and coarse-grid A* | KEEP: baseline description. | [代码/计算检查] (final code; qualitative success feedback; 实机验证 only as specified) |
| PID + lateral correction, with RPP at corners and goals | KEEP: baseline description. | [代码/计算检查] (final code; qualitative success feedback; 实机验证 only as specified) |
| Large-obstacle detection and dynamic replanning | KEEP: baseline description. | [代码/计算检查] (final code; qualitative success feedback; 实机验证 only as specified) |
| Smooth navigation with stops near cell centres | QUALIFY: qualitative performance in successful trials, not quantified ground-truth accuracy. | [代码/计算检查] (final code; qualitative success feedback; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

#### Slide 37

| Exact visible text | Verdict / correction | Evidence |
|---|---|---|
| Main lesson | KEEP: baseline description. | [代码/计算检查] (project engineering lesson, H; 实机验证 only as specified) |
| 37 | LABEL: page number. | [代码/计算检查] (project engineering lesson, H; 实机验证 only as specified) |
| Correct sensor geometry | KEEP: baseline description. | [代码/计算检查] (project engineering lesson, H; 实机验证 only as specified) |
| Consistent coordinate frames | KEEP: baseline description. | [代码/计算检查] (project engineering lesson, H; 实机验证 only as specified) |
| Control matched to the robot’s motion | KEEP: baseline description. | [代码/计算检查] (project engineering lesson, H; 实机验证 only as specified) |
| Localisation and control working together | KEEP: baseline description. | [代码/计算检查] (project engineering lesson, H; 实机验证 only as specified) |

**Speak:** “Use this slide with the evidence qualifications above.”

### 10.4 Speaker notes — complete audit

#### Slide 1 notes

> Autonomous navigation in a structured grid arena. The presentation covers the task, final methods, development, results, collaboration and conclusions.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 1 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 2 notes

> Section 1: Task and system overview

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 2 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 3 notes

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 3 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 4 notes

> Different speeds on the left and right sides turn the robot. Tyre slip matters during turns and affects where the body centre moves.

**Audit:** Qualify physical causality: slip/low-speed/noise are plausible contributors; individual causal magnitudes were not measured. [代码/计算检查] (slide 4 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 4 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 5 notes

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 5 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 6 notes

> Section 2: Final methods

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 6 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 7 notes

> The EKF uses raw forward velocity and IMU yaw and yaw rate. AMCL provides map-to-odom correction; the EKF supplies odom-to-body motion. Their transform chain produces the control pose.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 7 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 7 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 8 notes

> Each particle represents x, y and heading. Odometry predicts relative motion; the scan likelihood changes the weight. This supplies global correction to local odometry.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 8 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 8 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 9 notes

> These are covariance criteria for a concentrated initial estimate, not a measured ground-truth accuracy guarantee. No forced departure timeout is used.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 9 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 9 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 10 notes

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 10 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 11 notes

> The origin here is a reference cell centre. Four-neighbour moves have equal cost and the Manhattan heuristic is measured in cell steps. Fixed walls remove connections; obstacles remove cells.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 11 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 11 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 12 notes

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 12 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 13 notes

> The lateral error is perpendicular distance to the fixed centreline. Positive means left. e_f is the lateral error at a point 0.10 m forward of the robot reference. It depends on both the body displacement and heading.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 13 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 13 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 14 notes

> The derivative uses filtered yaw rate. Integral clamping and actuator limits apply in implementation. For a straight reference the heading error derivative is the negative robot yaw rate. The minus sign of the lateral term makes left displacement produce right correction.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 14 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 14 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 15 notes

> We reuse the Stanley lateral-error form and scale it into yaw rate for the differential interface. It is not a full four-wheel tyre-slip model. The base converts forward speed and yaw rate into left-right motion.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 15 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 15 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 16 notes

> We round the corner reference, then choose a lookahead point along it. Pure Pursuit geometry computes curvature from the point in robot coordinates. Regulation reduces speed when curvature requires it. The final reference uses the original quarter-circle corner geometry, with nominal 0.30 m radius capped by the 0.58 m cell spacing.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 16 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 16 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 17 notes

> The stopping test is based on progress along the final reference line. It does not guarantee zero lateral error. We removed repeated goal corrections because physical results were less consistent.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 17 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 17 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 18 notes

> The obstacle occupies a whole cell, but the laser may see only one face. Far and side boundaries are not the intended candidate faces.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 18 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 18 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 19 notes

> The surface crosses the cell centreline. A straight-line fit rejects scattered returns. The minimum span is visible width, not the assumed obstacle size.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 19 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 19 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 20 notes

> This is dynamic replanning after a newly observed obstacle. Updating the obstacle does not always require changing the route.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 20 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 20 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 21 notes

> Section 3: Development and iteration

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 21 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 22 notes

> These are functional groupings of many revisions, not three individual code changes. Development overlapped across localisation, control and obstacles. Only representative changes and challenges are discussed.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 22 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 22 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 23 notes

> The simple controller made the target, angle and stop position easy to inspect. Actual skid steering did not match fixed-centre rotation, creating offsets for the next segment.

**Audit:** Qualify physical causality: slip/low-speed/noise are plausible contributors; individual causal magnitudes were not measured. [代码/计算检查] (slide 23 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 23 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 24 notes

> We did not conduct a controlled equal-condition benchmark of every controller. Physical performance guided the final choice. Excessive gains, shorter lookahead and increased speed could amplify noise and slip.

**Audit:** Correctly disclaims controlled benchmark. Replace categorical noise/slip amplification with observed less-consistent tracking unless measured. [代码/计算检查] (slide 24 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 24 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 25 notes

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 25 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 26 notes

> The laser-to-body transform affects what physical point the pose represents. Correcting it improved stopping. Slip changes the effective turning centre, so a correct final angle can still leave a positional offset.

**Audit:** Qualify physical causality: slip/low-speed/noise are plausible contributors; individual causal magnitudes were not measured. [代码/计算检查] (slide 26 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 26 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 27 notes

> Large obstacles may be only partly visible. The visible-span check avoids demanding a full box outline. Low-speed response, pose fluctuations and slip can make repeated alignment counterproductive.

**Audit:** Qualify physical causality: slip/low-speed/noise are plausible contributors; individual causal magnitudes were not measured. [代码/计算检查] (slide 27 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 27 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 28 notes

> The startup change remained after faster speed trials were reverted. The robot’s physical trajectory and body position were important alongside the internally estimated pose.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 28 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 28 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 29 notes

> Section 4: Results and limitations

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 29 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 30 notes

> The assessment of centre accuracy is visual. Logged localisation residuals are not independent ground-truth measurements. Small lateral and corner offsets remain.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 30 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 30 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 31 notes

> The user confirmed that the demonstrated avoidance was dynamic replanning. The time is the recording duration. Source: /home/rb3302-7/Downloads/IMG_7340.mp4.

**Audit:** User confirmed dynamic replanning; video duration is verified. Exact video route and mapping to frozen source remain 未验证. [代码/计算检查] (slide 31 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 31 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 32 notes

> AMCL has recovery mechanisms. The limitation is the difficulty of quickly restoring reliable localisation after a large disturbance, not that AMCL has no recovery functionality.

**Audit:** General AMCL recovery capability exists; explicitly add that final recovery configuration and kidnapped-robot success were not verified. [代码/计算检查] (slide 32 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 32 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 33 notes

> Section 5: Team contributions

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 33 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 34 notes

> All three members contributed solution ideas. Discussion and feedback from physical testing guided which changes to keep.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 34 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 34 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 35 notes

> Section 6: Conclusion

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 35 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 36 notes

> Successful integration depended on reliable coordinate relationships and practical testing.

**Audit:** Supported as a baseline explanation; use the same qualifications as the visible slide. [代码/计算检查] (slide 36 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 36 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

#### Slide 37 notes

> Algorithm selection alone was not enough. The base’s physical behaviour and the meaning of its pose reference strongly affected the result.

**Audit:** Qualify physical causality: slip/low-speed/noise are plausible contributors; individual causal magnitudes were not measured. [代码/计算检查] (slide 37 source mapping; individual physical claims qualified above)

> Source: RB3302_Project_Report.docx and the confirmed physical test feedback. Code: SUCCESS_RPP_20261111_173648.

**Audit:** Provenance is too generic: replace with the specific source keys above; not every claim has physical validation and bundled logs precede freeze. [代码/计算检查] (slide 37 source mapping; individual physical claims qualified above)

**Speak:** “The notes must use specific evidence rather than a blanket claim of physical verification.”

### 10.5 Missing context worth adding verbally, without overloading the slides

- Rate trim is part of the actual final motion chain even though earlier slide planning intentionally omitted it. Explain it if asked about requested versus measured turn speed. [代码/计算检查] (turn_feedback.py)
  **Speak:** “The requested turn also passes through a small measured yaw-rate correction.”
- Standard RPP features such as Nav2 collision prediction, costmap speed scaling and recovery are not automatically inherited by our local implementation. [代码/计算检查] (final launch and RPP source)
  **Speak:** “Using the same pursuit formula does not import every Nav2 controller feature.”
- Graph clearance, arc checks and obstacle recognition address different risks; none alone proves the complete physical trajectory collision-free. [代码/计算检查] (grid.py/geometry/detector)
  **Speak:** “Planning clearance and real tracking accuracy must both be considered.”
- The last known uploaded experiment and the report snapshot must not be conflated during the demonstration. [代码/计算检查] (S versus P; 实机验证: Nov 12 logs)
  **Speak:** “We identify the demonstrated version before quoting its parameters or results.”

### 10.6 External technical references and reproducibility

These references explain established methods; project-specific values and results come from local evidence, not these websites.

- [Official Nav2 AMCL configuration](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/) — movement thresholds, model settings and recovery parameter meanings. This rolling documentation is explanatory, not proof of the exact robot binary/defaults.
- [Official Jazzy RPP configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/) — reference implementation features; our local tracker is not that plugin.
- [Nav2 AMCL source](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/motion_model/differential_motion_model.cpp) — historical model-review source identifier recorded in H; use the specific installed version for binary-level claims.

[代码/计算检查] (official method documentation, local source mapping; exact remote installed binaries 未验证)
**Speak:** “Established method references explain the algorithms; our own logs establish what happened in our tests.”

### 10.7 File identity at this review

The hashes identify reviewed files, not remote robot installation. No report, slide deck, runtime code or frozen backup was modified while preparing this handbook. [代码/计算检查] (read-only review plus new handbook creation)

- `/home/rb3302-7/RB3302_Project_Report.docx`
  - SHA256 `1615c20263d14dda3ce637367663eb85dcce9edbc122404fdee9482027d2753f`. [代码/计算检查] (local file digest)
- `/home/rb3302-7/RB3302_Project_Slides.pptx`
  - SHA256 `4ffc26076a03041f1dba52726d1b0579ceee5069b81a18b3297d146aaa995449`. [代码/计算检查] (local file digest)
- `/home/rb3302-7/Downloads/IMG_7340.mp4`
  - SHA256 `e4762e1becb12c0d30e311ac7e9c79fbf66792aca35a8076d3ae0104afb1566f`. [代码/计算检查] (local file digest)
- `/home/rb3302-7/rb3302_fresh/report/DEVELOPMENT_HISTORY.txt`
  - SHA256 `3810078079d73854f60731ba3ebedbab64b84f88f20ecaa883fb42ce934a1d38`. [代码/计算检查] (local file digest)
