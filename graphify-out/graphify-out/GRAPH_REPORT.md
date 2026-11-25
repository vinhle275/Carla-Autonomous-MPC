# Graph Report - graphify-out  (2026-11-20)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 1167 nodes · 2390 edges · 52 communities (42 shown, 10 thin omitted)
- Extraction: 88% EXTRACTED · 12% INFERRED · 0% AMBIGUOUS · INFERRED: 279 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `e1eeaeb9`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- OSQPVectorf
- LinearizedMpcController
- TelemetryData
- OSQPMatrix
- QpSolver
- qdldl_interface.c
- YellowLineDetector
- osqp_api.c
- ActuatorCommandData
- RoadCorridorTracker
- OSQPWorkspace_
- unified_app/main.cpp
- ObstacleFrenetOBB
- weights
- vehicle
- ActuatorTelemetryPkt
- GlobalTrajectory
- types.h
- util.c
- AraComPort
- CollisionDangerZone
- kkt.c
- ObstacleDetector
- csc_math.c
- carla_bridge_client.cpp
- frenet_utils.hpp
- osqp.h
- algebra_impl.h
- PerceptionSensorData
- qdldl
- carla_bridge_server.py
- CarlaBridgeClient
- CarlaBridgeDaemon
- .render
- VehicleStateData
- CrossSlice
- CockpitCameraViewer
- algebra_libs.c
- RawActorPacket
- .setup_simulation
- main.py
- BridgeMapCamHeader
- CorridorGroup
- CorridorResult
- TrajectoryPoint
- ActuatorOutput
- MpcOutput
- polish.h
- profilers.h
- launch.sh

## God Nodes (most connected - your core abstractions)
1. `LinearizedMpcController` - 77 edges
2. `OSQPVectorf` - 69 edges
3. `ObstacleFrenetOBB` - 40 edges
4. `OSQPWorkspace_` - 37 edges
5. `ActuatorCommandData` - 37 edges
6. `QpSolver` - 35 edges
7. `OSQPMatrix` - 32 edges
8. `adjoint_derivative_qdldl()` - 31 edges
9. `ActuatorTelemetryPkt` - 26 edges
10. `TelemetryData` - 25 edges

## Surprising Connections (you probably didn't know these)
- `main()` --references--> `ActuatorCommandData`  [INFERRED]
  apps/autonomous_core/main.cpp → libs/common/autosar_types.hpp
- `main()` --references--> `ActuatorCommandData`  [INFERRED]
  apps/paio_vehicle_interface/main.cpp → libs/common/autosar_types.hpp
- `main()` --references--> `PerceptionSensorData`  [INFERRED]
  apps/autonomous_core/main.cpp → libs/common/autosar_types.hpp
- `main()` --references--> `PerceptionSensorData`  [INFERRED]
  apps/paio_vehicle_interface/main.cpp → libs/common/autosar_types.hpp
- `main()` --references--> `TelemetryData`  [INFERRED]
  apps/autonomous_core/main.cpp → libs/common/autosar_types.hpp

## Import Cycles
- None detected.

## Communities (52 total, 10 thin omitted)

### Community 0 - "OSQPVectorf"
Cohesion: 0.06
Nodes (95): OSQPVectorf, OSQPVectori, adapt_rho(), check_termination(), compute_dual_res(), compute_dual_tol(), compute_duality_gap_tol(), compute_obj_val_dual_gap() (+87 more)

### Community 1 - "LinearizedMpcController"
Cohesion: 0.04
Nodes (44): LinearizedMpcController, active_chain_, active_obs_, buffer_accel_, buffer_idx_, buffer_steer_, corridor_tracker_, current_road_max_ (+36 more)

### Community 2 - "TelemetryData"
Cohesion: 0.04
Nodes (43): main(), TelemetryData, accel_cmd, brake_cmd, heading_error_rad, is_roadblock, jerk_filtered_m_s3, lateral_error_m (+35 more)

### Community 3 - "OSQPMatrix"
Cohesion: 0.12
Nodes (41): OSQPMatrix, csc_calloc(), csc_copy(), csc_cumsum(), csc_done(), csc_extract_diag(), csc_is_eq(), csc_malloc() (+33 more)

### Community 4 - "QpSolver"
Cohesion: 0.06
Nodes (30): QpSettings, adaptive_rho, alpha, eps_abs, eps_rel, max_iter, polishing, rho (+22 more)

### Community 5 - "qdldl_interface.c"
Cohesion: 0.12
Nodes (29): amd_l_order(), amd_order(), qdldl_solver, QDLDL_bool, QDLDL_float, QDLDL_int, _adj_assemble_csc(), _adj_perturb() (+21 more)

### Community 6 - "YellowLineDetector"
Cohesion: 0.06
Nodes (30): LineType, DASHED, NONE, SOLID, DetectedBeVLine, d_ego_m, fill_ratio, is_yellow (+22 more)

### Community 7 - "osqp_api.c"
Cohesion: 0.12
Nodes (27): osqp_algebra_default_linsys(), osqp_adjoint_derivative_compute(), osqp_adjoint_derivative_get_mat(), osqp_adjoint_derivative_get_vec(), osqp_capabilities(), osqp_cleanup(), osqp_codegen(), osqp_error_message() (+19 more)

### Community 8 - "ActuatorCommandData"
Cohesion: 0.05
Nodes (31): ActuatorCommandData, acceleration_cmd, active_obs_id, brake, bypass_side, current_idx, d_yellow_line_m, detect_time_ms (+23 more)

### Community 9 - "RoadCorridorTracker"
Cohesion: 0.09
Nodes (16): main(), CorridorBoundsData, is_junction, is_lateral_crossing, is_roadblock, is_yellow_solid, road_max, road_min (+8 more)

### Community 10 - "OSQPWorkspace_"
Cohesion: 0.06
Nodes (31): OSQPWorkspace_, Adelta_x, Atdelta_y, Aty, Ax, data, delta_x, delta_y (+23 more)

### Community 11 - "unified_app/main.cpp"
Cohesion: 0.12
Nodes (4): launch_carla_bridge_daemon(), main(), resolve_bridge_script(), terminate_carla_bridge_daemon()

### Community 12 - "ObstacleFrenetOBB"
Cohesion: 0.07
Nodes (28): ObstacleFrenetOBB, A_shift, affects_lane, box_yaw, center_x, center_y, d_center, d_max (+20 more)

### Community 13 - "weights"
Cohesion: 0.08
Nodes (25): mpc, horizon_N, max_iterations, sample_time_dt, s_curve, approach_margin_m, approach_ramp_m, hold_distance_m (+17 more)

### Community 14 - "vehicle"
Cohesion: 0.08
Nodes (25): danger_zone, clear_hold_time_s, length_m, moving_speed_threshold_m_s, width_m, roadblock, deceleration_m_s2, safe_stop_distance_m (+17 more)

### Community 15 - "ActuatorTelemetryPkt"
Cohesion: 0.08
Nodes (26): ActuatorTelemetryPkt, accel_cmd, active_obs_id, brake, bypass_side, current_idx, d_yellow_m, detect_ms (+18 more)

### Community 17 - "GlobalTrajectory"
Cohesion: 0.12
Nodes (18): GlobalTrajectory, is_closed_loop, s, theta, v, x, y, MovingObstacleWaypoint (+10 more)

### Community 18 - "types.h"
Cohesion: 0.11
Nodes (13): OSQPMatrix_, OSQPVectorf_, OSQPVectori_, osqp_linsys_solver_type, linsys_solver, nthreads, type, OSQPData (+5 more)

### Community 19 - "util.c"
Cohesion: 0.13
Nodes (15): c_strcpy(), copy_settings(), dump_csc_matrix(), dump_vec(), print_csc_matrix(), print_dns_matrix(), print_footer(), print_header() (+7 more)

### Community 20 - "AraComPort"
Cohesion: 0.11
Nodes (12): AraComPort, heap_fallback_, is_producer_, port_name_, seq_counter_, shm_ptr_, get_current_time_ns(), SharedMemoryBuffer (+4 more)

### Community 21 - "CollisionDangerZone"
Cohesion: 0.17
Nodes (11): CollisionDangerZone, front_overhang_, zone_length_, zone_width_, Point2D, x, y, ZoneCheckResult (+3 more)

### Community 22 - "kkt.c"
Cohesion: 0.36
Nodes (17): _count_diagonal_entries(), form_KKT(), _kkt_assemble_csc(), _kkt_assemble_csr(), _kkt_backshift_colptrs(), _kkt_colcount_block(), _kkt_colcount_diag(), _kkt_colcount_missing_diag() (+9 more)

### Community 24 - "ObstacleDetector"
Cohesion: 0.12
Nodes (9): ObstacleDetector, detection_range_, fov_deg_, half_fov_rad_, prev_positions_, surround_range_, track_drop_behind_, tracked_obstacles_ (+1 more)

### Community 25 - "csc_math.c"
Cohesion: 0.33
Nodes (15): csc_AtDA_extract_diag(), csc_Atxpy(), csc_Axpy(), csc_Axpy_sym_triu(), csc_col_norm_inf(), csc_lmult_diag(), csc_rmult_diag(), csc_row_norm_inf() (+7 more)

### Community 27 - "frenet_utils.hpp"
Cohesion: 0.14
Nodes (13): compute_obb_frenet_bounds(), FrenetProjection, closest_idx, d, nx, ny, s, tx (+5 more)

### Community 28 - "osqp.h"
Cohesion: 0.18
Nodes (3): osqp_error_type, _osqp_error(), _osqp_error_line()

### Community 29 - "algebra_impl.h"
Cohesion: 0.13
Nodes (12): OSQPMatrix_, csc, symmetry, OSQPMatrix_symmetry_type, NONE, TRIU, OSQPVectorf_, length (+4 more)

### Community 30 - "PerceptionSensorData"
Cohesion: 0.12
Nodes (14): PerceptionSensorData, actors, cam_h, cam_rgb, cam_w, has_cam_frame, is_junction, is_valid (+6 more)

### Community 31 - "qdldl"
Cohesion: 0.12
Nodes (13): qdldl, bp, Dinv, L, m, n, nthreads, P (+5 more)

### Community 33 - "CarlaBridgeClient"
Cohesion: 0.15
Nodes (9): CarlaBridgeClient, cached_cam_rgb_, connected_, fallback_state_, has_cached_cam_, host_, io_mutex_, port_ (+1 more)

### Community 36 - "VehicleStateData"
Cohesion: 0.17
Nodes (11): VehicleStateData, acceleration_m_s2, current_steer, is_valid, jerk_m_s3, sequence_id, timestamp_ns, velocity_m_s (+3 more)

### Community 37 - "CrossSlice"
Cohesion: 0.17
Nodes (10): CrossSlice, affects_lane, blocked, d_target, gaps, has_min_s_app_start, min_s_app_start, obs_list (+2 more)

### Community 39 - "algebra_libs.c"
Cohesion: 0.27
Nodes (8): LinSysSolver, adjoint_derivative_linsys_solver(), osqp_algebra_device_name(), osqp_algebra_free_libs(), osqp_algebra_init_libs(), osqp_algebra_init_linsys_solver(), osqp_algebra_linsys_supported(), osqp_algebra_name()

### Community 40 - "RawActorPacket"
Cohesion: 0.18
Nodes (11): RawActorPacket, extent_x, extent_y, id, is_vehicle, vx, vy, x (+3 more)

### Community 42 - "main.py"
Cohesion: 0.39
Nodes (3): save_moving_obstacles_to_csv(), save_obstacles_to_csv(), save_waypoints_to_csv()

### Community 43 - "BridgeMapCamHeader"
Cohesion: 0.22
Nodes (9): BridgeMapCamHeader, cam_h, cam_w, has_cam_frame, is_junction, map_is_yellow_solid, map_road_max, map_road_min (+1 more)

### Community 44 - "CorridorGroup"
Cohesion: 0.25
Nodes (8): CorridorGroup, A_shift, d_target, has_min_s_app_start, min_s_app_start, s_front, s_rear, side_sign

### Community 45 - "CorridorResult"
Cohesion: 0.25
Nodes (8): CorridorResult, d_left_legal, d_right_legal, is_junction, is_yellow_solid, lane_width, road_max, road_min

### Community 46 - "TrajectoryPoint"
Cohesion: 0.33
Nodes (6): TrajectoryPoint, s, theta_rad, v_m_s, x, y

### Community 47 - "ActuatorOutput"
Cohesion: 0.33
Nodes (5): ActuationMapper, ActuatorOutput, brake, steer, throttle

### Community 48 - "MpcOutput"
Cohesion: 0.40
Nodes (5): MpcOutput, acceleration_m_s2, cost, is_success, steer_rad

### Community 50 - "profilers.h"
Cohesion: 0.50
Nodes (3): OSQPProfilerEvent, OSQPProfilerItemInfo, OSQPProfilerSection

## Knowledge Gaps
- **409 isolated node(s):** `active_chain_`, `active_obs_`, `buffer_accel_`, `buffer_idx_`, `buffer_steer_` (+404 more)
  These have ≤1 connection - possible missing edges. (Counts symbols only; 583 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **10 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `LinearizedMpcController` connect `LinearizedMpcController` to `QpSolver`, `CrossSlice`, `YellowLineDetector`, `RoadCorridorTracker`, `ObstacleFrenetOBB`, `CorridorGroup`, `autosar_types.hpp`, `linearized_mpc_controller.cpp`, `ObstacleDetector`?**
  _High betweenness centrality (0.130) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `ActuatorCommandData` (e.g. with `main()` and `main()`) actually correct?**
  _`ActuatorCommandData` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `active_chain_`, `active_obs_`, `buffer_accel_` to the rest of the system?**
  _409 weakly-connected nodes found - possible missing edges._
- **Should `OSQPVectorf` be split into smaller, more focused modules?**
  _Cohesion score 0.06388206388206388 - nodes in this community are weakly interconnected._
- **Why does `QpSolver` connect `QpSolver` to `autosar_types.hpp`, `LinearizedMpcController`?**
  _High betweenness centrality (0.101) - this node is a cross-community bridge._
- **Should `LinearizedMpcController` be split into smaller, more focused modules?**
  _Cohesion score 0.038461538461538464 - nodes in this community are weakly interconnected._
- **Why does `ObstacleFrenetOBB` connect `ObstacleFrenetOBB` to `LinearizedMpcController`, `TelemetryData`, `CrossSlice`, `autosar_types.hpp`, `CollisionDangerZone`, `linearized_mpc_controller.cpp`, `ObstacleDetector`?**
  _High betweenness centrality (0.067) - this node is a cross-community bridge._