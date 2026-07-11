# Autonomous Car Modules Audit Report

**Audit date:** 2026-07-10  
**Scope:** `/home/kekec/alaz/modules` authored source, package metadata, launch/configuration wiring, generated artifacts, tests, and available build logs.  
**Verdict:** **Not ready for autonomous driving or vehicle testing.** The current global launch graph cannot form a complete Autoware pipeline. The hardware command path, emergency-stop path, lidar/camera topic graph, vehicle geometry, and perception coordinate semantics contain blocking conflicts. Do not test this stack on a moving vehicle until the Critical and High findings are resolved and hardware-in-the-loop safety tests pass.

## Remediation Status (2026-07-11)

Phase 1 addressed Critical findings C1-C3 for the custom Alaz competition car:

- Production bringup now selects `my_vehicle_launch` and no longer launches the legacy scalar `ros2_can_bridge`.
- `my_vehicle_launch` includes `my_vehicle_interface` and the source-tree `ros2_socketcan 1.3.0` transport.
- The interface defaults to a safety stop and continuously commands centered steering, zero throttle, neutral, and 100% brake during emergency, missing/stale control commands, or missing/stale mission emergency state.
- Mission control publishes a reliable transient emergency state before normal Run execution. Recovery is automatic when monitored inputs recover, as required by the project decision.
- Clean isolated build: 9 affected/dependency packages passed.
- Safety policy: 6/6 GTests passed.
- Source-level vehicle checks: 87 passed, 0 failed, 7 calibration/protocol warnings.
- Live ROS interface checks: 108 passed, 0 failed, 0 warnings.
- Mission-to-interface fault test: `healthy=false`, `camera fault=true`, `recovered=false` passed.

C4-C6 and all High/Medium/Low findings remain open. The verdict remains **not ready for vehicle motion**, particularly because perception, complete Autoware bringup, physical CAN verification, calibration, hardware E-stop behavior, and HIL testing are unresolved.

## Antigravity Fix Review (2026-07-11)

**Review scope:** changes currently present in the working tree after the Antigravity client edits, checked against the original Critical and High findings.

**Result:** The fixes are only partially OK. Several source-level changes are useful, but the stack is still **not ready for vehicle motion**. The biggest remaining blockers are duplicate perception packages, mission-control topic mismatches, emergency-stop release before reset, and incomplete validation of the full global launch graph.

### Blocking Findings From This Review

1. **C4 is not actually fixed in the normal `modules` workspace.**
   - The fixed bridge is under `modules/perception/detection_ws/src/autoware_detection_autoware_bridge`.
   - A second old bridge still exists under `modules/detection/detection_ws/src/autoware_detection_autoware_bridge`.
   - `colcon list --base-paths modules --packages-select autoware_detection_autoware_bridge --paths-only` resolves to the old `modules/detection/...` package.
   - The old bridge still copies image pixel coordinates into 3D object pose at `modules/detection/detection_ws/src/autoware_detection_autoware_bridge/autoware_detection_autoware_bridge/detection_autoware_bridge_node.py:168`.
   - **Status:** Not OK. Remove/exclude the duplicate detection workspace or apply the same fix to the active package.

2. **The new monocular 3D projection is safer than raw pixels but still not planner-safe.**
   - The fixed bridge waits for `CameraInfo` and publishes in `base_link`, which is an improvement.
   - It estimates depth from camera height, pitch, and the bottom of a 2D box at `modules/perception/detection_ws/src/autoware_detection_autoware_bridge/autoware_detection_autoware_bridge/detection_autoware_bridge_node.py:275`.
   - This is still a heuristic monocular estimate, not validated camera-lidar fusion or depth-based object localization.
   - **Status:** Partially OK for visualization/prototyping. Not OK as a trusted planner collision input without calibration and validation.

3. **Mission control still monitors old sensor topics.**
   - Central topics now define lidar as `/sensing/lidar/top/scan` and camera as `/sensing/camera/camera0/image_raw` in `modules/global_bringup/config/topics.yaml:4` and `modules/global_bringup/config/topics.yaml:8`.
   - Sensor launch remaps to those topics in `modules/sensor/rdw_sensor_kit_launch/launch/lidar.launch.xml:5` and `modules/sensor/rdw_sensor_kit_launch/launch/camera.launch.xml:4`.
   - `StartMode` and `EmergencyMode` still subscribe to `/sensing/scan` and `/sensing/image_raw` in `modules/mission_control/include/mission_control/mode_start.hpp:15` and `modules/mission_control/include/mission_control/mode_emergency.hpp:14`.
   - **Status:** Not OK. Start/emergency readiness will not see the renamed sensor topics.

4. **Emergency can publish `false` before operator reset is accepted.**
   - `MissionController::control_loop()` publishes `emergency_stop = emergency_mode_->isEmergencyTriggered()` before executing the current mode at `modules/mission_control/src/mission_control_launch.cpp:110`.
   - `EmergencyMode::execute()` stays in `MODE_EMERGENCY` while waiting for manual reset when topics are healthy at `modules/mission_control/src/mode_emergency.cpp:39`.
   - Therefore, when sensors recover but reset has not been requested, `/mission_control/emergency_stop` can become `false` while the state machine is still in emergency.
   - **Status:** Not OK. Emergency stop should remain active while `CURRENT_MODE == MODE_EMERGENCY` unless a reset has been accepted and the next mode is selected.

5. **Planning/vehicle geometry is still inconsistent in one active config file.**
   - `modules/vehicle/my_vehicle_description/config/vehicle_info.param.yaml:5` uses wheelbase `1.55`.
   - `modules/planning/config/behavior_path_planner.param.yaml` and `default_velocity_smoother.param.yaml` match the custom-car values.
   - `modules/planning/config/planning.yaml:6` still contains old larger vehicle values (`wheel_base: 2.7`, `max_steer_angle: 0.61`), though the current planning launch does not use that file by default.
   - **Status:** Mostly OK for the launched path, but stale config remains and can reintroduce H4 if used later.

6. **Duplicate detection workspaces remain a major repository conflict.**
   - Duplicate package names exist for `autoware_bytetrack`, `autoware_detection_autoware_bridge`, `autoware_tensorrt_yolox`, `autoware_traffic_light_classifier`, and `tier4_perception_launch`.
   - **Status:** Not OK. Keep one authoritative detection workspace or add explicit `COLCON_IGNORE`/build instructions for the inactive copy.

### Critical/High Status After Antigravity Changes

| Finding | Status | Notes |
|---|---|---|
| C1 vehicle wiring | OK from previous Phase 1 | Production bringup uses `my_vehicle_launch` and `my_vehicle_interface`. |
| C2 unsafe scalar CAN bridge | OK from previous Phase 1 | Legacy scalar bridge is out of production bringup. |
| C3 emergency actuator path | Regressed / not OK | Interface safety exists, but mission control can publish emergency false before reset and still uses stale sensor topics. |
| C4 invalid 2D-to-3D perception | Not OK | Fixed copy is not the package selected by `colcon --base-paths modules`; active duplicate still has pixel-to-3D behavior. |
| C5 incomplete global graph | Partially OK | `global.launch.py` now fails fast and includes more modules, but no full end-to-end Autoware launch validation passed. |
| C6 verification/safety case | Partially OK | More tests exist, but no HIL/real-car safety verification and full global graph is unverified. |
| H1 lidar topics | Partially OK | Sensor and clustering now share central topic, but mission control still uses old `/sensing/scan`. |
| H2 dead pointcloud relay | OK source-level | Replaced with real `pointcloud_to_laserscan/laserscan_to_pointcloud_node`; XML parses and the executable exists locally. |
| H3 camera topic conflict | Partially OK | Sensor/perception/localization moved toward central topic; mission control still uses old `/sensing/image_raw`. |
| H4 vehicle geometry conflict | Mostly OK | Launched planner configs match custom car, but stale `planning.yaml` still conflicts. |
| H5 control launch/config not installed | OK source-level | `control/CMakeLists.txt` now installs `launch/` and `config/`. |
| H6 control package names | Likely OK source-level | Names now match packages in `src/universe/autoware_universe`; overlay build/source still required. |
| H7 automatic recovery/engage | Not OK | Manual reset was added, but emergency-stop publication can clear before reset. Pause requires manual resume now. |
| H8 traffic light map association | Partially OK | Default traffic publication was disabled in code, but launch default still sets `publish_traffic_signals=true`; association remains unvalidated. |
| H9 localization hard coding | Partially OK | Hardcoded initial pose removed and projector YAML loading added; map path and quality gates remain limited. |
| H10 odometry contracts/units | Partially OK | Speed division and TF were fixed; odometry still consumes scalar `/vehicle_speed` and `/steering_angle` instead of Autoware vehicle status topics. |

### Validation Performed In This Review

- `python3 -m py_compile` passed for the changed Python launch/helper/bridge files.
- ROS XML frontend parsing passed for the changed sensor, perception, bridge, and lidar-clustering launch files.
- `global_bringup/scripts/get_topic.py` resolves the new lidar/camera/pointcloud topics.
- `pointcloud_to_laserscan` provides `laserscan_to_pointcloud_node` in the current ROS environment.
- Clean isolated `odometry` build passed.
- `odometry` GTest passed: 4/4.
- Clean isolated `mission_control` build passed.

### Required Next Fixes

1. Resolve the duplicate detection workspaces first. The current repo can build/run the old perception bridge even though a fixed copy exists elsewhere.
2. Move mission-control sensor topic definitions to the same central topics used by sensor launch, or pass them as parameters.
3. Keep `/mission_control/emergency_stop=true` while the state machine is in `MODE_EMERGENCY` and no manual reset has been accepted.
4. Either disable planner-facing monocular 3D object output or clearly gate it behind calibration/fusion validation.
5. Remove or align stale geometry/config files that conflict with the custom-car dimensions.

## Critical/High Remediation Pass (2026-07-11)

This pass addressed the remaining Critical and High findings only. Medium and Low findings are intentionally left for the next phase.

### Changes Made

- Added `modules/detection/detection_ws/src/COLCON_IGNORE` so the legacy duplicate detection workspace is no longer discovered by root `colcon --base-paths modules` builds.
- Kept the authoritative detection workspace under `modules/perception/detection_ws/src`; `colcon --base-paths modules/perception/detection_ws/src --packages-select autoware_detection_autoware_bridge` resolves to that package.
- Disabled planner-facing monocular 3D object and traffic-light publications by default in `autoware_detection_autoware_bridge` and `tier4_perception_launch`. The bridge no longer feeds heuristic camera-only 3D objects into planner topics unless explicitly enabled.
- Updated mission-control sensor watchdog topics to `/sensing/lidar/top/scan` and `/sensing/camera/camera0/image_raw`, matching sensor launch and `topics.yaml`.
- Latched `/mission_control/emergency_stop=true` while `CURRENT_MODE == MODE_EMERGENCY`; the stop flag cannot clear merely because sensors recovered before operator reset is accepted.
- Aligned stale `modules/planning/config/planning.yaml` vehicle dimensions with the custom-car dimensions used by `my_vehicle_description` and the active planner configs.
- Extended odometry to consume Autoware vehicle reports from `/vehicle/status/velocity_status` and `/vehicle/status/steering_status`, while preserving the older scalar inputs for compatibility.
- Added explicit odometry dependencies on `autoware_vehicle_msgs` and `tf2_ros`.
- Removed `global_bringup` runtime dependency cycles from leaf packages by using canonical topic defaults directly in leaf launch files.

### Critical/High Status After This Pass

| Finding | Status | Notes |
|---|---|---|
| C1 vehicle wiring | Fixed | Production bringup uses `my_vehicle_launch` and `my_vehicle_interface`. |
| C2 unsafe scalar CAN bridge | Fixed | Legacy scalar bridge is out of production bringup. |
| C3 emergency actuator path | Fixed source-level | Interface safety path exists, mission stop topic is latched during emergency, and watched sensor topics now match launched sensors. Still requires HIL/vehicle verification. |
| C4 invalid 2D-to-3D perception | Fixed by gating | Duplicate active package conflict is removed from root builds, and planner-facing monocular 3D outputs default to disabled. |
| C5 incomplete global graph | Partially fixed | Global launch now fails fast and production modules are wired, but full Autoware planning/control dependency graph was not built in this pass. |
| C6 verification/safety case | Partially fixed | Focused builds/tests pass, but HIL/real-car safety verification remains open. |
| H1 lidar topics | Fixed | Sensor launch, clustering, and mission control use `/sensing/lidar/top/scan`. |
| H2 dead pointcloud relay | Fixed source-level | Dead relay replaced by real LaserScan-to-PointCloud2 launch path. |
| H3 camera topic conflict | Fixed for active launch graph | Sensor, perception, localization fallback, and mission control use `/sensing/camera/camera0/image_raw`. |
| H4 geometry conflict | Fixed for current configs | Stale `planning.yaml` values now match custom-car dimensions. |
| H5 control launch/config install | Fixed source-level | `control` installs launch/config directories. |
| H6 control package names | Fixed source-level | Launch package names match local Autoware source package names. Full controller dependency build remains separate. |
| H7 automatic recovery/engage | Fixed source-level | Pause/manual reset behavior is operator-gated, and emergency stop remains latched while in emergency. |
| H8 traffic-light map association | Fixed by gating | Traffic-light publication defaults to disabled until map association is implemented/validated. |
| H9 localization hard coding | Partially fixed | Hardcoded initial pose removed and projector YAML loading added; map/localization quality still needs real-site validation. |
| H10 odometry contracts/units | Fixed source-level | Odometry now consumes Autoware vehicle status reports, uses m/s directly, and publishes `odom -> base_link` TF. |

### Validation Performed In This Pass

- Root package discovery no longer selects the legacy duplicate detection packages.
- Nested perception detection workspace builds: `autoware_detection_autoware_bridge`, `tier4_perception_launch`, `autoware_bytetrack`, `autoware_tensorrt_yolox`, and `autoware_traffic_light_classifier`.
- Changed XML launch files parse with ROS launch frontend.
- Changed Python files compile with `PYTHONDONTWRITEBYTECODE=1`.
- Clean isolated `odometry` build passed.
- `odometry` GTest passed: 4/4.
- Clean isolated `mission_control` build passed.
- A broader selected production build passed 9 packages, then stopped at `planning` because full Autoware planning/control dependency packages were not selected for this focused pass.
- `git diff --check` passes after whitespace cleanup.

### Remaining Before Vehicle Motion

- Build and source the full pinned Autoware planning/control dependency graph.
- Run full global launch validation with real or representative map/sensor inputs.
- Run CAN loopback, HIL, and closed-course tests with the real vehicle.
- Confirm camera calibration, lidar frame transform, M8N GNSS topic/driver, and final vehicle dimensions.

## Medium/Low Remediation Pass (2026-07-11)

This pass reviewed the Antigravity medium/low cleanup and fixed the remaining safe source/documentation gaps.

### Changes Made

- Confirmed the legacy `modules/detection/detection_ws` package graph has been removed from active root `colcon` discovery. No duplicate package names remain under `modules`.
- Removed remaining tracked Python bytecode artifacts and kept `.DS_Store`, `__pycache__`, `build/`, `install/`, and `log/` out of the authored tree.
- Corrected `.gitignore` so only the root `/src/` overlay is ignored; nested package `src` directories are no longer hidden accidentally.
- Added `.gitmodules` for `modules/sensor/sllidar_ros2`, matching the existing gitlink/submodule state.
- Added missing `rdw_sensor_kit_launch` runtime dependencies for the actually launched packages: `perception`, `sllidar_ros2`, `robot_state_publisher`, and `xacro`.
- Isolated `pseudo_vehicle_data` default topics under `/simulation/pseudo_vehicle/*` and kept legacy `/vehicle_speed` and `/steering_angle` only as an explicit test-bringup override.
- Registered `vehicle_simulator_node` as an installable pseudo-vehicle executable.
- Made `alaz_lidar_clustering` output and marker topics configurable and aligned the default cluster output with `global_bringup/config/topics.yaml`: `/perception/lidar_clustering/clusters`.
- Updated package metadata placeholders/versions/licenses in `global_bringup`, `localization`, `mission_control`, `ros2_can_bridge`, and `alaz_lidar_clustering`.
- Converted `modules/detection/launch/detection_launch.xml` into a deprecated stub so the removed detection workspace cannot be launched accidentally.
- Updated stale documentation for detection assets, localization, pseudo vehicle data, `my_vehicle_launch`, and RDW launch notes.
- Made perception detection model asset paths configurable with `ALAZ_DETECTION_ASSETS`, while keeping the existing `/workspace/modules/detection` Docker default.

### Medium/Low Status After This Pass

| Finding | Status | Notes |
|---|---|---|
| M1 duplicate packages | Fixed | Root `colcon` discovery now has no duplicate package names. |
| M2 generated artifacts tracked | Fixed | No tracked `__pycache__`, `.pyc`, `.DS_Store`, `build`, `install`, or `log` artifacts remain present on disk. Historical deletions are still visible in git status until committed. |
| M3 package discovery hazards | Fixed source-level | `.gitignore` no longer hides nested `src` trees, and `sllidar_ros2` is represented as a valid submodule via `.gitmodules`. |
| M4 `global_bringup` dependency metadata | Fixed source-level | Runtime package dependencies were expanded; full graph build still depends on the larger Autoware overlay. |
| M5 central configuration usage | Partially fixed | Sensor/perception/odometry paths use central topics or canonical defaults. `autoware_args.yaml` remains a future extension file rather than a complete Autoware arg source. |
| M6 runtime dependency installation | Fixed for perception NUC path | Runtime apt/pip installation was removed from the NUC script by Antigravity; dependencies belong in the container/provisioning layer. |
| M7 perception process supervision | Open | The main perception launch still delegates to a shell pipeline with weak process supervision. This is not safe to harden without deciding the production perception process model. |
| M8 sensor metadata | Fixed | Package metadata now matches the SLLidar/perception/xacro launch implementation. |
| M9 pseudo vehicle collision | Fixed | Default pseudo topics are simulation-namespaced, and test bringup explicitly opts into legacy scalar feedback only when needed. |
| M10 lidar package unreachable/conflicting output | Fixed | Console entry points exist and `alaz_lidar_2d` now publishes to `/perception/lidar_clustering/clusters` by default. |
| M11 historical Autoware version skew | Not directly actionable | Current authored tree no longer reproduces the historical missing-header logs in this pass. Full pinned Autoware overlay validation remains required. |
| L1 package metadata placeholders | Fixed for active authored packages scanned in this pass | Remaining TODOs are either tuning notes, future-extension comments, or third-party submodule code. |
| L2 documentation conflicts | Fixed for the stale active docs found | Detection, localization, pseudo vehicle, and vehicle launch docs were aligned with current source intent. |
| L3 duplicate/stale configs | Fixed for planning geometry | `planning.yaml` was removed by Antigravity; localization template files remain as documented templates. |
| L4 odometry logging | Fixed in previous critical/high pass | Per-publication INFO spam is no longer present in the active odometry source. |
| L5 portability | Partially fixed | Detection asset paths are now env-configurable. Some Docker/Autoware layout assumptions remain by design until the production deployment layout is finalized. |

### Validation Performed In This Pass

- Duplicate package scan under `modules`: no duplicates.
- Root package discovery lists 18 active packages, including `sllidar_ros2`, and no legacy detection workspace packages.
- Generated-artifact scan: no tracked generated artifacts remain present on disk.
- Python compile passed for changed pseudo-vehicle and lidar-clustering nodes with `PYTHONDONTWRITEBYTECODE=1`.
- ROS launch frontend parsed the changed lidar clustering, deprecated detection stub, and perception detection module XML files.
- Clean focused build passed 7 packages: `perception`, `rdw_sensor_kit_description`, `sllidar_ros2`, `alaz_lidar_clustering`, `pseudo_vehicle_data`, `localization`, and `rdw_sensor_kit_launch`. `sllidar_ros2` emitted third-party compiler warnings only.
- `git diff --check` passed.

## Severity Summary

| Severity | Count | Meaning |
|---|---:|---|
| Critical | 6 | Can prevent operation or create unsafe vehicle behavior |
| High | 10 | Breaks a major pipeline or invalidates essential output |
| Medium | 11 | Reliability, reproducibility, maintainability, or partial-function issue |
| Low | 5 | Hygiene/documentation issue |

## Critical Findings

### C1. Autoware control commands do not reach the RDW kart

- `global_bringup/config/bringup.yaml:2-9` enables `rdw_vehicle_launch` and `ros2_can_bridge`.
- `vehicle/rdw_vehicle_launch/launch/vehicle.launch.xml:13-15` includes `vehicle_interface.launch.xml`.
- `vehicle/rdw_vehicle_launch/launch/vehicle_interface.launch.xml:1-4` is empty: it starts no interface node.
- `ros2_can_bridge/ros2_can_bridge/bridge_node.py:33-37` subscribes to relative scalar topics (`throttle_cmd`, `brake_cmd`, `steering_cmd`, `gear_cmd`, `active_cmd`) using `std_msgs`.
- Autoware produces `/control/command/control_cmd` as `autoware_control_msgs/msg/Control` and `/control/command/gear_cmd` as `autoware_vehicle_msgs/msg/GearCommand`. There is no translator between these contracts.
- A compatible implementation exists in `vehicle/my_vehicle_interface`, but it is not included by the selected RDW launch. It also expects a separate `ros2_socketcan` bridge, which global bringup does not launch.

**Impact:** The planner/controller can run while the kart receives no corresponding command. Conversely, manually published scalar commands can operate the raw CAN bridge outside the Autoware command-gate/state-machine path.

**Required fix:** Select one vehicle interface architecture. Prefer wiring `my_vehicle_interface` (renamed/generalized for RDW) plus `ros2_socketcan` into `rdw_vehicle_launch`; remove or isolate the scalar `ros2_can_bridge` from production bringup.

### C2. The active raw CAN bridge has no command watchdog and conflicts with the documented CAN protocol

- `ros2_can_bridge/ros2_can_bridge/bridge_node.py:29-30` starts in drive gear and active state.
- `ros2_can_bridge/ros2_can_bridge/bridge_node.py:47-49` immediately starts periodic motor, steering, and brake frames.
- `ros2_can_bridge/ros2_can_bridge/bridge_node.py:94-106` retains the most recent command indefinitely. If a ROS command publisher dies, periodic CAN transmission continues with stale throttle/steering.
- `ros2_can_bridge/ros2_can_bridge/bridge_node.py:14-15` expects feedback IDs `0x120` and `0x130` and decodes both as little-endian floats.
- The tested interface and project documentation use speed `0x440` (big-endian integer), steering sensor `0x1E5`, steering ECU `0x720`, and motor ECU `0x730`; see `vehicle/my_vehicle_interface/include/my_vehicle_interface/can_utils.hpp:34-39`.
- Deactivation stops all periodic messages (`bridge_node.py:88-92`) rather than positively commanding zero throttle and a safe brake state.

**Impact:** Stale acceleration can continue after upstream failure. Real feedback is likely ignored or decoded incorrectly. This is unsafe for a drive-by-wire vehicle.

**Required fix:** Do not use this bridge on hardware. Add a monotonic command deadline, start disabled/neutral, validate feedback freshness, command an independently reviewed fail-safe state on timeout, and use one verified CAN definition.

### C3. Emergency state does not stop the vehicle

- `mission_control/src/mode_emergency.cpp:33-44` only publishes `/mission_control/emergency_stop` as `std_msgs/Bool`.
- No production module subscribes to `/mission_control/emergency_stop` to apply brake, remove throttle, disable autonomy, or trigger a hardware safety controller.
- `mission_control/src/mode_run.cpp:95-97` listens to a different topic, `/api/autoware/get/emergency`.
- The selected scalar CAN bridge has no subscription to either emergency topic.
- The unused `my_vehicle_interface` timeout sends zero throttle **and zero brake**, not a controlled stop (`vehicle/my_vehicle_interface/src/vehicle_interface_node.cpp:241-295`). Its ECU error/idle handling only logs warnings (`vehicle_interface_node.cpp:193-211`).

**Impact:** Entering `MODE_EMERGENCY` changes software state and publishes a flag but does not guarantee deceleration or actuator inhibition.

**Required fix:** Implement a fail-safe command gate/safety controller with a direct, tested actuator path; use normally-safe behavior, hardware E-stop integration, feedback confirmation, watchdogs, and latched recovery requiring explicit operator acknowledgement.

### C4. Camera detections are published as physically invalid 3D objects

- `perception/detection_ws/src/autoware_detection_autoware_bridge/.../detection_autoware_bridge_node.py:171-180` copies 2D image pixel center `(u, v)` into a 3D `Pose` `(x, y)`.
- The same bridge copies pixel width/height into metric `Shape` dimensions (`:183-189`).
- It then publishes these values as Autoware `DetectedObjects`, `TrackedObjects`, and `PredictedObjects` (`:92-141`) with the image header/frame.
- It marks every tracked object stationary (`:205-213`) and creates a one-pose “predicted” path (`:216-229`).

**Impact:** A 2D detection around pixel `(640, 360)` can be interpreted as an object hundreds of metres away with a hundreds-of-metres footprint. These messages must not be supplied to collision avoidance or motion planning.

**Required fix:** Do not bridge image detections directly to 3D Autoware objects. Publish feature/ROI messages for calibrated camera-lidar fusion, or estimate 3D pose with validated geometry/depth and transform it into the required vehicle/map frame. Only publish tracked/predicted types from real tracking/prediction stages.

### C5. Global bringup is not a complete autonomous-driving graph

- `global_bringup/launch/global.launch.py:23-58` only loads `bringup.yaml` and includes package launch files. `autoware_args.yaml` is loaded but never used; `topics.yaml` and `frames.yaml` are not loaded.
- `global_bringup/config/autoware_args.yaml:4-10` declares Autoware disabled, but this setting has no effect.
- No `autoware_launch` or equivalent full stack is included.
- `planning/launch/planning.launch.py:35-50` starts only behavior path planning and velocity smoothing. It lacks route/mission planning, scenario selection, behavior velocity planning, motion planning, obstacle avoidance, planning validation, and the surrounding Autoware data transforms/gates.
- `control/launch/control.launch.py` starts two controller nodes only and lacks command gating, operation-mode integration, and vehicle-command validation.
- Runtime lookup errors are caught and only printed by `global.launch.py:41-57`; an enabled module can be silently omitted while global launch continues.

**Impact:** `ros2 launch global_bringup global.launch.py` can appear to start successfully while essential modules are absent. It cannot provide an end-to-end autonomous route-to-actuator pipeline.

**Required fix:** Base bringup on a version-pinned Autoware launch graph and override vehicle/sensor/localization/perception components deliberately. Add launch-time contract assertions and fail fast when an enabled package, executable, file, or required argument is missing.

### C6. No safety case or production-grade verification exists

- `colcon test-result --all --verbose` reports `0 tests`, and numerous broken result/artifact links.
- The only executable vehicle verification suite is source-text/encoding verification. It reported 83 passes and 7 warnings, but performed no compiled C++ unit test, ROS graph test, CAN loopback test, actuator feedback test, timeout injection, or hardware-in-the-loop test.
- The suite labels “zero throttle, zero brake” on timeout as a safe stop, which is not generally a stop on a rolling vehicle.
- Mission control, odometry, localization, global bringup, planning, control, sensor wiring, and production perception have no CI-enforced integration tests.

**Impact:** Safety behavior and inter-module contracts are unverified. Passing the current test script must not be treated as evidence of vehicle safety.

**Required fix:** Add unit, launch, contract, fault-injection, SIL, CAN-loopback, HIL, and closed-course acceptance tests. Define explicit safe-state requirements and independently verify them.

## High Findings

### H1. Lidar topics are disconnected

- The SLLidar node publishes relative `scan`; inside the `sensing` namespace this becomes `/sensing/scan` (`sensor/sllidar_ros2/src/sllidar_node.cpp:62`, `sensor/rdw_sensor_kit_launch/launch/sensing.launch.xml:8-28`).
- Clustering is explicitly configured for `/scan` (`alaz_lidar_clustering/launch/lidar_clustering.launch.xml:2-6`), overriding its code default `/sensing/lidar/top/scan`.
- `global_bringup/config/topics.yaml:8-10` claims the scan is `/sensing/lidar/top/scan`.
- Mission control uses `/sensing/scan`, which is the only one of these that matches the current driver namespace.

**Impact:** The enabled lidar clustering node receives no real scans.

### H2. The advertised pointcloud pipeline relays a nonexistent source

- `sensor/rdw_sensor_kit_launch/launch/pointcloud_preprocessor.launch.py:13-21` relays `/sensing/lidar/top/pointcloud_raw` to `/sensing/lidar/concatenated/pointcloud`.
- No real sensor node publishes `/sensing/lidar/top/pointcloud_raw`; only an unlaunched dummy publisher does (`alaz_lidar_clustering/alaz_lidar_clustering/dummy_lidar.py`).
- The actual LaserScan-to-PointCloud converter is in `perception/launch/laserscan_to_pcl_and_occ.launch.xml` but is not included by the main `perception.launch.xml`; it is started only by a shell script after runtime detection.

**Impact:** Autoware's pointcloud input and any 3D clustering/fusion path are empty in the normal global launch.

### H3. Camera topic names conflict across sensing, configuration, localization, and perception

- The sensor launch publishes `/sensing/image_raw` (`sensor/rdw_sensor_kit_launch/launch/camera.launch.xml:3-10`).
- Global topics and localization expect `/sensing/camera/camera0/image_raw` (`global_bringup/config/topics.yaml:4-6`, `localization/launch/localization.launch.py:82-83`).
- Global bringup passes `/sensing/image_raw` to the outer perception launch, but the spawned NUC script independently scans for `/sensing/camera/camera0/image_raw` and does not recognize `/sensing/image_raw` (`perception/scripts/nuc_docker_perception.sh:569-598`).

**Impact:** YabLoc and camera detection can launch without receiving images; fallback camera info may be published on another topic tree.

### H4. RDW geometry conflicts with active planning and steering parameters

| Parameter | RDW description | Active planner config | Vehicle interface default |
|---|---:|---:|---:|
| Wheelbase | 1.05 m | 1.55 m | N/A |
| Wheel tread | 1.00 m | 1.40 m | N/A |
| Front overhang | 0.40 m | 0.80 m | N/A |
| Rear overhang | 0.27 m | 0.85 m | N/A |
| Max steer angle | 0.215 rad | 0.5236 rad | 0.5236 rad |

Sources: `vehicle/rdw_vehicle_description/config/vehicle_info.param.yaml`, `planning/config/behavior_path_planner.param.yaml`, and `vehicle/my_vehicle_interface/config/vehicle_interface.param.yaml`.

**Impact:** Collision footprint, turn feasibility, controller response, and CAN steering scaling describe different vehicles. Note that the RDW file comments call `0.215 rad` approximately 30 degrees, but it is approximately 12.3 degrees.

### H5. Control package launch/config is not installed

- `control/CMakeLists.txt` never installs `launch/` or `config/`.
- The current `install/control/share/control` contains package metadata only, no `launch/control.launch.py` or YAML files.
- Global bringup searches the installed package share and silently skips the missing launch file.

**Impact:** Control is enabled in YAML but omitted at runtime.

### H6. Control launch package names conflict with declared dependencies

- CMake/package metadata require `autoware_trajectory_follower_node` and `autoware_mpc_lateral_controller`.
- The launch file requests packages `trajectory_follower_node` and `mpc_lateral_controller` (`control/launch/control.launch.py:29-47`).
- None of those controller packages are currently discoverable in this host environment; the exact expected names therefore also require validation against the pinned Autoware version.

**Impact:** Even after installing the launch files, node lookup is likely to fail or bind to an incompatible Autoware release.

### H7. Mission control can engage automatically and recover from emergency without operator acknowledgement

- Pause always transitions to Run after five seconds (`mission_control/src/mode_pause.cpp:5-19`).
- Run publishes engage before checking whether any goal exists (`mission_control/src/mode_run.cpp:107-118`).
- With no goals, the machine cycles Pause -> Run -> Pause and republishes engage periodically.
- Emergency automatically returns to Pause as soon as watched topics are fresh (`mission_control/src/mode_emergency.cpp:33-44`), after which it returns to Run without a latched operator reset.
- Start marks localization successful after any single `nav_msgs/Odometry` message and never validates quality/state (`mission_control/src/mode_start.cpp:20,47-59,87-90`).

**Impact:** Autonomous engagement and emergency recovery are not guarded by a route, localization quality, vehicle readiness, or explicit operator action.

### H8. Traffic-light outputs cannot reliably associate with the map

- When no numeric ID is present, the bridge assigns `-(index + 1)` as the traffic-light group/lanelet ID (`perception/.../detection_autoware_bridge_node.py:291-307`).
- Image detector indices are not lanelet regulatory-element IDs.

**Impact:** The planner cannot reliably match a classified lamp to the traffic light controlling the current lane, so red-light behavior is not trustworthy.

### H9. Localization is hard-coded and its documented configuration is unused

- `localization/config/localization.yaml` claims global bringup reads and forwards it, but no code reads it.
- The map defaults to `/workspace/maps/map.osm` (`localization/launch/localization.launch.py:78`) regardless of `autoware_args.yaml`.
- Initial YabLoc pose is hard-coded to `(63.5139, 2.6648, yaw=pi)` and automatically published after five seconds (`localization.launch.py:179-193`).
- Projector information is hard-coded as Local/WGS84 with zero origin (`localization/map_projector_info_pub.py:20-25`) rather than being loaded from the selected map directory.
- There is no localization quality gate, pose/twist fusion, EKF, or configurable initialization workflow in this module.

**Impact:** Selecting another map or starting position can produce incorrect localization while mission control only checks topic presence.

### H10. Odometry contracts and units do not agree with available vehicle interfaces

- Odometry subscribes to scalar `/vehicle_speed` and `/steering_angle` (`odometry/include/odometry_node.hpp:48-50`).
- The Autoware-compatible interface publishes structured `/vehicle/status/velocity_status` and `/vehicle/status/steering_status`, so it does not feed odometry.
- The scalar CAN bridge uses the wrong feedback IDs and publishes an unlabelled float.
- Odometry divides input speed by 36 (`odometry/src/odometry_node.cpp:46`), implying the input is hectometres/hour; pseudo data explicitly generates metres/second, which would be 36 times too small.
- Odometry does not publish the `odom -> base_link` TF despite including TF dependencies.
- Its fallback speed simulation depends on `/throttle` `Float32`, while the scalar bridge uses `throttle_cmd` `Int32`; the topics/types do not connect.

**Impact:** Dead reckoning is disconnected or dimensionally wrong and does not complete the TF tree.

## Medium Findings

### M1. Duplicate, divergent perception packages

Five package names exist in both `detection/detection_ws/src` and `perception/detection_ws/src`:

- `autoware_bytetrack`
- `autoware_detection_autoware_bridge`
- `autoware_tensorrt_yolox`
- `autoware_traffic_light_classifier`
- `tier4_perception_launch`

The copies differ in launch topology and code. Notably, the perception copy has YOLOv8 and compatibility fixes, while the detection copy lacks them. Workspace-level `colcon list --base-paths modules` discovers the detection copies but stops recursion below the outer `perception` package; the NUC script separately builds and sources the perception-local workspace.

**Risk:** Behavior depends on overlay/source order. A build can silently run the older implementation.

### M2. Generated artifacts are tracked and stale

- 366 generated/cache files are tracked: 158 under build trees, 106 under install trees, 91 under logs, 14 Python cache matches, and one `.DS_Store` (categories overlap for cache files).
- At audit time, 240 broken symlinks existed under generated build/install/module trees, many pointing to a former `/workspace` checkout.
- `rosdep check` crashes on a missing installed `package.xml` instead of reporting dependencies.
- `colcon test-result` likewise encounters broken links.

**Risk:** Clean builds, package discovery, dependency checks, and runtime overlays are non-reproducible. Generated artifacts must be removed from version control and ignored, then rebuilt from source.

### M3. Essential current source is untracked

`git status --short` reports:

- `?? modules/global_bringup/scripts/`
- `?? modules/sensor/sllidar_ros2/`

`sllidar_ros2` also contains a nested `.git` directory. On a clean clone, the enabled sensor launch has no driver package, and `global_bringup/CMakeLists.txt` attempts to install a `scripts/` directory that may not exist. Perception directly calls `get_topic.py` from that untracked directory.

### M4. Global bringup omits runtime dependencies

`global_bringup/package.xml` declares launch/YAML dependencies but none of the packages enabled in `bringup.yaml`. This prevents correct dependency closure/build ordering and makes deployment incomplete unless the whole workspace happens to be present.

### M5. Global configuration gives a false source of truth

- `topics.yaml` and `frames.yaml` are mostly unused.
- `autoware_args.yaml` is read but not applied.
- The declared `vehicle_model` launch argument is never used.
- `_load_yaml` swallows parse/read failures and returns `{}`.

**Risk:** Operators can edit configuration with no runtime effect and receive no error.

### M6. Perception launch mutates the runtime environment

`perception/scripts/nuc_docker_perception.sh:410-438` runs `apt-get` and `pip3 install` during launch and can downgrade NumPy globally. It suppresses several setup failures and assumes ROS Humble, `/autoware`, and `/workspace`.

**Risk:** Startup requires root/network access, is nondeterministic, and can break ROS/OpenCV Python dependencies. Dependencies belong in a pinned container image or provisioning step.

### M7. Perception process supervision is weak

The main perception XML starts a large shell pipeline as a generic executable. The script starts many background processes and often prints success after fixed sleeps. Several missing inputs/packages downgrade behavior or skip fusion rather than failing production bringup.

**Risk:** The system can report a launched perception stack while producing no valid planner input.

### M8. Sensor metadata does not describe actual implementation

`rdw_sensor_kit_launch/package.xml` declares `usb_cam` and `urg_node`, while launch files actually use the local `perception` camera script and `sllidar_ros2`. It omits runtime dependencies on `perception`, `sllidar_ros2`, `robot_state_publisher`, and `xacro` used directly by launch files.

### M9. Pseudo vehicle package is incomplete and can collide with real feedback

- `vehicle_simulator_node.py` is not registered as a console entry point; only `auto_pseudo` is installable (`pseudo_vehicle_data/setup.py:25-29`).
- Pseudo nodes publish the same scalar `/vehicle_speed` and `/steering_angle` topics used by odometry and the raw CAN bridge.
- There is no simulation namespace or mutual-exclusion guard.

**Risk:** If enabled with hardware, simulated feedback can mix with real feedback.

### M10. Lidar package contains unreachable implementations

`alaz_lidar_clustering/setup.py` installs only `alaz_lidar_2d`. `dummy_lidar.py` and `lidar_cluster_node.py` have no console entry points. The installed 2D node publishes `.../detection/clusters`, while global topics claim `.../lidar_clustering/clusters`, and perception uses other cluster topics.

### M11. Historical build logs show Autoware version skew

`modules/log/build_2026-05-26_15-12-06/.../stderr.log` records missing deprecated Autoware headers and API mismatches in behavior planner code. That source is no longer present in the current authored tree, so this is historical evidence rather than a directly reproducible current failure, but it indicates unpinned/incompatible Autoware overlays.

## Low Findings

### L1. Package metadata contains placeholders

Several packages use `TODO` descriptions/licenses, placeholder maintainers, or `0.0.0` versions. Examples include `odometry/package.xml`, `pseudo_vehicle_data/package.xml`, and `alaz_lidar_clustering/package.xml`.

### L2. Documentation conflicts with source

- `detection/README.md` references `scripts/publish_dummy_camera.py`, which does not exist.
- `localization/README.md` says the module does not start YabLoc nodes, but the launch file does.
- `my_vehicle_interface/README.md` reports 79 tests while the current script reports 83.
- Several copied vehicle-launch READMEs still identify the package as archived `sample_vehicle_launch`.

### L3. Duplicate/stale configuration files

Planning has both `planning.yaml` and the actually launched `behavior_path_planner.param.yaml`; they contain different vehicle geometry. Localization has several local param YAMLs that the launch ignores in favor of upstream package defaults. This invites edits to inactive files.

### L4. Excessive runtime logging in odometry

`odometry/src/odometry_node.cpp:84` logs every publication at INFO at 20 Hz, causing log noise and avoidable I/O.

### L5. Repository portability is weak

Authored launch/scripts contain hard-coded `/workspace`, `/autoware`, ROS Humble, and map/model paths. Deployment layout is therefore part of implicit behavior rather than validated configuration.

## File Conflict Matrix

| Area | Source A | Source B | Conflict |
|---|---|---|---|
| Vehicle interface | `rdw_vehicle_launch/.../vehicle_interface.launch.xml` | `my_vehicle_interface/...` | Selected RDW interface is empty; compatible implementation is unused |
| CAN protocol | `ros2_can_bridge/bridge_node.py` | `my_vehicle_interface/can_utils.hpp` | Different feedback IDs and encodings |
| Camera image | `rdw_sensor_kit_launch/camera.launch.xml` | `global_bringup/topics.yaml`, localization, NUC script | `/sensing/image_raw` vs `/sensing/camera/camera0/image_raw` |
| Lidar scan | SLLidar under `sensing` namespace | clustering/global topics | `/sensing/scan` vs `/scan` vs `/sensing/lidar/top/scan` |
| Lidar pointcloud | sensor relay | actual publishers | Relay input has no real publisher |
| Vehicle geometry | `rdw_vehicle_description` | planning config/interface config | RDW dimensions/steering differ materially |
| Detection packages | `detection/detection_ws/src` | `perception/detection_ws/src` | Same package names, divergent implementations |
| Autoware enable/config | `autoware_args.yaml` | `global.launch.py` | Configuration is loaded/declared but not applied |
| Emergency | `/mission_control/emergency_stop` | `/api/autoware/get/emergency`, CAN nodes | No single end-to-end stop contract |
| Odometry | scalar CAN/pseudo topics | Autoware vehicle reports | Different message types, units, and topic names |

## Missing Production Capabilities

These are absent or not integrated in the normal global graph:

1. A single verified Autoware-compatible vehicle interface and CAN transport.
2. A latched emergency-stop/command-gate path that produces and confirms a safe actuator state.
3. Hardware enable/manual/autonomous mode authority and feedback, rather than always reporting autonomous.
4. Complete map/route/scenario/behavior/motion planning and planning validation.
5. Complete control command gating and vehicle command validation.
6. Valid metric 3D perception or calibrated fusion before publishing planner object topics.
7. Map-associated traffic-light detection/classification.
8. Consistent sensor topic/frame naming and a complete TF tree.
9. Fused localization/state estimation with quality monitoring and configurable initialization.
10. Diagnostics, health aggregation, data freshness/quality checks, and degraded-mode policy.
11. Deterministic dependency/container/version management.
12. Automated launch-contract, SIL, fault-injection, CAN-loopback, HIL, and closed-course tests.

## Module Status

| Module | Status | Main reason |
|---|---|---|
| `global_bringup` | Blocking | Silent skips, inactive config, incomplete system graph |
| `vehicle/rdw_vehicle_launch` | Blocking | Empty vehicle interface |
| `ros2_can_bridge` | Unsafe | Stale-command risk and incompatible feedback protocol |
| `vehicle/my_vehicle_interface` | Not integrated | Better message contract, but unused and fail-safe behavior incomplete |
| `sensor` | Blocking | Lidar pointcloud dead path and camera topic mismatch |
| `alaz_lidar_clustering` | Blocking | Wrong launched scan topic; output not integrated |
| `localization` | Blocking | Camera mismatch, hard-coded map/pose/projector, no quality/fusion gate |
| `perception` | Unsafe for planner | Invalid 2D-to-3D bridge; fragile shell orchestration |
| `detection` | Conflicting | Duplicate older perception packages and hard-coded deployment paths |
| `planning` | Incomplete | Wrong vehicle geometry and only a partial planning graph |
| `control` | Blocking | Launch/config not installed; package-name/version mismatch |
| `odometry` | Blocking | Topic/type/unit mismatch and missing TF publication |
| `mission_control` | Unsafe | Emergency has no actuator effect; automatic engage/recovery logic |
| `pseudo_vehicle_data` | Development only | Incomplete install and possible collision with hardware topics |

## Validation Performed

| Check | Result |
|---|---|
| Authored Python syntax | 96 files parsed, 0 syntax failures |
| Authored XML/Xacro syntax | 60 files parsed, 0 XML parse failures |
| Authored YAML syntax | 38 files parsed, 0 YAML parse failures |
| Authored shell syntax | 0 `bash -n` failures |
| ROS package discovery | 23 packages discovered by workspace-level `colcon list` |
| Duplicate package scan | 5 duplicate package names found across perception/detection workspaces |
| `rosdep check` | Could not complete because stale install tree references a missing package manifest |
| `colcon test-result --all --verbose` | 0 tests; stale/broken artifact errors |
| `pytest` | Command unavailable on host |
| Vehicle wiki script | 83 pass, 0 fail, 7 warnings when run from its package directory; static/source-level only |
| Existing generated links | 240 broken symlinks found |
| Existing tracked artifacts | 366 generated/cache files tracked |
| Live ROS launch/HIL | Not run: required Autoware packages, ROS overlays, hardware, and drivers are not available/healthy in this host environment |

## Remediation Order

1. **Freeze hardware testing.** Define the safe actuator state, E-stop architecture, enable authority, and watchdog requirements first.
2. **Choose one CAN/vehicle interface.** Integrate it into RDW launch with verified protocol, feedback, timeouts, and hardware E-stop behavior. Remove the production path to the other bridge.
3. **Pin one Autoware release and deployment image.** Remove runtime package installation and version-dependent overlays.
4. **Clean repository structure.** Remove generated build/install/log/cache content from Git, fix `.gitignore`, convert the nested lidar repository to a declared dependency/submodule/vendor source, and commit required bringup scripts.
5. **Replace global bringup with a fail-fast, complete Autoware graph.** Declare all runtime dependencies and make configuration effective.
6. **Unify topics, frames, QoS, and units in a machine-checked interface specification.** Fix camera/lidar/odometry/TF wiring.
7. **Use one vehicle geometry source.** Generate planner/controller/interface parameters from the selected RDW description and validate measured values.
8. **Remove the fake 2D-to-3D object bridge from planner topics.** Integrate calibrated fusion/3D estimation and map-aware traffic-light association.
9. **Redesign mission control around explicit readiness, route, operation mode, latched faults, and operator reset.** Connect emergency output to the command gate/actuator safety layer.
10. **Add CI and staged validation.** Clean build, lint/unit tests, launch tests, graph/type/QoS assertions, recorded-data SIL, fault injection, CAN loopback, HIL, then controlled closed-course testing.

## Final Assessment

There are both missing parts and direct file conflicts. The most serious issue is not a single bug but the absence of a coherent end-to-end contract: three different topic conventions, two incompatible CAN implementations, two divergent perception workspaces, inactive global configuration, partial Autoware planning/control launches, and an emergency state with no actuator effect. Syntax is mostly valid, so many nodes may start; that makes the system more dangerous because startup success does not imply correct or safe behavior.
