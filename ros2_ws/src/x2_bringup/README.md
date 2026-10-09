# x2_bringup

Laptop-side ROS 2 glue for the AgiBot X2 box pick-and-place stack (Jazzy, domain 77, localhost only).

* `robojudo_link_node` -- TCP JSON-lines client to the RoboJuDo walking process (127.0.0.1:8770, reconnects).
  Publishes `/x2/odom` (odom -> base_link), TF, `/x2/crate_pose` (odom frame, only if `crate_age_ms < 500`),
  `/x2/crate_visible`, `/x2/q_arm`, `/x2/arm_names`. Subscribes `/x2/cmd_vel_out` (vx, wz only; vy dropped,
  clipped to [0,0.5] / +-0.5), `/x2/arm_cmd` (14; empty = policy default), `/x2/hand_cmd` (20 = L10+R10; empty =
  default), `/estop` (latched -> zero velocity until false). Command line at 20 Hz; Twist older than 0.5 s -> 0.
* `dummy_obstacles_node` -- one far point at 10 Hz on `/local_voxel_map/obstacles` so A* plans.
* `config/x2_planner_params.yaml` -- overlay on the planner defaults (merged at launch; overlay wins).
* `launch/x2_nav.launch.py` -- link + dummy obstacles + planner (bridge/rviz off, domain 77) + `pnp_fsm_node`
  (`pnp:=false` to omit). `/mpc/cmd_vel` reaches `/x2/cmd_vel_out` only through the FSM.
* `tools/fake_robojudo.py` -- fake server (planar odom, crate in frame `odom`, `--crate-dropout A:B`, SIGUSR1 toggles).
* `tools/compute_x2_extrinsics.py` -- derives the default extrinsics below from the URDF + KILVO config.

Run: `python3 tools/fake_robojudo.py &` then `../../x2_run.sh` (venv + colcon notes inside), then
`ros2 service call /pnp/start std_srvs/srv/Trigger` (with ROS_DOMAIN_ID=77, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST).

## Frames (robot case: server `frame_id != "odom"`; `"odom"` = identity, for sim)
* KILVO `/kilvo/aft_mapped_to_init`: world `camera_init` (z up, gravity-aligned, arbitrary yaw; used as `odom`),
  body `aft_mapped` = the chest LiDAR's built-in IMU (KILVO HOWTOUSEKILVO.md section 5), NOT camera, NOT pelvis.
  KILVO does not publish a camera pose, and the crate camera (rgbd_head_front) differs from its VIO camera.
* `T_odom_base = T_odom_imu * T_imu_base`; `T_odom_crate = T_odom_imu * T_imu_cam * T_cam_crate`, where
  `/fpose/crate_pose` is in the URDF frame `rgbd_head_front` (boxTrack uses the same URDF FK).
* Params `tracked_to_base_{xyz,quat}`, `tracked_to_cam_{xyz,quat}` (quat xyzw) default to
  base: [-0.33299,-0.01488,0.08942] / [0.001157,0.705007,-0.000780,0.709199];
  cam:  [0.17236,-0.02741,0.02646] / [0.666033,0.664329,-0.239351,-0.240370],
  from KILVO `config/x2.yaml` extrinsic_R/T (LiDAR->IMU) + `x2_ultra.urdf` (torso_link -> lidar_chest_front,
  pelvis, rgbd_head_front) at waist = head = 0. Check: camera sits 0.066 m forward / 0.505 m above the pelvis, optical
  axis 40 deg below level (matches vhit_camera_server's "pitched 40 deg").
* Assumes waist joints and head yaw/pitch are ~0 (fixed params). Crate pose latency (~0.3 s) is not compensated.

## On-robot (PC2): KILVO + navigation, `x2_onboard.launch.py`
Deploy from the laptop with `ros2_ws/x2_onboard_deploy.sh` (rsync to PC2 `~/talos_nav_ws/src`, offline casadi wheel into
`~/talos_nav_ws/pydeps`, colcon build, pure-python tests); run with `~/talos_nav_ws/run_nav_on_pc2.sh start [mc]|stop|status|log`
(source: `ros2_ws/x2_onboard_run_on_pc2.sh`). Vendor DDS profile, default domain: KILVO needs it for the HAL topics.

* `kilvo_base_odom_node` -- `/kilvo/aft_mapped_to_init` + `/aima/hal/joint/{waist,head}/state` (aimdk_msgs, BEST_EFFORT; optional
  import: without it waist = head = 0) -> `/x2/odom` (pelvis in `odom` = KILVO's `camera_init`). Optional `/fpose/crate_pose`
  (rgbd_head_front) -> `/x2/crate_pose` (odom). Pure logic in `onboard_chain.py`.
* Obstacles: `g1_local_map` fed with the world-frame `/kilvo/cloud_registered` and `/x2/odom` (`config/x2_onboard_local_map.yaml`:
  5 scans/s, 5 m window, floor removed relative to the local minimum; new optional params `max_rate_hz`, `output_frame`).
* `a_star_node` + `mpc_node` (overlay `x2_onboard_planner.yaml`) and `pnp_fsm` (`x2_onboard_fsm.yaml`: `manipulation: false`, i.e.
  ALIGN -> SETTLE -> DONE standing; the vendor `mc` owns the arms).
* `onboard_nav_container.py` (`x2_onboard_nav`) runs all of these as nodes of ONE rclpy context: ONE DDS participant for the nav
  stack (plus KILVO's own: `x2_leg_kinematics`, `kilvo`). No /rosout, no parameter services, no /tf listener.
  `split:=true` -> two participants if one process is CPU-bound.
* `walker:=none` (default): nothing commands the robot. `walker:=mc`: `mc_velocity_node` turns `/x2/cmd_vel_out` (the FSM gate's
  output, never the raw planner) into `McLocomotionVelocity` for the vendor controller as input source `talos_nav`, priority 64 (below
  the PS5 bridge's `talos_gamepad` 65: the pad overrides), no mode changes ever. Planner overlay `x2_planner_params_mc.yaml`.
  `deadband_mode: lift` (default): mc ignores |v| < 0.2 m/s and |wz| < 0.1 rad/s, so nonzero commands below that (above 0.03 / 0.02)
  are RAISED to the threshold (the robot moves faster than the planner asked, never slower); `zero` = the PS5 bridge's behaviour
  (stand), `none` = pass through. Stale input (> 0.3 s) -> zeros 0.3 s -> silence; `/estop` latches zeros. The FSM gate forwards vx and wz only (vy = 0).

## PC2 / laptop navigation split (relay)

PC2 keeps only the cheap things; the planner stack and the box detector run on the laptop.

```
PC2  (x2_onboard.launch.py mode:=relay, process x2_onboard_nav --nodes base_odom,relay,image[,mc])
  kilvo_base_odom    /x2/odom, /x2/cam_pose (T_odom_cam, rgbd_head_front optical frame, FK), [/x2/crate_pose: crate_on_pc2:=true]
  nav_relay_server   TCP 0.0.0.0:5596   <->  laptop nav_relay_client
  nav_image_server   TCP 0.0.0.0:5597    ->  laptop detector (head RGB-D; idle, unsubscribed, without a client)
laptop (x2_laptop_nav.launch.py, domain 77 localhost; ros2_ws/x2_laptop_run.sh)
  nav_relay_client + g1_local_map + A* + MPC + pnp_fsm + crate_to_odom        operator: ros2_ws/x2_nav go|reset|estop|clear
```

Port 5596 (frames: `uint32 BE header_len, uint32 BE payload_len, JSON header, payload`; `nav_relay_proto.py`):
PC2 -> laptop `odom|cam|crate {"t","stamp","p":[xyz],"q":[xyzw]}` (odom, cam <= 50 Hz), `cloud {"t","stamp","n"}` +
n*3 little-endian float32 xyz in odom (KILVO scan, 6 m radius around /x2/odom, 0.1 m voxel, 2 Hz), `hb`.
Laptop -> PC2 `cmd {"t","vx","vy","wz"}` 20 Hz (zeros if the local Twist is older than 0.3 s), `estop {"t","value"}`, `hb`.
Safety: PC2 publishes /x2/cmd_vel_out at 20 Hz while commands are < 0.3 s old; client gone or silent 0.3 s -> zero
Twists for 0.3 s, then nothing; mc_velocity_node adds its own stale -> zeros -> silence.

Port 5597 (same framing, `nav_image_proto.py`), at most `image_hz` = 10 per stream, latest-wins, images AS PUBLISHED
(module mounted upside down: the detector handles orientation):
* `info {"t","frame_id","width","height","K":[9],"D","distortion_model"}` on connect + 1 Hz
* `rgb {"t","stamp_sec","stamp_nsec","frame_id","width","height","encoding":"jpeg","src_encoding"}` payload JPEG (q80);
  `cv2.imdecode` -> BGR (`nav_image_proto.decode_rgb`)
* `depth {...,"encoding":"png16","unit":"mm"}` payload 16-bit PNG, mm, 0 = invalid (`decode_depth_m` -> metres float32)
* `hb`. Sources `/aima/hal/sensor/rgbd_head_front/{rgb_image,depth_image,rgb_camera_info}`; `rgb_source: compressed`
  forwards the vendor `rgb_image/compressed` instead of re-encoding.
Detector pairs rgb/depth by stamp (camera clock). Its output `/fpose/crate_pose` (frame rgbd_head_front) on the laptop
graph is turned into `/x2/crate_pose` (odom) by `crate_to_odom` using `/x2/cam_pose` at arrival - `latency_s` (0.3 s, TODO measure).
