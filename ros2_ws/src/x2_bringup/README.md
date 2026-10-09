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
