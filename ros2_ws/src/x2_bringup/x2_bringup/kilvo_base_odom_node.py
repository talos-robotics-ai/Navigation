"""On-robot base odometry: KILVO (chest-LiDAR IMU pose) + HAL waist/head joints -> /x2/odom (pelvis).

Runs on PC2 next to KILVO, replacing the laptop's RoboJuDo TCP link as the pose source:

    /kilvo/aft_mapped_to_init  (Odometry, `camera_init` -> `aft_mapped` = chest-LiDAR IMU)
    /aima/hal/joint/{waist,head}/state (aimdk_msgs/JointStateArray, BEST_EFFORT)
        T_odom_base = T_odom_imu * T_imu_base(q_waist, q_head)       (odom_frames.Extrinsics.from_fk)
    -> /x2/odom  (Odometry, `odom` -> `base_link` = pelvis)

`odom` IS KILVO's world frame (`camera_init`, gravity aligned, z up): no re-origin, so the cloud
`/kilvo/cloud_registered` (same world) needs no transform to feed the obstacle map.

Optional (param `crate_topic`, default '/fpose/crate_pose'; '' = off): the crate pose, which
boxTrack publishes in `rgbd_head_front`, re-expressed in `odom` on /x2/crate_pose (same chain as
robojudo_link_node: odom_frames.crate_in_odom), only while it is fresher than `crate_max_age_ms`.

aimdk_msgs is optional at import time (it exists only in the vendor workspace on the robot): without
it the node still runs, with waist = head = 0, and says so once.

Publishes nothing on /tf unless `publish_tf` (off: the vendor graph has its own /tf).
Its twist is the planner-unused zero twist: KILVO's twist is in the IMU frame, not the pelvis's.
"""
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from . import se3
from .odom_frames import R_IL, T_IL
from .onboard_chain import HAL_JOINTS, BasePoseChain, joints_from_hal
from .urdf_fk import Urdf

try:   # vendor message package: present on the X2 only
    from aimdk_msgs.msg import JointStateArray
    _AIMDK_ERR = None
except ImportError as _e:   # pragma: no cover - depends on the machine
    JointStateArray = None
    _AIMDK_ERR = _e

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)


class KilvoBaseOdom(Node):
    def __init__(self, **kw):
        super().__init__('kilvo_base_odom', **kw)
        P = self.declare_parameter
        P('kilvo_odom_topic', '/kilvo/aft_mapped_to_init')
        P('odom_topic', '/x2/odom')
        P('crate_topic', '/fpose/crate_pose')
        P('crate_out_topic', '/x2/crate_pose')
        P('crate_max_age_ms', 500.0)
        P('odom_frame', 'odom')
        P('base_frame', 'base_link')
        P('publish_tf', False)
        P('urdf_path', '')
        P('lidar_to_imu_R', [float(v) for v in R_IL])
        P('lidar_to_imu_t', [float(v) for v in T_IL])
        P('joint_stale_s', 2.0)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._odom_frame, self._base_frame = g('odom_frame'), g('base_frame')
        self._crate_max_age = float(g('crate_max_age_ms')) / 1e3
        self._joint_stale = float(g('joint_stale_s'))
        urdf_path = g('urdf_path')
        if not urdf_path:
            from ament_index_python.packages import get_package_share_directory
            urdf_path = os.path.join(get_package_share_directory('x2_bringup'), 'urdf', 'x2_ultra.urdf')
        self.chain = BasePoseChain(Urdf.load(urdf_path), g('lidar_to_imu_R'), g('lidar_to_imu_t'))
        self._t_joint = {}
        self._last_odom_d = None
        self._n_in = self._n_out = 0

        self._pub = self.create_publisher(Odometry, g('odom_topic'), 10)
        self._tf = None
        if g('publish_tf'):
            from tf2_ros import TransformBroadcaster
            self._tf = TransformBroadcaster(self)
        # KILVO publishes RELIABLE; a BEST_EFFORT reader matches either.
        self.create_subscription(Odometry, g('kilvo_odom_topic'), self._on_odom, BEST_EFFORT)

        if JointStateArray is None:
            self.get_logger().warn(f'aimdk_msgs not importable ({_AIMDK_ERR}): waist = head = 0 for the pelvis FK')
        else:
            for grp in HAL_JOINTS:
                self.create_subscription(JointStateArray, f'/aima/hal/joint/{grp}/state',
                                         self._joint_cb(grp), BEST_EFFORT)
        self._crate_pub = None
        if g('crate_topic'):
            self._crate_pub = self.create_publisher(PoseStamped, g('crate_out_topic'), 10)
            self.create_subscription(PoseStamped, g('crate_topic'), self._on_crate, 10)
        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(f'{g("kilvo_odom_topic")} + waist/head joints -> {g("odom_topic")} '
                               f'({self._odom_frame}->{self._base_frame}), crate: {g("crate_topic") or "off"}')

    def _joint_cb(self, grp):
        def cb(m):
            t = time.monotonic()
            if t - self._t_joint.get(grp, 0.0) < 0.02:   # the HAL publishes ~1 kHz; waist/head move slowly
                return
            q = joints_from_hal(grp, [j.name for j in m.joints], [j.position for j in m.joints])
            if not q:
                self.get_logger().warn(f'{grp}: unexpected joint array ({len(m.joints)} joints)',
                                       throttle_duration_sec=5)
                return
            self.chain.set_joints(q)
            self._t_joint[grp] = time.monotonic()
        return cb

    def _on_odom(self, m):
        self._n_in += 1
        d = self.chain.odom_dict(m)
        self._last_odom_d = d
        T = self.chain.base(d)
        p, q = se3.position(T), se3.quat(T)
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()   # receipt time: the planner's timeouts use it
        o.header.frame_id, o.child_frame_id = self._odom_frame, self._base_frame
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, p)
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = map(float, q)
        self._pub.publish(o)
        self._n_out += 1
        if self._tf:
            t = TransformStamped()
            t.header, t.child_frame_id = o.header, self._base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = map(float, q)
            self._tf.sendTransform(t)

    def _on_crate(self, m):
        if self._last_odom_d is None:
            return
        now = self.get_clock().now()
        stamp = rclpy.time.Time.from_msg(m.header.stamp)
        if stamp.nanoseconds and (now - stamp).nanoseconds * 1e-9 > self._crate_max_age:
            return
        p, o = m.pose.position, m.pose.orientation
        T = self.chain.crate({'position': [p.x, p.y, p.z], 'quat_xyzw': [o.x, o.y, o.z, o.w]}, self._last_odom_d)
        if T is None:
            return
        out = PoseStamped()
        out.header.stamp, out.header.frame_id = m.header.stamp, self._odom_frame
        pp, qq = se3.position(T), se3.quat(T)
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, pp)
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = map(float, qq)
        self._crate_pub.publish(out)

    def _heartbeat(self):
        now = time.monotonic()
        ages = {g: (now - t) for g, t in self._t_joint.items()}
        msg = f'odom in/out {self._n_in}/{self._n_out}; joint ages {{{", ".join(f"{g}: {a:.1f}s" for g, a in ages.items())}}}'
        if JointStateArray is not None and (len(ages) < len(HAL_JOINTS) or max(ages.values()) > self._joint_stale):
            self.get_logger().warn(msg + ' -- waist/head joints missing or stale; FK keeps the last values (0 if none)')
        else:
            self.get_logger().info(msg)
        self._n_in = self._n_out = 0


def main(args=None):
    rclpy.init(args=args)
    n = KilvoBaseOdom()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
