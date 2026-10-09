"""TCP JSON-lines client <-> ROS 2 for the RoboJuDo X2 walking process (127.0.0.1:8770).

server -> us (~20 Hz): odom, crate, q_arm, arm_names ...   (see README)
us -> server (20 Hz) : {"vx","wz","arm","hand_l","hand_r"}

Safety: the command line is ALWAYS sent at `send_hz`; velocity is zero when the last
/x2/cmd_vel_out is older than `cmd_timeout` or /estop is latched. vy is never sent.
"""
import json
import socket
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_ros import TransformBroadcaster

from . import se3
from .odom_frames import R_IL, T_IL, Extrinsics, base_in_odom, crate_in_odom
from .urdf_fk import Urdf

class RoboJuDoLink(Node):
    def __init__(self):
        super().__init__('robojudo_link')
        P = self.declare_parameter
        P('host', '127.0.0.1')
        P('port', 8770)
        P('send_hz', 20.0)
        P('cmd_timeout', 0.5)           # s: older Twist -> vx = wz = 0
        P('arm_timeout', 0.5)           # s: older arm/hand cmd -> null (policy default)
        P('crate_max_age_ms', 500.0)
        P('vx_max', 0.5)                # safety clip (policy: vx in [0, ~0.5])
        P('wz_max', 0.5)
        P('odom_frame', 'odom')
        P('base_frame', 'base_link')
        P('publish_tf', True)
        # Extrinsics, used only when the server reports frame_id != "odom" (robot / KILVO):
        # lidar->IMU from KILVO x2.yaml, lidar/pelvis/camera from the URDF FK over the measured joints.
        P('urdf_path', '')
        P('lidar_to_imu_R', [float(v) for v in R_IL])
        P('lidar_to_imu_t', [float(v) for v in T_IL])
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self._host, self._port = g('host'), int(g('port'))
        self._cmd_timeout, self._arm_timeout = float(g('cmd_timeout')), float(g('arm_timeout'))
        self._crate_max_age = float(g('crate_max_age_ms'))
        self._vx_max, self._wz_max = float(g('vx_max')), float(g('wz_max'))
        self._odom_frame, self._base_frame = g('odom_frame'), g('base_frame')
        from ament_index_python.packages import get_package_share_directory
        import os
        self._urdf = Urdf.load(g('urdf_path') or os.path.join(
            get_package_share_directory('x2_bringup'), 'urdf', 'x2_ultra.urdf'))
        self._R_il, self._t_il = g('lidar_to_imu_R'), g('lidar_to_imu_t')
        self._warned_joints = False
        self._ext = Extrinsics.from_fk(self._urdf, {}, self._R_il, self._t_il)   # joints = 0 until seen

        # command state
        self._vx = self._wz = 0.0
        self._t_cmd = -1e9
        self._arm = self._hand = None
        self._t_arm = self._t_hand = -1e9
        self._estop = False
        self._estop_logged = False

        # socket state
        self._sock = None
        self._sock_lock = threading.Lock()
        self._state = None
        self._state_seq = self._proc_seq = 0
        self._state_lock = threading.Lock()
        self._stop = False
        self._connected = False

        self._odom_pub = self.create_publisher(Odometry, '/x2/odom', 10)
        self._crate_pub = self.create_publisher(PoseStamped, '/x2/crate_pose', 10)
        self._vis_pub = self.create_publisher(Bool, '/x2/crate_visible', 10)
        self._qarm_pub = self.create_publisher(Float64MultiArray, '/x2/q_arm', 10)
        self._names_pub = self.create_publisher(String, '/x2/arm_names', 1)
        self._tf = TransformBroadcaster(self) if g('publish_tf') else None
        self.create_subscription(Twist, '/x2/cmd_vel_out', self._on_twist, 10)
        self.create_subscription(Float64MultiArray, '/x2/arm_cmd', self._on_arm, 10)
        self.create_subscription(Float64MultiArray, '/x2/hand_cmd', self._on_hand, 10)
        self.create_subscription(Bool, '/estop', self._on_estop, 10)
        self.create_timer(1.0 / float(g('send_hz')), self._send_tick)
        self.create_timer(0.02, self._process_tick)
        threading.Thread(target=self._reader, daemon=True).start()
        self.get_logger().info(f'RoboJuDo link -> {self._host}:{self._port}')

    # ---------------- ROS inputs
    def _now(self):
        return time.monotonic()

    def _on_twist(self, m: Twist):
        self._vx, self._wz, self._t_cmd = m.linear.x, m.angular.z, self._now()   # vy dropped

    def _on_arm(self, m: Float64MultiArray):
        d = list(m.data)
        self._arm, self._t_arm = (d if len(d) == 14 else None), self._now()
        if d and len(d) != 14:
            self.get_logger().warn(f'/x2/arm_cmd needs 14 values, got {len(d)}', throttle_duration_sec=2)

    def _on_hand(self, m: Float64MultiArray):
        d = list(m.data)
        self._hand, self._t_hand = (d if len(d) == 20 else None), self._now()
        if d and len(d) != 20:
            self.get_logger().warn(f'/x2/hand_cmd needs 20 values, got {len(d)}', throttle_duration_sec=2)

    def _on_estop(self, m: Bool):
        if m.data != self._estop:
            self.get_logger().warn(f'ESTOP {"LATCHED" if m.data else "released"}')
        self._estop = bool(m.data)

    # ---------------- command line out
    def _command_line(self):
        t = self._now()
        vx = wz = 0.0
        if not self._estop and t - self._t_cmd < self._cmd_timeout:
            vx = float(np.clip(self._vx, 0.0, self._vx_max))
            wz = float(np.clip(self._wz, -self._wz_max, self._wz_max))
        arm = self._arm if t - self._t_arm < self._arm_timeout else None
        hand = self._hand if t - self._t_hand < self._arm_timeout else None
        return {'vx': vx, 'wz': wz, 'arm': arm,
                'hand_l': hand[:10] if hand else None, 'hand_r': hand[10:] if hand else None}

    def _send_tick(self):
        with self._sock_lock:
            s = self._sock
        if s is None:
            return
        try:
            s.sendall((json.dumps(self._command_line()) + '\n').encode())
        except OSError as e:
            self.get_logger().warn(f'send failed: {e}', throttle_duration_sec=2)
            self._drop(s)

    # ---------------- reader thread (reconnect loop)
    def _drop(self, s):
        with self._sock_lock:
            if self._sock is s:
                self._sock = None
        try:
            s.close()
        except OSError:
            pass

    def _reader(self):
        while not self._stop:
            try:
                s = socket.create_connection((self._host, self._port), timeout=1.0)
                s.settimeout(1.0)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                self._connected = False
                time.sleep(0.5)
                continue
            with self._sock_lock:
                self._sock = s
            self._connected = True
            self.get_logger().info('connected to RoboJuDo server')
            buf = b''
            try:
                while not self._stop:
                    try:
                        chunk = s.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    buf += chunk
                    *lines, buf = buf.split(b'\n')
                    for ln in lines:
                        if ln.strip():
                            try:
                                st = json.loads(ln)
                            except ValueError:
                                continue
                            with self._state_lock:
                                self._state, self._state_seq = st, self._state_seq + 1
            except OSError:
                pass
            self._drop(s)
            self._connected = False
            self.get_logger().warn('RoboJuDo server disconnected; reconnecting')

    # ---------------- state -> ROS
    def _process_tick(self):
        with self._state_lock:
            st, seq = self._state, self._state_seq
        if st is None or seq == self._proc_seq:
            return
        self._proc_seq = seq
        now = self.get_clock().now()
        odom = st.get('odom')
        names, qj = st.get('joint_names'), st.get('q')
        if names and qj and len(names) == len(qj):      # waist + head move with the walking policy
            if not self._warned_joints:
                self._warned_joints = True
                missing = sorted(set(self._urdf.chain_joints('lidar_chest_front')
                                     + self._urdf.chain_joints('rgbd_head_front')) - set(names))
                if missing:
                    self.get_logger().warn(f'state lacks joints {missing}; FK uses 0 for them')
            self._ext = Extrinsics.from_fk(self._urdf, dict(zip(names, qj)), self._R_il, self._t_il)
        T_ob = None
        if odom is not None:
            T_ob = base_in_odom(odom, self._ext)
            self._publish_base(T_ob, odom, now)
        if st.get('arm_names') and not getattr(self, '_names_sent', False):
            self._names_pub.publish(String(data=json.dumps(st['arm_names'])))
            self._names_sent = True
        if st.get('q_arm') is not None:
            self._qarm_pub.publish(Float64MultiArray(data=[float(v) for v in st['q_arm']]))
        crate = st.get('crate')
        age = st.get('crate_age_ms')
        visible = False
        if crate is not None and age is not None and float(age) < self._crate_max_age:
            T = crate_in_odom(crate, odom, self._ext)
            if T is not None:
                visible = True
                msg = PoseStamped()
                msg.header.frame_id = self._odom_frame
                msg.header.stamp = (now - rclpy.duration.Duration(seconds=float(age) / 1e3)).to_msg()
                p, q = se3.position(T), se3.quat(T)
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
                (msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w) = map(float, q)
                self._crate_pub.publish(msg)
        self._vis_pub.publish(Bool(data=visible))

    def _publish_base(self, T, odom, now):
        p, q = se3.position(T), se3.quat(T)
        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id, o.child_frame_id = self._odom_frame, self._base_frame
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, p)
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = map(float, q)
        lv = odom.get('lin_vel')
        if lv:   # world-frame velocity (KILVO) / sim; planner does not use it
            o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, lv)
        self._odom_pub.publish(o)
        if self._tf:
            t = TransformStamped()
            t.header = o.header
            t.child_frame_id = self._base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = map(float, q)
            self._tf.sendTransform(t)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main():
    rclpy.init()
    n = RoboJuDoLink()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.destroy_node()
        finally:
            rclpy.try_shutdown()
