"""
UR5 ROS2 Node - Wraps existing UR5_Interface for ROS control
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

from magpie_msgs.srv import MoveJoint, MoveLinear, GetPose, SetSpeed

from magpie_control.ur5 import UR5_Interface
from magpie_control import poses
from magpie_control.ros_utils import (
    pose_msg_to_matrix, matrix_to_pose_msg, pose_vec_to_msg)

# servoJ/servoL defaults — per SDU Robotics RTDE API
# time: duration each call blocks (s) — match your publish rate (0.002 = 500 Hz)
# lookahead_time: smoothing window (s), valid range [0.03, 0.2]
# gain: proportional position gain, valid range [100, 2000]; higher = stiffer
_SERVO_TIME        = 0.002
_SERVO_LOOKAHEAD   = 0.1
_SERVO_GAIN        = 300

# Standard UR5 joint names expected by ROS tooling
_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class UR5Node(Node):
    """ROS2 node wrapping UR5_Interface for arm control."""

    def __init__(self):
        super().__init__('ur5_node')

        self.declare_parameter('robot_ip', '192.168.0.4')
        self.declare_parameter('publish_rate', 500)  # RTDE streams at 500 Hz
        self.declare_parameter('default_linear_speed', 0.25)
        self.declare_parameter('default_linear_accel', 0.5)
        self.declare_parameter('default_joint_speed', 1.05)
        self.declare_parameter('default_joint_accel', 1.4)

        robot_ip = self.get_parameter('robot_ip').value
        self.lin_speed = self.get_parameter('default_linear_speed').value
        self.lin_accel = self.get_parameter('default_linear_accel').value
        self.rot_speed = self.get_parameter('default_joint_speed').value
        self.rot_accel = self.get_parameter('default_joint_accel').value

        try:
            self.get_logger().info(f'Connecting to UR5 at {robot_ip}...')
            self.ur5 = UR5_Interface(robotIP=robot_ip)
            self.ur5.start()
            self.get_logger().info('UR5 connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to UR5: {e}')
            raise

        self._teach_mode = False

        # Publishers
        self.pub_joints = self.create_publisher(JointState, 'arm/joint_states', 10)
        self.pub_tcp = self.create_publisher(PoseStamped, 'arm/tcp_pose', 10)

        # Services
        self.create_service(MoveJoint,  'arm/move_j',      self.move_j_callback)
        self.create_service(MoveLinear, 'arm/move_l',      self.move_l_callback)
        self.create_service(GetPose,    'arm/get_pose',    self.get_pose_callback)
        self.create_service(SetSpeed,   'arm/set_speed',   self.set_speed_callback)
        self.create_service(Trigger,    'arm/move_safe',   self.move_safe_callback)
        self.create_service(Trigger,    'arm/stop',        self.stop_callback)
        self.create_service(Trigger,    'arm/teach_mode',  self.teach_mode_callback)

        # Servo subscriptions for high-frequency streaming
        self.create_subscription(JointState,   'arm/servo_j_cmd', self.servo_j_callback, 10)
        self.create_subscription(PoseStamped,  'arm/servo_l_cmd', self.servo_l_callback, 10)

        pub_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / pub_rate, self.publish_state)

        # allow publish_rate to be changed at runtime (e.g. `ros2 param set`)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info('UR5 Node initialized')

    def _on_set_parameters(self, params):
        """Apply runtime parameter changes. Recreates the publish timer when
        publish_rate changes, so the rate is tunable without restarting."""
        for p in params:
            if p.name == 'publish_rate':
                if p.value <= 0:
                    return SetParametersResult(
                        successful=False, reason='publish_rate must be > 0')
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / p.value, self.publish_state)
                self.get_logger().info(f'publish_rate set to {p.value} Hz')
        return SetParametersResult(successful=True)

    def publish_state(self):
        """Publish joint states and TCP pose at fixed rate."""
        try:
            now = self.get_clock().now().to_msg()
            q = self.ur5.get_joint_angles()

            js = JointState()
            js.header.stamp = now
            js.name = _JOINT_NAMES
            js.position = q.tolist()
            self.pub_joints.publish(js)

            tcp = PoseStamped()
            tcp.header.stamp = now
            tcp.header.frame_id = 'base'
            # raw 6D RTDE pose straight to a Pose msg — avoids a 6D->4x4->6D round-trip
            tcp.pose = pose_vec_to_msg(self.ur5.recv.getActualTCPPose())
            self.pub_tcp.publish(tcp)
        except Exception as e:
            self.get_logger().warning(f'Error publishing arm state: {e}')

    def _teach_mode_blocked(self, response):
        """Fill response and return True if teach mode is active."""
        if self._teach_mode:
            response.success = False
            response.message = 'Teach mode active — call /arm/teach_mode to disable first'
            return True
        return False

    def teach_mode_callback(self, request, response):
        """Toggle freedrive (teach) mode. Blocks all motion commands while active."""
        try:
            self.ur5.toggle_teach_mode()
            self._teach_mode = not self._teach_mode
            state = 'enabled' if self._teach_mode else 'disabled'
            self.get_logger().info(f'Teach mode {state}')
            response.success = True
            response.message = f'Teach mode {state}'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'TeachMode toggle failed: {e}')
        return response

    def servo_j_callback(self, msg):
        """Stream joint-space servo target (high frequency). Publish to /arm/servo_j_cmd."""
        if self._teach_mode:
            self.get_logger().warning('ServoJ blocked: teach mode active', throttle_duration_sec=2.0)
            return
        q = list(msg.position)
        if len(q) != 6:
            self.get_logger().warning(f'ServoJ: expected 6 joints, got {len(q)}')
            return
        try:
            self.ur5.ctrl.servoJ(q, 0.0, 0.0, _SERVO_TIME, _SERVO_LOOKAHEAD, _SERVO_GAIN)
        except Exception as e:
            self.get_logger().error(f'ServoJ failed: {e}')

    def servo_l_callback(self, msg):
        """Stream Cartesian servo target (high frequency). Publish to /arm/servo_l_cmd."""
        if self._teach_mode:
            self.get_logger().warning('ServoL blocked: teach mode active', throttle_duration_sec=2.0)
            return
        try:
            matrix = pose_msg_to_matrix(msg.pose)
            vec = poses.pose_mtrx_to_vec(np.array(matrix))
            self.ur5.ctrl.servoL(vec, 0.0, 0.0, _SERVO_TIME, _SERVO_LOOKAHEAD, _SERVO_GAIN)
        except Exception as e:
            self.get_logger().error(f'ServoL failed: {e}')

    def move_j_callback(self, request, response):
        """Move to joint configuration."""
        if self._teach_mode_blocked(response):
            return response
        try:
            q = list(request.joint_positions)
            speed = request.speed if request.speed > 0.0 else self.rot_speed
            accel = request.acceleration if request.acceleration > 0.0 else self.rot_accel
            self.get_logger().info(f'MoveJ to {[f"{v:.3f}" for v in q]}')
            self.ur5.moveJ(q, rotSpeed=speed, rotAccel=accel,
                           asynch=request.async_mode)
            response.success = True
            response.message = 'MoveJ complete'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'MoveJ failed: {e}')
        return response

    def move_l_callback(self, request, response):
        """Move end-effector linearly to target pose."""
        if self._teach_mode_blocked(response):
            return response
        try:
            matrix = pose_msg_to_matrix(request.target_pose)
            speed = request.speed if request.speed > 0.0 else self.lin_speed
            accel = request.acceleration if request.acceleration > 0.0 else self.lin_accel
            self.get_logger().info(
                f'MoveL to xyz=[{request.target_pose.position.x:.3f}, '
                f'{request.target_pose.position.y:.3f}, '
                f'{request.target_pose.position.z:.3f}]'
            )
            self.ur5.moveL(matrix, linSpeed=speed, linAccel=accel,
                           asynch=request.async_mode)
            response.success = True
            response.message = 'MoveL complete'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'MoveL failed: {e}')
        return response

    def get_pose_callback(self, request, response):
        """Return current TCP pose and joint angles."""
        try:
            response.current_pose = pose_vec_to_msg(self.ur5.recv.getActualTCPPose())
            response.joint_positions = self.ur5.get_joint_angles().tolist()
            response.success = True
            response.message = 'OK'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'GetPose failed: {e}')
        return response

    def set_speed_callback(self, request, response):
        """Update default linear and joint speeds."""
        try:
            if request.speed > 0.0:
                self.lin_speed = request.speed
                self.rot_speed = request.speed
            if request.acceleration > 0.0:
                self.lin_accel = request.acceleration
                self.rot_accel = request.acceleration
            response.success = True
            response.message = (f'Speed set to {self.lin_speed:.2f}, '
                                f'accel to {self.lin_accel:.2f}')
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def move_safe_callback(self, request, response):
        """Move to pre-defined safe joint configuration."""
        if self._teach_mode_blocked(response):
            return response
        try:
            self.get_logger().info('Moving to safe position...')
            self.ur5.move_safe(rotSpeed=self.rot_speed,
                               rotAccel=self.rot_accel, asynch=False)
            response.success = True
            response.message = 'At safe position'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'MoveSafe failed: {e}')
        return response

    def stop_callback(self, request, response):
        """Stop all arm motion immediately, including any active servo mode."""
        try:
            self.get_logger().warning('ARM STOP called')
            self.ur5.ctrl.servoStop()  # exit servo mode first if active
            self.ur5.ctrl.stopL()
            response.success = True
            response.message = 'Arm stopped'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self.get_logger().info('Shutting down UR5 Node...')
        try:
            self.ur5.ctrl.servoStop()  # exit servo mode if active
        except:
            pass
        try:
            self.ur5.stop()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UR5Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
