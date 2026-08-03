"""
Gripper ROS2 Node - Wraps existing Gripper class for ROS control
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
try:
    from magpie_msgs.srv import SetGripperPosition, SetGripperForce
    from magpie_msgs.msg import GripperState, DeliGraspParams
    from magpie_msgs.action import DeliGrasp
except ModuleNotFoundError as exc:
    if exc.name == 'magpie_msgs':
        raise ModuleNotFoundError(
            "magpie_msgs is not available in this shell. Rebuild and source your workspace: "
            "source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash"
        ) from exc
    raise
import numpy as np

# Import existing gripper class
from magpie_control.gripper import Gripper


class GripperNode(Node):
    """ROS2 node wrapping Gripper class for control"""

    def __init__(self):
        super().__init__('gripper_node')

        # Declare parameters
        self.declare_parameter('auto_detect_port', True)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('use_eflesh', False)
        self.declare_parameter('default_speed', 100)
        self.declare_parameter('default_torque', 200)

        # Get parameters
        auto_detect = self.get_parameter('auto_detect_port').value
        port = None if auto_detect else self.get_parameter('port').value
        use_eflesh = self.get_parameter('use_eflesh').value

        # Initialize gripper
        try:
            self.get_logger().info('Initializing gripper...')
            self.gripper = Gripper(
                servoport=port,
                debug=False,
                use_eflesh=use_eflesh
            )

            # Set default parameters
            default_speed = self.get_parameter('default_speed').value
            default_torque = self.get_parameter('default_torque').value
            self.gripper.set_speed(default_speed)
            self.gripper.set_torque(default_torque)

            self.get_logger().info('Gripper initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize gripper: {e}')
            raise

        # Create services
        self.srv_open = self.create_service(
            Trigger, 'gripper/open', self.open_callback)
        self.srv_close = self.create_service(
            Trigger, 'gripper/close', self.close_callback)
        self.srv_set_position = self.create_service(
            SetGripperPosition, 'gripper/set_position', self.set_position_callback)
        self.srv_set_force = self.create_service(
            SetGripperForce, 'gripper/set_force', self.set_force_callback)
        self.srv_calibrate = self.create_service(
            Trigger, 'gripper/calibrate', self.calibrate_callback)
        self.srv_reset_parameters = self.create_service(
            Trigger, 'gripper/reset_parameters', self.reset_parameters_callback)
        self.srv_clear_error = self.create_service(
            Trigger, 'gripper/clear_error', self.clear_error_callback)

        # Create action server for DeliGrasp
        self.action_server = ActionServer(
            self,
            DeliGrasp,
            'gripper/deligrasp',
            self.deligrasp_execute_callback
        )

        # Create publisher for gripper state
        self.pub_state = self.create_publisher(GripperState, 'gripper/state', 10)

        # Latest state snapshot — updated by publish_state, read by deligrasp callback
        self._latest_state = GripperState()
        self._deligrasp_state_log = []   # [(aperture_mm, force_N), ...]
        self._collecting_log = False

        # Create timer for publishing state
        self.timer = self.create_timer(0.1, self.publish_state)  # 10 Hz

        self.get_logger().info('Gripper Node initialized')

    def publish_state(self):
        """Publish current gripper state"""
        try:
            msg = GripperState()
            msg.position = self.gripper.get_aperture()
            force = self.gripper.get_force(finger='both')
            msg.force = float(np.mean(force)) if isinstance(force, list) else float(force)
            temp = self.gripper.get_temp(finger='both')
            msg.temperature = float(np.mean(temp)) if isinstance(temp, list) else float(temp)
            msg.is_moving = False  # TODO: implement is_moving detection
            msg.contact_detected = False  # TODO: implement contact detection

            # Individual finger positions in millimeters.
            msg.finger_positions = [
                self.gripper.get_aperture(finger='right'),
                self.gripper.get_aperture(finger='left'),
            ]

            self._latest_state = msg
            if self._collecting_log:
                self._deligrasp_state_log.append((msg.position, msg.force))
            self.pub_state.publish(msg)
        except Exception as e:
            self.get_logger().warning(f'Error publishing gripper state: {e}')

    def open_callback(self, request, response):
        """Service callback to open gripper"""
        try:
            self.get_logger().info('Opening gripper...')
            self.gripper.open_gripper()
            response.success = True
            response.message = 'Gripper opened successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to open gripper: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def close_callback(self, request, response):
        """Service callback to close gripper"""
        try:
            self.get_logger().info('Closing gripper...')
            self.gripper.close_gripper()
            response.success = True
            response.message = 'Gripper closed successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to close gripper: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def set_position_callback(self, request, response):
        """Service callback to set gripper position"""
        try:
            target_mm = request.position

            # Optional per-request speed scaling from [0.0, 1.0] to Dynamixel units.
            if request.speed > 0.0:
                speed_bits = int(min(max(request.speed, 0.0), 1.0) * 1023)
                speed_bits = max(speed_bits, 1)
                self.gripper.set_speed(speed_bits)

            self.get_logger().info(f'Setting gripper position to {target_mm:.2f} mm')
            self.gripper.set_goal_aperture(target_mm, finger='both', record_load=False)

            # Get actual position
            actual_mm = self.gripper.get_aperture()
            response.actual_position = actual_mm
            response.success = True
            response.message = f'Gripper position set to {actual_mm:.2f} mm'
        except Exception as e:
            response.success = False
            response.message = f'Failed to set position: {str(e)}'
            response.actual_position = 0.0
            self.get_logger().error(response.message)

        return response

    def set_force_callback(self, request, response):
        """Service callback to set gripper force limit"""
        try:
            # Gripper.set_force() uses the empirically-calibrated N→load polynomial
            # (Stephen Otto's thesis, p17). Do not bypass it with a raw linear scaling.
            self.get_logger().info(f'Setting gripper force limit to {request.max_force:.2f} N')
            self.gripper.set_force(request.max_force, finger='both')

            response.success = True
            response.message = f'Force limit set to {request.max_force:.2f} N'
        except Exception as e:
            response.success = False
            response.message = f'Failed to set force: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def calibrate_callback(self, request, response):
        """Service callback to calibrate gripper"""
        try:
            self.get_logger().info('Calibrating gripper...')
            # Open fully, wait for motion to complete, then close.
            # spin_once cannot be called inside a callback — it deadlocks.
            self.gripper.open_gripper()
            time.sleep(2.0)
            self.gripper.close_gripper()

            response.success = True
            response.message = 'Gripper calibrated successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to calibrate: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def reset_parameters_callback(self, request, response):
        """Service callback to reset gripper parameters to defaults"""
        try:
            self.get_logger().info('Resetting gripper parameters...')
            self.gripper.reset_parameters()
            response.success = True
            response.message = 'Gripper parameters reset successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to reset parameters: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def clear_error_callback(self, request, response):
        """Re-enable motor torque after an overload/error without resetting all parameters."""
        try:
            self.gripper.reset_packet_overload()
            response.success = True
            response.message = 'Motor torque re-enabled'
            self.get_logger().info('Gripper error cleared')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'clear_error failed: {e}')
        return response

    async def deligrasp_execute_callback(self, goal_handle):
        """Action callback for DeliGrasp execution"""
        self.get_logger().info('Executing DeliGrasp...')

        params = goal_handle.request.params
        feedback_msg = DeliGrasp.Feedback()

        try:
            # Start sampling /gripper/state topic data for the force log
            self._deligrasp_state_log = []
            self._collecting_log = True

            feedback_msg.phase = 'approach'
            feedback_msg.current_aperture = self._latest_state.position
            feedback_msg.current_force = self._latest_state.force
            goal_handle.publish_feedback(feedback_msg)

            if hasattr(self.gripper, 'deligrasp_async'):
                feedback_msg.phase = 'deligrasp'
                goal_handle.publish_feedback(feedback_msg)
                await self.gripper.deligrasp_async(
                    x=params.goal_aperture,
                    fc=params.initial_force,
                    dx=params.additional_closure,
                    df=params.additional_force,
                    complete=params.complete_grasp
                )
            else:
                self.get_logger().warning('deligrasp_async not available, using simple close')
                self.gripper.set_force(params.initial_force, finger='both')
                self.gripper.close_gripper()

            self._collecting_log = False

            # Final state and log come from /gripper/state topic samples
            final_aperture_mm = self._latest_state.position
            final_force_n     = self._latest_state.force
            force_log         = [f for _, f in self._deligrasp_state_log]

            result_msg = DeliGrasp.Result()
            result_msg.success = True
            result_msg.message = 'DeliGrasp completed successfully'
            result_msg.final_aperture = float(final_aperture_mm)
            result_msg.final_force    = float(final_force_n)
            result_msg.force_log      = force_log

            goal_handle.succeed()
            return result_msg

        except Exception as e:
            self._collecting_log = False
            self.get_logger().error(f'DeliGrasp failed: {e}')
            result_msg = DeliGrasp.Result()
            result_msg.success = False
            result_msg.message = f'DeliGrasp failed: {str(e)}'
            goal_handle.abort()
            return result_msg

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Gripper Node...')
        try:
            self.gripper.open_gripper()
        except:
            pass
        try:
            self.gripper.disconnect()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
