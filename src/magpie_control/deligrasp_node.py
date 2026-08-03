"""
DeliGrasp ROS2 Node - Perception-driven grasping pipeline

Pipeline per grasp:
  1. Detect object in color image (Grounding DINO)
  2. Get 3D position from depth image + camera intrinsics
  3. Transform camera-frame point to world frame using live TCP pose
  4. Move arm to approach pose above the object
  5. Descend to grasp pose
  6. Execute DeliGrasp (force-controlled gripper action)
  7. Retreat to approach height
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from magpie_msgs.srv import MoveLinear
from magpie_msgs.action import DeliGrasp
from magpie_msgs.msg import DeliGraspParams

from magpie_control.homog_utils import homog_xform, R_krot
from magpie_control.ros_utils import (
    pose_msg_to_matrix, matrix_to_pose_msg,
    pixel_to_camera_point, camera_point_to_world)

# TCP-to-camera transform — matches _CAMERA_XFORM in ur5.py
_TCP_TO_CAM = homog_xform(
    rotnMatx=R_krot([0.0, 0.0, 1.0], -np.pi / 2.0),
    posnVctr=[0.0, 0.0, 0.120],
)


# ── Node ───────────────────────────────────────────────────────────────────────

class DeliGraspNode(Node):

    def __init__(self):
        super().__init__('deligrasp_node')

        # ReentrantCallbackGroup lets the grasp service callback call other
        # services without deadlocking the single ROS executor thread.
        self.cbg = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('object_query', 'object')
        self.declare_parameter('detection_confidence', 0.3)
        self.declare_parameter('approach_height', 0.10)   # m above object
        self.declare_parameter('grasp_z_offset', 0.02)    # m, fine descent past approach
        self.declare_parameter('initial_force', 1.5)      # N
        self.declare_parameter('additional_force', 0.2)   # N
        self.declare_parameter('additional_closure', 1.0) # mm
        # 'grounding_dino' or 'owlvit'
        self.declare_parameter('detector_type', 'grounding_dino')
        # LLM: set use_llm=true and llm_instruction to derive object_query via OpenAI
        self.declare_parameter('use_llm', False)
        self.declare_parameter('llm_instruction', '')

        # ── State ───────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.color_image = None   # latest RGB frame (numpy HxWx3)
        self.depth_image = None   # latest depth frame (numpy HxW, mm uint16)
        self.camera_info = None   # sensor_msgs/CameraInfo
        self.tcp_matrix  = None   # latest TCP pose as 4x4 numpy array

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/gripper_camera/color/image_raw', self._color_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            Image, '/camera/gripper_camera/depth/image_rect_raw', self._depth_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            CameraInfo, '/camera/gripper_camera/color/camera_info', self._caminfo_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            PoseStamped, '/arm/tcp_pose', self._tcp_cb, 10,
            callback_group=self.cbg)

        # ── Service clients ──────────────────────────────────────────────────
        self.cli_move_l    = self.create_client(
            MoveLinear, '/arm/move_l', callback_group=self.cbg)
        self.cli_move_safe = self.create_client(
            Trigger, '/arm/move_safe', callback_group=self.cbg)
        self.cli_open      = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cbg)

        # ── Action client ────────────────────────────────────────────────────
        self.ac_deligrasp = ActionClient(
            self, DeliGrasp, '/gripper/deligrasp', callback_group=self.cbg)

        # ── Service server ───────────────────────────────────────────────────
        self.create_service(
            Trigger, 'grasp/execute', self._grasp_cb, callback_group=self.cbg)

        # ── Perception models ────────────────────────────────────────────────
        self._load_perception()
        self._check_realsense_usb_speed()

        self.get_logger().info('DeliGrasp Node ready')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _caminfo_cb(self, msg):
        self.camera_info = msg

    def _tcp_cb(self, msg):
        self.tcp_matrix = pose_msg_to_matrix(msg.pose)

    # ── Perception ─────────────────────────────────────────────────────────────

    def _check_realsense_usb_speed(self):
        """Warn if the RealSense D405 is connected to a USB <3.0 port (limits FPS to ~10)."""
        import subprocess, re
        try:
            out = subprocess.check_output(
                ['lsusb', '-v', '-d', '8086:0b5b'],
                stderr=subprocess.DEVNULL, text=True)
            match = re.search(r'bcdUSB\s+([\d.]+)', out)
            if match:
                version = float(match.group(1))
                if version < 3.0:
                    self.get_logger().warning(
                        f'RealSense D405 connected at USB {version:.2f} — '
                        'max ~10 FPS. Plug into a blue USB 3.0 port for 30 FPS.')
                else:
                    self.get_logger().info(f'RealSense D405 USB speed: {version:.2f} (OK)')
            else:
                self.get_logger().warning('RealSense D405 not detected via lsusb — is it plugged in?')
        except Exception:
            pass  # lsusb not available or camera not connected — not fatal

    def _load_perception(self):
        detector_type = self.get_parameter('detector_type').value
        self.detector = None
        if detector_type == 'grounding_dino':
            try:
                from magpie_perception.label_dino import LabelDINO
                self.detector = LabelDINO()
                self.get_logger().info('Grounding DINO loaded')
            except ImportError:
                self.get_logger().warning('magpie_perception not installed — detector unavailable')
        elif detector_type == 'owlvit':
            try:
                from magpie_perception.label_owl import LabelOWL
                self.detector = LabelOWL()
                self.get_logger().info('OWL-ViT loaded')
            except ImportError:
                self.get_logger().warning('magpie_perception not installed — detector unavailable')
        else:
            self.get_logger().warning(f'Unknown detector_type "{detector_type}" — use grounding_dino or owlvit')

    def _resolve_query(self):
        """Return the object query string, optionally via LLM."""
        if self.get_parameter('use_llm').value:
            instruction = self.get_parameter('llm_instruction').value
            if not instruction:
                self.get_logger().warning('use_llm=true but llm_instruction is empty — falling back to object_query')
            else:
                try:
                    return self._llm_extract_object(instruction)
                except Exception as e:
                    self.get_logger().error(f'LLM query extraction failed: {e} — falling back to object_query')
        return self.get_parameter('object_query').value

    def _llm_extract_object(self, instruction: str) -> str:
        """Use OpenAI + magpie_prompts to extract object name from a natural language instruction."""
        import re, ast
        from openai import OpenAI
        from magpie_prompts.prompts.dg_command_enumerator import prompt_command_enumerator
        client = OpenAI()  # reads OPENAI_API_KEY from environment
        response = client.chat.completions.create(
            model='gpt-4o-mini',
            messages=[
                {'role': 'system', 'content': prompt_command_enumerator},
                {'role': 'user',   'content': instruction},
            ],
        )
        text = response.choices[0].message.content
        match = re.search(r'\[start of enumeration\](.*?)\[end of enumeration\]', text, re.DOTALL)
        if not match:
            raise RuntimeError(f'LLM did not return expected format: {text}')
        parsed = ast.literal_eval(match.group(1).strip())
        objects = parsed.get('objects', [])
        if not objects:
            raise RuntimeError(f'LLM returned no objects: {parsed}')
        query = objects[0]
        self.get_logger().info(f'LLM resolved "{instruction}" → query="{query}"')
        return query

    def _detect(self, query, confidence):
        """Run detector. Returns (boxes, labels, scores) or raises."""
        if self.detector is None:
            raise RuntimeError('No detector loaded — check detector_type parameter')
        boxes, labels, scores = self.detector.label(self.color_image, query, confidence)
        return boxes, labels, scores

    # ── Service helpers ────────────────────────────────────────────────────────

    def _call(self, client, request, timeout=5.0):
        """Synchronous service call safe to use inside a ReentrantCallbackGroup."""
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'Service not available: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        return future.result()

    def _move_l(self, matrix, speed=0.10, accel=0.2):
        req = MoveLinear.Request()
        req.target_pose = matrix_to_pose_msg(matrix)
        req.speed = speed
        req.acceleration = accel
        req.async_mode = False
        resp = self._call(self.cli_move_l, req, timeout=30.0)
        if not resp.success:
            raise RuntimeError(f'MoveL failed: {resp.message}')

    # ── Main grasp pipeline ────────────────────────────────────────────────────

    def _grasp_cb(self, request, response):
        try:
            self._run_grasp_pipeline()
            response.success = True
            response.message = 'Grasp complete'
        except Exception as e:
            self.get_logger().error(f'Grasp failed: {e}')
            self._safe_abort()
            response.success = False
            response.message = str(e)
        return response

    def _run_grasp_pipeline(self):
        """Detect -> localise -> grasp. Each stage is its own method below."""
        self._check_inputs()
        query = self._resolve_query()
        u, v = self._detect_centroid(query)
        p_world = self._localize(u, v)
        self._execute_grasp(p_world)

    def _check_inputs(self):
        """Fail early if any camera/arm topic hasn't produced data yet."""
        for name, val in [('color image', self.color_image),
                          ('depth image', self.depth_image),
                          ('camera info', self.camera_info),
                          ('arm pose',    self.tcp_matrix)]:
            if val is None:
                raise RuntimeError(f'No {name} received yet — is the pipeline running?')

    def _detect_centroid(self, query):
        """Run the detector; return the pixel centroid (u, v) of the best box."""
        confidence = self.get_parameter('detection_confidence').value
        self.get_logger().info(f'Detecting: "{query}"')
        boxes, labels, scores = self._detect(query, confidence)
        if len(boxes) == 0:
            raise RuntimeError(f'No "{query}" detected in current view')

        best = int(np.argmax(scores))
        x1, y1, x2, y2 = boxes[best]
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        self.get_logger().info(
            f'Detected "{labels[best]}" (conf={scores[best]:.2f}) bbox=[{x1},{y1},{x2},{y2}]')
        return u, v

    def _localize(self, u, v):
        """Sample depth at (u, v) and return the object's world-frame position."""
        pad = 5
        roi = self.depth_image[
            max(0, v - pad):v + pad,
            max(0, u - pad):u + pad,
        ].astype(float)
        valid = roi[roi > 0]
        if len(valid) == 0:
            raise RuntimeError('Depth image has no valid pixels at detection centroid')
        depth_m = float(np.median(valid)) / 1000.0   # mm → m

        p_cam   = pixel_to_camera_point(u, v, depth_m, self.camera_info.k)
        p_world = camera_point_to_world(p_cam, self.tcp_matrix, _TCP_TO_CAM)
        self.get_logger().info(
            f'Object world position: x={p_world[0]:.3f} y={p_world[1]:.3f} z={p_world[2]:.3f} m')
        return p_world

    def _execute_grasp(self, p_world):
        """Open, approach above the object, descend, DeliGrasp, and retreat."""
        approach_h = self.get_parameter('approach_height').value
        grasp_off  = self.get_parameter('grasp_z_offset').value

        # Open gripper
        self.get_logger().info('Opening gripper')
        self._call(self.cli_open, Trigger.Request())
        time.sleep(0.3)

        # Approach pose: keep TCP orientation, translate XY to object, Z to approach height
        approach = np.array(self.tcp_matrix)
        approach[0, 3] = p_world[0]
        approach[1, 3] = p_world[1]
        approach[2, 3] = p_world[2] + approach_h
        self.get_logger().info('Moving to approach pose')
        self._move_l(approach, speed=0.15, accel=0.3)

        # Descend to grasp pose
        grasp = approach.copy()
        grasp[2, 3] = p_world[2] + grasp_off
        self.get_logger().info('Descending to grasp pose')
        self._move_l(grasp, speed=0.05, accel=0.1)

        # Force-controlled close
        self._run_deligrasp()

        # Retreat
        self.get_logger().info('Retreating')
        retreat = grasp.copy()
        retreat[2, 3] += approach_h
        self._move_l(retreat, speed=0.10, accel=0.2)

    def _run_deligrasp(self):
        """Send the DeliGrasp action goal and wait for the force-controlled close."""
        self.get_logger().info('Executing DeliGrasp')
        params = DeliGraspParams()
        params.goal_aperture    = 30.0
        params.initial_force    = float(self.get_parameter('initial_force').value)
        params.additional_force = float(self.get_parameter('additional_force').value)
        params.additional_closure = float(self.get_parameter('additional_closure').value)
        params.complete_grasp   = True

        if not self.ac_deligrasp.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('DeliGrasp action server not available')

        goal_future = self.ac_deligrasp.send_goal_async(
            DeliGrasp.Goal(params=params))
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        gh = goal_future.result()
        if not gh.accepted:
            raise RuntimeError('DeliGrasp goal rejected by action server')

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f'DeliGrasp returned failure: {result.message}')

        self.get_logger().info(
            f'Grasped — aperture={result.final_aperture:.1f}mm '
            f'force={result.final_force:.2f}N')

    def _safe_abort(self):
        """Best-effort safety recovery: open gripper and go to safe pose."""
        for client, req in [(self.cli_open, Trigger.Request()),
                            (self.cli_move_safe, Trigger.Request())]:
            try:
                self._call(client, req, timeout=5.0)
            except Exception:
                pass

    def destroy_node(self):
        self.get_logger().info('Shutting down DeliGrasp Node')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeliGraspNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
