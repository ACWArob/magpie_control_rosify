<!--
  docs/RUNNING.md
  WHAT THIS IS: a per-node "run it by hand in one terminal" reference for the
                magpie_control ROS 2 stack (gripper, UR5, F/T, tactile, deligrasp).
  WHY IT'S HERE: the launch file starts all five nodes at once, which is great for a
                demo but hides which node owns which device. During bring-up and
                debugging you almost always want to start ONE node on its own — to
                check its hardware, read its topics, or restart it without killing the
                rest. This doc is that map: for each node, what it needs running first,
                the exact command, and how to confirm it's alive.
-->

# Running the ROS 2 Nodes Individually

> **What this is for:** starting each `magpie_control` node on its own, in its own terminal, during bring-up or debugging.
> **Why it's here:** `gripper_control.launch.py` starts all five nodes together — convenient, but it hides which node talks to which device and makes it hard to restart just one. When something isn't working, you run the single node that owns that hardware and watch it directly. This is the per-node map for doing that.

Every node lives in the `magpie_control` package and is launched with `ros2 run magpie_control <node>`. They share one parameter file, `config/gripper_config.yaml`.

---

## Before anything — source the workspace (every new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash
```

Each command below can also take the shared config file so it uses the same parameters as the full launch:

```bash
--ros-args --params-file $(ros2 pkg prefix magpie_control)/share/magpie_control/config/gripper_config.yaml
```

If you omit it, the node still runs on its built-in defaults.

---

## `gripper_node` — MAGPIE gripper (serial)

**Needs:** the gripper plugged in (OpenRB-150 at `/dev/ttyACM0`), and your user in the `dialout` group.

```bash
# auto-detect the serial port
ros2 run magpie_control gripper_node

# or force a port if auto-detect fails
ros2 run magpie_control gripper_node --ros-args \
  -p auto_detect_port:=false -p port:=/dev/ttyACM0
```

**Verify:**
```bash
ros2 topic echo /gripper/state --once          # aperture ~103 mm when open
ros2 service call /gripper/open  std_srvs/srv/Trigger {}
ros2 service call /gripper/close std_srvs/srv/Trigger {}
```

---

## `ur5_node` — UR5 arm (RTDE over Ethernet)

**Needs:** the arm powered on and reachable at `robot_ip` (default `192.168.0.4`). Only one RTDE control client can connect at a time — make sure no other session is holding the arm.

```bash
ros2 run magpie_control ur5_node

# override the robot IP or publish rate
ros2 run magpie_control ur5_node --ros-args \
  -p robot_ip:=192.168.0.4 -p publish_rate:=500
```

**Verify:**
```bash
ros2 topic hz /arm/tcp_pose                     # ~500 Hz
ros2 service call /arm/get_pose  magpie_msgs/srv/GetPose {}
ros2 service call /arm/move_safe std_srvs/srv/Trigger {}
```

---

## `ft_sensor_node` — ATI Mini45 force/torque (UDP)

**Needs:** the ATI NetFT box reachable at `ip_address` (default `192.168.0.6`, UDP 49152). Standalone sensor — not the arm's wrist sensor.

```bash
ros2 run magpie_control ft_sensor_node
```

**Verify:**
```bash
ros2 topic hz   /ft_sensor/wrench               # ~50 Hz
ros2 topic echo /ft_sensor/wrench --once
ros2 service call /ft_sensor/zero std_srvs/srv/Trigger {}   # re-zero (gripper open, unloaded)
```

---

## `tactile_sensor_node` — E-flesh tactile (stub)

**Needs:** nothing. It is a placeholder, disabled by default (`enable: false`). Runs but publishes nothing until an E-flesh driver exists.

```bash
ros2 run magpie_control tactile_sensor_node
```

---

## `deligrasp_node` — detection + grasp orchestration

**Needs the others up first.** This node calls the arm and gripper and reads the camera, so start those before it:

1. **Camera** (separate package, matching namespace):
   ```bash
   ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
   ```
2. **`gripper_node`** and **`ur5_node`** running (see above).
3. Then the orchestrator:
   ```bash
   ros2 run magpie_control deligrasp_node
   ```

**Verify / trigger a grasp:**
```bash
ros2 param set /deligrasp_node object_query "bottle"    # what to grab
ros2 service call /grasp/execute std_srvs/srv/Trigger {}
```

> Recommended startup order: **camera → gripper_node → ur5_node → deligrasp_node.**

---

## Or start everything at once

Once you're past bring-up, the launch file does all five nodes together (camera still launches separately):

```bash
ros2 launch magpie_control gripper_control.launch.py
```

See [`ros_integration_overview.html`](ros_integration_overview.html) for the node graph and the full grasp pipeline.
