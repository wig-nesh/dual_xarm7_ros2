# dual xarm7 control stack

Two UFACTORY xArm7 manipulators with xArm grippers, two RealSense D455
cameras, ROS 2 Humble with MoveIt, in Docker on NixOS. Phases and progress
live in plan.md.

## setup

Requirements: Nix with flakes, direnv, Docker via the rootful daemon
(`sudo ./docker/run.sh`), an X11 or XWayland session. Rootless docker
cannot pass USB devices (cameras), and USB devices plugged in after the
container starts are invisible — plug cameras first, then start.

1. `direnv allow`
2. `./docker/run.sh` builds the image on first run and opens a shell in the
   container. `./docker/run.sh --rebuild` rebuilds after Dockerfile changes.
3. Inside the container, run `init_workspace` once. It installs rosdep
   dependencies and builds workspace/src with colcon. Later containers source
   the existing build automatically.

The container uses host networking, so anything reachable from the host (arm
controllers, cameras) is reachable inside.

## ufactory studio

UfactoryStudio is an Electron 13 application and crashes on the host, because
the NixOS unstable glibc is too new for it. It runs inside the Ubuntu 22.04
container instead, through the `ufactory-studio` command.

`ufactory_studio/` holds the extracted application payload and is gitignored.
To recreate it from the AppImage (kept locally, the official download page
lists only the Windows client):

```
./UfactoryStudio-Linux-1.0.1.AppImage --appimage-extract
rm -rf ufactory_studio && mv squashfs-root ufactory_studio
```

Then start the container and run `ufactory-studio`.

## arm network

The arm controllers have static addresses and run no DHCP server, so the host
needs a manual address on that interface. Arm one is at 192.168.1.243, the
host is 192.168.1.50.

This machine uses a USB ethernet adapter (`enp0s20f0u1u4`) and the profile is
already configured as connection `xarm`. On a fresh machine:

```
nmcli device status                                  # find the ethernet device name
nmcli connection add type ethernet con-name xarm ifname enp0s20f0u1u4 \
  ipv4.method manual ipv4.addresses 192.168.1.50/24
ping 192.168.1.243
```

The profile autoconnects whenever the cable is plugged in. No gateway is set,
so an existing wifi connection keeps the default route. Both arms ship with
the same factory default IP, so give the second arm a distinct address before
connecting both to the same switch.

## quick arm checks

Start the driver (needs the network from the previous section):

```
ros2 launch xarm_api xarm7_driver.launch.py robot_ip:=192.168.1.243 add_gripper:=true \
  extra_robot_api_params_path:=/colcon_ws/install/dual_xarm_bringup/share/dual_xarm_bringup/config/gripper_services.yaml
```

The extra params file enables the gripper services; without it the
`set_gripper_*` services stay hidden and the gripper never enables.

Read state (the NaN velocity and effort are normal, the controller does not
report them):

```
ros2 topic echo /xarm/joint_states --once
ros2 service call /xarm/get_servo_angle xarm_msgs/srv/GetFloat32List
```

Enable before anything moves (id 8 means all axes):

```
ros2 service call /xarm/clean_error xarm_msgs/srv/Call
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```

Gripper, position scale is 0-850, 400 is roughly half open:

```
ros2 service call /xarm/set_gripper_enable xarm_msgs/srv/SetInt16 "{data: 1}"
ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove "{pos: 400, wait: true}"
```

First joint, absolute angles in radians for all seven joints (the `relative:
true` mode returns error 997 on firmware v2.8.0, use absolute angles):

```
ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint \
  "{angles: [0.17, 0.50, 0.00, 0.48, 1.66, -1.54, 0.00], speed: 0.3, acc: 0.3, mvtime: 0.0, wait: true, timeout: 10.0, radius: -1.0, relative: false}"
```

`ret=0` means success, 9 means not enabled, 2 means a controller error that
`/xarm/clean_error` usually clears.

## moveit + rviz

Start the driver (as above) and MoveIt in separate terminals. MoveIt launches
the planning stack and RViz for the first arm:

```
ros2 launch xarm_moveit_config _robot_moveit_realmove.launch.py \
  robot_ip:=192.168.1.243 dof:=7 robot_type:=xarm add_gripper:=true
```

The IK solver is KDL (`config/xarm7/kinematics.yaml`). Confirm the running
instance loaded it:

```
ros2 param get /move_group robot_description_kinematics.xarm7.kinematics_solver
# -> kdl_kinematics_plugin/KDLKinematicsPlugin
```

The simple motion test moves the end effector by small deltas along each base
axis using exact IK to joint goals (FK current pose, add delta, `/compute_ik`
from the current joint state, plan in joint space). Each move waits for Enter,
`r` replans, `s` skips:

```
ros2 run dual_xarm_bringup single_arm_motions.py
```

It starts from the `home_lifted` named pose, then shifts the TCP +3 cm / -3 cm
along base x, z, and y, then opens/closes the gripper. Keep MoveIt/RViz up to
watch the planned paths.

## camera + hand-eye calibration

Eye-on-base calibration of one D455 against arm one, via easy_handeye2. All
parameters live in `workspace/src/dual_xarm_bringup/config/handeye_calibration.yaml`.

### board

Print a 5x5 charuco board:

```
ros2 run dual_xarm_bringup make_charuco_board.py --output /colcon_ws/charuco_board.png
```

Print at the stated dpi, then **measure the actual square and marker with
calipers** and set `board.square_length_m` / `board.marker_length_m` to the
measured values. A board size error becomes a scale error in the calibration
directly: 1 percent wrong board = 1 cm wrong at 1 m.

### calibrate

Rigidly mount the board on the gripper, face outward. Start the usual stack
(driver, MoveIt realmove, enable services, camera), then:

```
ros2 launch dual_xarm_bringup camera_calibrate.launch.py
```

This starts the charuco detector and the easy_handeye2 server (no GUI; on
Humble the rqt calibrator is broken, everything goes through services).

Jog the arm so the board is centered, face-on, about 0.5 m from the camera —
that pose is the nominal all sample orientations rotate around. Then:

```
ros2 run dual_xarm_bringup calibrate_camera.py
```

It walks through orientation candidates (Enter executes, `r` replans, `s`
skips a pose), takes 12 samples, solves, and copies the result to
`/colcon_ws/calibrations/<name>.yaml` (which persists across containers).

Sanity checks:

- The printed camera translation should match a tape measure from the arm base.
- Cross-check solvers; agreement within ~1 mm / under a degree means clean data:

```
ros2 service call /easy_handeye2/calibration/set_algorithm easy_handeye2_msgs/srv/SetAlgorithm "{new_algorithm: 'OpenCV/Park'}"
ros2 service call /easy_handeye2/calibration/compute_calibration easy_handeye2_msgs/srv/ComputeCalibration
```

If the board was measured after calibrating and differs from the configured
size, correct the saved yaml instead of redoing the sampling: multiply the
translation by `measured_length / configured_length`.

### publish and use

```
ros2 launch dual_xarm_bringup publish_calibration.launch.py
ros2 run tf2_ros tf2_echo link_base camera_link
```

This publishes a static `link_base -> camera_link` transform (composed with
the driver's camera tree, so the camera frame stays single-parented). Anything
the camera detects is now expressible in arm coordinates.

### table marker and hover demo

The `table_marker:` config section drives a second detector instance:

```
ros2 launch dual_xarm_bringup table_marker.launch.py
ros2 run dual_xarm_bringup marker_hover_demo.py
```

`marker_type: aruco` tracks a single marker (dictionary, id, size); the demo
hovers the tool `hover_offset_m` above it, `hover_orientation: down`. For
marker boards use `marker_type: grid` with the physical layout
(`markers_x/y`, lengths, `grid_id_order`). Unknown dictionary or layout?
Probe a live frame:

```
python3 /colcon_ws/src/dual_xarm_bringup/scripts/probe_dictionary.py
python3 /colcon_ws/src/dual_xarm_bringup/scripts/probe_grid_layout.py
```

Expected accuracy with a calibrated setup: 1-2 cm on a single marker,
sub-centimeter on a multi-marker board.

Note: `handeye.robot_effector_frame` must match the running robot model —
`link_tcp` with `add_gripper:=true`, `link_eef` on a bare flange.

