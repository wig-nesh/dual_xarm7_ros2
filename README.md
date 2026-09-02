# dual xarm7 control stack

Two UFACTORY xArm7 manipulators with xArm grippers, two RealSense D455
cameras, ROS 2 Humble with MoveIt, in Docker on NixOS. Phases and progress
live in plan.md.

## setup

Requirements: Nix with flakes, direnv, Docker (rootless works), an X11 or
XWayland session.

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

