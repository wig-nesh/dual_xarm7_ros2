# dual xarm7 control stack plan

Two UFACTORY xArm7 manipulators with xArm grippers, two RealSense D455 cameras,
ROS 2 Humble with ros2_control and MoveIt, running in Docker on NixOS.
Calibration with easy_handeye2 (eye-on-base) and ArUco markers. Arm one is at
192.168.1.243, the second arm address is pending.

Upstream sources, cloned into the colcon workspace:

- xarm_ros2, humble branch, at workspace/src/xarm_ros2 (xarm_gazebo and
  thirdparty/realsense_gazebo_plugin carry COLCON_IGNORE, real hardware only)
- easy_handeye2, at workspace/src/easy_handeye2

## phase 0: docker environment

- [x] Dockerfile: Ubuntu 22.04, ROS 2 Humble desktop, colcon, python deps
- [x] flake.nix and .envrc (direnv) for the host shell; the container receives
      the NVIDIA driver through bind mounts, no nvidia container toolkit
- [x] run script: X11 passthrough (RViz), host network (xArm SDK is TCP),
      USB passthrough for the D455s, workspace bind mount
- [x] move the cloned repos into the colcon workspace
- [x] build the workspace inside the container, verify xarm packages are listed
- [x] UfactoryStudio GUI runs inside the container (its Electron 13 build
      cannot run against the host glibc)

## phase 1: one arm plus moveit

- [x] launch one xarm7 with gripper (xarm7_driver.launch.py), verify
      xarm_api: enable, state, gripper open/close (gripper commands need the
      enable service from dual_xarm_bringup/config/gripper_services.yaml)
- [x] plan and execute from the RViz motion planning panel
- [ ] python script: joint goal, Cartesian pose goal, gripper commands
- [ ] add the table to the planning scene so plans clear it

## phase 2: two arms

- [ ] measure a rough base-to-base offset, publish it as a static transform (placeholder)
- [ ] dual xarm7 MoveIt config (dual_xarm7_moveit_realmove.launch.py), both
      arms in one planning scene
- [ ] coordinated planning for the combined group, plus per-arm scripts
- [ ] reduced velocity limits while both arms share the workspace

## phase 3: cameras and calibration

- [ ] realsense-ros for both D455s, distinct serials, depth aligned to color
- [ ] print ArUco markers: one per end effector, one fixed on the table
- [ ] easy_handeye2 eye-on-base: each camera to its nearest arm base
- [ ] table frame from the table marker
- [ ] refine the base-to-base transform using the calibrated camera TFs
- [ ] merge both point clouds through the calibrated TFs, publish one combined cloud
- [ ] node that saves the combined cloud to a PCD file

## phase 4: server integration (deferred, to discuss)

- [ ] client that sends the combined PCD to the server
- [ ] parse the two returned arm configurations, check joint limits
- [ ] plan and execute them with MoveIt
- [ ] collision checking against the merged cloud
