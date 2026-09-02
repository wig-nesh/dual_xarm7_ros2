#!/usr/bin/env python3
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Vector3
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    RobotTrajectory,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from xarm_msgs.srv import GetFloat32List, GripperMove, SetInt16

GROUP_NAME = 'xarm7'
PLANNING_FRAME = 'link_base'
END_EFFECTOR_LINK = 'link_tcp'
JOINT_NAMES = [f'joint{index}' for index in range(1, 8)]
VELOCITY_SCALING = 0.2
ACCELERATION_SCALING = 0.2


class SingleArmMotions(Node):

    def __init__(self) -> None:
        super().__init__('single_arm_motions')
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.get_servo_angle_client = self.create_client(GetFloat32List, '/xarm/get_servo_angle')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.gripper_enable_client = self.create_client(SetInt16, '/xarm/set_gripper_enable')
        self.gripper_position_client = self.create_client(GripperMove, '/xarm/set_gripper_position')

    def read_current_angles(self) -> list[float]:
        if not self.get_servo_angle_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('get_servo_angle unavailable, is the driver running?')
        future = self.get_servo_angle_client.call_async(GetFloat32List.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError('get_servo_angle timed out')
        response = future.result()
        if response.ret != 0:
            raise RuntimeError(f'get_servo_angle failed: ret={response.ret}')
        return list(response.datas)

    def build_request(self, goal_constraints: Constraints) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = GROUP_NAME
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = VELOCITY_SCALING
        request.max_acceleration_scaling_factor = ACCELERATION_SCALING
        request.goal_constraints.append(goal_constraints)
        return request

    def plan(self, request: MotionPlanRequest, description: str) -> RobotTrajectory:
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('move_action server unavailable, is the moveit launch running?')
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError(f'{description}: goal submission timed out')
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f'{description}: goal rejected by move_group')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            goal_handle.cancel_goal()
            raise RuntimeError(f'{description}: planning timed out')
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'{description}: planning failed, error code {result.error_code.val}')
        trajectory = result.planned_trajectory
        if len(trajectory.joint_trajectory.points) == 0:
            raise RuntimeError(f'{description}: planner returned an empty trajectory')
        points = trajectory.joint_trajectory.points
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
        self.get_logger().info(
            f'{description}: planned {len(points)} waypoints, {duration:.1f} s, '
            f'end angles {[round(angle, 3) for angle in points[-1].positions]}')
        return trajectory

    def execute(self, trajectory: RobotTrajectory, description: str) -> None:
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('execute_trajectory server unavailable')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError(f'{description}: execution goal submission timed out')
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f'{description}: execution goal rejected')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        if not result_future.done():
            goal_handle.cancel_goal()
            raise RuntimeError(f'{description}: execution timed out, cancelled')
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'{description}: execution failed, error code {result.error_code.val}')
        self.get_logger().info(f'{description}: done')

    def joint_goal_constraints(self, angles_rad: list[float]) -> Constraints:
        if len(angles_rad) != len(JOINT_NAMES):
            raise ValueError('exactly seven joint angles are required')
        constraints = Constraints()
        for joint_name, angle_rad in zip(JOINT_NAMES, angles_rad):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = angle_rad
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints

    def plan_confirm_and_execute_request(self, request: MotionPlanRequest, description: str) -> None:
        while True:
            trajectory = self.plan(request, description)
            answer = input(f'{description}: Enter to execute, r to replan, s to skip: ')
            normalized_answer = answer.strip().lower()
            if normalized_answer == 'r':
                continue
            if normalized_answer == 's':
                self.get_logger().info(f'{description}: skipped')
                return
            self.execute(trajectory, description)
            return

    def move_to_joint_angles(self, angles_rad: list[float], description: str) -> None:
        request = self.build_request(self.joint_goal_constraints(angles_rad))
        self.plan_confirm_and_execute_request(request, description)

    def read_current_eef_pose(self) -> Pose:
        angles = self.read_current_angles()
        request = GetPositionFK.Request()
        request.header.frame_id = PLANNING_FRAME
        request.fk_link_names = [END_EFFECTOR_LINK]
        request.robot_state.joint_state.name = JOINT_NAMES
        request.robot_state.joint_state.position = angles
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('compute_fk unavailable, is move_group running?')
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError('compute_fk timed out')
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'compute_fk failed, error code {response.error_code.val}')
        if len(response.pose_stamped) == 0:
            raise RuntimeError(f'compute_fk returned no pose for {END_EFFECTOR_LINK}')
        return response.pose_stamped[0].pose

    def solve_ik(self, target_pose: Pose, seed_angles: list[float]) -> list[float]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP_NAME
        request.ik_request.pose_stamped.header.frame_id = PLANNING_FRAME
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.ik_link_name = END_EFFECTOR_LINK
        request.ik_request.timeout.sec = 5
        request.ik_request.robot_state.joint_state.name = JOINT_NAMES
        request.ik_request.robot_state.joint_state.position = seed_angles
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('compute_ik unavailable, is move_group running?')
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done():
            raise RuntimeError('compute_ik timed out')
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'compute_ik failed, error code {response.error_code.val}')
        return list(response.solution.joint_state.position[: len(JOINT_NAMES)])

    def move_eef_delta(self, delta: Vector3, description: str) -> None:
        seed_angles = self.read_current_angles()
        current_pose = self.read_current_eef_pose()
        self.get_logger().info(
            f'{description}: current eef position '
            f'{current_pose.position.x:.3f} {current_pose.position.y:.3f} {current_pose.position.z:.3f}')
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + delta.x
        target_pose.position.y = current_pose.position.y + delta.y
        target_pose.position.z = current_pose.position.z + delta.z
        target_pose.orientation = current_pose.orientation
        target_angles = self.solve_ik(target_pose, seed_angles)
        self.get_logger().info(
            f'{description}: ik displacement '
            f'{[round(b - a, 4) for a, b in zip(seed_angles, target_angles)]}')
        self.move_to_joint_angles(target_angles, description)

    def set_gripper(self, position_0_to_850: float) -> None:
        answer = input(f'gripper to {position_0_to_850}: press Enter to execute, s to skip: ')
        if answer.strip().lower() == 's':
            self.get_logger().info('gripper: skipped')
            return
        if not self.gripper_enable_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('gripper enable service unavailable')
        enable_future = self.gripper_enable_client.call_async(self.set_int16_request(1))
        rclpy.spin_until_future_complete(self, enable_future, timeout_sec=5.0)
        if not enable_future.done() or enable_future.result().ret != 0:
            raise RuntimeError('gripper enable failed')
        gripper_request = GripperMove.Request()
        gripper_request.pos = position_0_to_850
        gripper_request.wait = True
        if not self.gripper_position_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('gripper position service unavailable')
        gripper_future = self.gripper_position_client.call_async(gripper_request)
        rclpy.spin_until_future_complete(self, gripper_future, timeout_sec=15.0)
        if not gripper_future.done():
            raise RuntimeError('gripper move timed out')
        response = gripper_future.result()
        if response.ret != 0:
            raise RuntimeError(f'gripper move failed: ret={response.ret}')
        self.get_logger().info(f'gripper at {position_0_to_850}')

    @staticmethod
    def set_int16_request(data: int) -> SetInt16.Request:
        request = SetInt16.Request()
        request.data = data
        return request


def load_home_lifted_positions() -> list[float]:
    poses_path = Path(get_package_share_directory('dual_xarm_bringup')) / 'config' / 'named_poses.yaml'
    named_poses = yaml.safe_load(poses_path.read_text())
    positions = named_poses['home_lifted']['positions_rad']
    if len(positions) != len(JOINT_NAMES):
        raise ValueError(f'home_lifted must define {len(JOINT_NAMES)} joint positions')
    return positions


def main() -> None:
    rclpy.init()
    node = SingleArmMotions()
    try:
        node.move_to_joint_angles(load_home_lifted_positions(), 'joint goal: home_lifted')

        node.move_eef_delta(Vector3(x=0.03), 'eef delta: tcp plus 3 cm along base x')
        node.move_eef_delta(Vector3(x=-0.03), 'eef delta: back to start')

        node.move_eef_delta(Vector3(z=0.03), 'eef delta: tcp plus 3 cm along base z')
        node.move_eef_delta(Vector3(z=-0.03), 'eef delta: back to start')

        node.move_eef_delta(Vector3(y=0.03), 'eef delta: tcp plus 3 cm along base y')
        node.move_eef_delta(Vector3(y=-0.03), 'eef delta: back to start')

        node.set_gripper(400.0)
        node.set_gripper(100.0)
    except Exception as error:
        node.get_logger().error(str(error))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
