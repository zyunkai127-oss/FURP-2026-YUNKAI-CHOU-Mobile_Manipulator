#!/usr/bin/env python3
"""Manual eye-to-hand detection -> TF -> IK -> MoveIt grasp sequence."""

from copy import deepcopy
from threading import Thread
from time import sleep

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import panda
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped with tf2_ros
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


# URDF limits for the seven non-continuous Panda arm joints (radians).
PANDA_LIMITS = {
    "panda_joint1": (-2.8973, 2.8973),
    "panda_joint2": (-1.7628, 1.7628),
    "panda_joint3": (-2.8973, 2.8973),
    "panda_joint4": (-3.0718, -0.0698),
    "panda_joint5": (-2.8973, 2.8973),
    "panda_joint6": (-0.0175, 3.7525),
    "panda_joint7": (-2.8973, 2.8973),
}


class EyeToHandGraspDemo(Node):
    def __init__(self):
        super().__init__("eye_to_hand_grasp_demo")
        self.cb_group = ReentrantCallbackGroup()

        self._declare_parameters()
        self.camera_frame = self.get_parameter("camera_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.arm_group = self.get_parameter("arm_group").value
        self.gripper_group = self.get_parameter("gripper_group").value
        self.eef_link = self.get_parameter("end_effector_link").value
        self.execute = self.get_parameter("execute").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.cb_group
        )
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_pub = self.create_publisher(Marker, "object_marker", marker_qos)

        self.arm = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=self.base_frame,
            end_effector_name=self.eef_link,
            group_name=self.arm_group,
            callback_group=self.cb_group,
        )
        self.arm.max_velocity = self.get_parameter("max_velocity").value
        self.arm.max_acceleration = self.get_parameter("max_acceleration").value
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=self.gripper_group,
            callback_group=self.cb_group,
        )

    def _declare_parameters(self):
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", panda.base_link_name())
        self.declare_parameter("arm_group", panda.MOVE_GROUP_ARM)
        self.declare_parameter("gripper_group", panda.MOVE_GROUP_GRIPPER)
        self.declare_parameter("end_effector_link", panda.end_effector_name())
        self.declare_parameter("object_position_camera", [0.0, 0.0, 1.084481443])
        self.declare_parameter("object_size", [0.05, 0.05, 0.18])
        self.declare_parameter("grasp_z_offset", 0.0)
        self.declare_parameter("approach_distance", 0.15)
        self.declare_parameter("lift_distance", 0.20)
        self.declare_parameter("grasp_quaternion_xyzw", [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter(
            "home_joint_positions",
            [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398],
        )
        self.declare_parameter("max_velocity", 0.25)
        self.declare_parameter("max_acceleration", 0.25)
        self.declare_parameter("execute", True)

    def detected_object_pose(self):
        xyz = self.get_parameter("object_position_camera").value
        pose = PoseStamped()
        pose.header.frame_id = self.camera_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        pose.pose.orientation.w = 1.0
        return pose

    def transform_to_base(self, camera_pose):
        try:
            base_pose = self.tf_buffer.transform(
                camera_pose,
                self.base_frame,
                timeout=Duration(seconds=10.0),
            )
        except TransformException as error:
            raise RuntimeError(
                f"Cannot transform {camera_pose.header.frame_id} -> "
                f"{self.base_frame}: {error}"
            ) from error

        p = base_pose.pose.position
        self.get_logger().info(
            "Object transformed to base frame: "
            f"x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f} m"
        )
        return base_pose

    def make_grasp_poses(self, object_pose):
        q = self.get_parameter("grasp_quaternion_xyzw").value
        grasp = deepcopy(object_pose)
        grasp.header.frame_id = self.base_frame
        grasp.pose.position.z += self.get_parameter("grasp_z_offset").value
        (
            grasp.pose.orientation.x,
            grasp.pose.orientation.y,
            grasp.pose.orientation.z,
            grasp.pose.orientation.w,
        ) = q

        approach = deepcopy(grasp)
        approach.pose.position.z += self.get_parameter("approach_distance").value
        lift = deepcopy(grasp)
        lift.pose.position.z += self.get_parameter("lift_distance").value
        return approach, grasp, lift

    def compute_ik(self, target_pose):
        if not self.ik_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("MoveIt /compute_ik service is not available")

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.arm_group
        request.ik_request.ik_link_name = self.eef_link
        request.ik_request.pose_stamped = target_pose
        request.ik_request.timeout = Duration(seconds=5.0).to_msg()
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.is_diff = True

        future = self.ik_client.call_async(request)
        while rclpy.ok() and not future.done():
            sleep(0.05)
        response = future.result()
        if response is None or response.error_code.val != response.error_code.SUCCESS:
            code = response.error_code.val if response else "no response"
            raise RuntimeError(f"No collision-free IK solution (MoveIt code {code})")

        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position
        solution = dict(zip(names, positions))
        violations = []
        for name, (lower, upper) in PANDA_LIMITS.items():
            value = solution.get(name)
            if value is None or not lower <= value <= upper:
                violations.append(f"{name}={value} not in [{lower}, {upper}]")
        if violations:
            raise RuntimeError("IK joint-limit validation failed: " + "; ".join(violations))

        joint_text = ", ".join(
            f"{name}={solution[name]:.3f}" for name in PANDA_LIMITS
        )
        self.get_logger().info(f"IK valid and within joint limits: {joint_text}")
        return solution

    def publish_object_marker(self, object_pose):
        size = self.get_parameter("object_size").value
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = object_pose.pose
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.15,
            0.55,
            0.95,
            0.9,
        )
        self.marker_pub.publish(marker)

    def move_joints(self, label, positions):
        self.get_logger().info(label)
        self.arm.move_to_configuration(positions)
        result = self.arm.wait_until_executed()
        if result is False:
            raise RuntimeError(f"MoveIt failed during: {label}")

    def move_pose(self, label, target, cartesian=False):
        self.get_logger().info(label)
        p, q = target.pose.position, target.pose.orientation
        self.arm.move_to_pose(
            position=[p.x, p.y, p.z],
            quat_xyzw=[q.x, q.y, q.z, q.w],
            cartesian=cartesian,
            frame_id=target.header.frame_id,
        )
        result = self.arm.wait_until_executed()
        if result is False:
            raise RuntimeError(f"MoveIt failed during: {label}")

    def run(self):
        object_camera = self.detected_object_pose()
        object_base = self.transform_to_base(object_camera)
        self.publish_object_marker(object_base)
        approach, grasp, lift = self.make_grasp_poses(object_base)

        # The floor is a real MoveIt collision object. Its top is just below z=0.
        self.arm.add_collision_box(
            id="floor",
            size=[2.0, 2.0, 0.02],
            position=[0.0, 0.0, -0.011],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id=self.base_frame,
        )
        sleep(1.0)

        # An explicit service call proves IK feasibility before planning/execution.
        self.compute_ik(grasp)
        if not self.execute:
            self.get_logger().info("IK-only run complete (execute:=false)")
            return

        home = self.get_parameter("home_joint_positions").value
        self.move_joints("1/6 Move to home", home)
        self.get_logger().info("2/6 Open gripper")
        self.gripper.open()
        self.gripper.wait_until_executed()
        self.move_pose("3/6 Move to approach pose", approach)
        self.move_pose("4/6 Descend to grasp pose", grasp, cartesian=True)
        self.get_logger().info("5/6 Close gripper and attach object")
        self.gripper.close()
        self.gripper.wait_until_executed()

        size = self.get_parameter("object_size").value
        p = object_base.pose.position
        self.arm.add_collision_box(
            id="target_object",
            size=size,
            position=[p.x, p.y, p.z],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id=self.base_frame,
        )
        sleep(1.0)
        self.arm.attach_collision_object(
            id="target_object",
            link_name=self.eef_link,
            touch_links=["panda_hand", "panda_leftfinger", "panda_rightfinger"],
        )
        sleep(1.0)
        self.move_pose("6/6 Lift object", lift, cartesian=True)
        self.get_logger().info("Grasp sequence completed successfully")


def main(args=None):
    rclpy.init(args=args)
    node = EyeToHandGraspDemo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Give TF and MoveIt action/service discovery a moment to settle.
        sleep(2.0)
        node.run()
    except Exception as error:  # Keep a single clear terminal failure message.
        node.get_logger().error(str(error))
        raise
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
