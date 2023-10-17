import moveit_commander
import rospy
import tf2_ros
import sys
import numpy as np

# from robothon_utils.tf_utils import tmsg_to_array
# from robothon_utils.tf_utils import tarray_to_posemsg

from moveit_msgs.msg import Constraints
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.msg import JointConstraint
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

from geometry_msgs.msg import TransformStamped, PoseStamped

from shape_msgs.msg import SolidPrimitive

from tf import transformations as tfs

from collections import deque

# from tfs import translation_matrix
from ndx_util.tf_util import (
    tmsg_to_array,
    tarray_to_posemsg,
    get_pose,
    get_transform,
    vector3_msg_to_numpy,
    tarray_to_tmsg,
)

import moveit_msgs
from geometry_msgs.msg import WrenchStamped, Vector3
from controller_manager_msgs.srv import SwitchController


class NdxArm:
    def __init__(
        self,
        move_group_name,
        tool_frame=None,
        planning_frame=None,
        velocity_scale=0.02,
        acc_scale=0.02,
        tf_buffer=None,
        planner="RRTConnect",
        planning_time=5,
        force_sensor_topic=None,
    ):

        # Moveit config
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(move_group_name)

        # Moveit frames
        self.mvit_planning_frame = self.move_group.get_planning_frame()
        self.mvit_tool_link = self.move_group.get_end_effector_link()
        # self.touch_links = self.robot.get_link_names(group="hand")
        rospy.logwarn("Planning frame: {}".format(self.mvit_planning_frame))
        rospy.logwarn("EE link: {}".format(self.mvit_tool_link))
        # Wait for everything to initialize
        rospy.sleep(1.0)

        # Initialise tf listener
        if tf_buffer is None:
            self.tf_buffer = tf2_ros.Buffer()
        else:
            self.tf_buffer = tf_buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialise tf broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Wait for everything to initialize
        rospy.sleep(1.0)

        # Set moveit parameters
        self.move_group.set_max_velocity_scaling_factor(velocity_scale)
        self.move_group.set_max_acceleration_scaling_factor(acc_scale)
        # self.move_group.set_planning_pipeline_id("ompl")
        self.move_group.set_planner_id(planner)
        # self.move_group.set_planner_id("RRTstar")
        # self.move_group.set_planner_id("PRMstar")
        self.move_group.allow_replanning(True)
        self.move_group.set_planning_time(planning_time)
        # self.move_group.set_num_planning_attempts(5)

        # Wait for everything to initialize
        rospy.sleep(1.0)

        if planning_frame is None:
            planning_frame = self.mvit_planning_frame
        if tool_frame is None:
            tool_frame = self.mvit_tool_link

        # Setting the offset to proper calculate transforms to moveit's ee frame
        self.set_tool_frame(tool_frame)

        # Setting the reference frame for target poses:
        self.set_planning_frame(planning_frame)

        self.curr_force = None
        self.curr_torque = None
        self.last_curr_force = deque(maxlen=10)
        self.last_curr_torque = deque(maxlen=10)
        if force_sensor_topic is not None:
            rospy.Subscriber(
                force_sensor_topic, WrenchStamped, self.force_callback
            )
            self.switch_controllers = rospy.ServiceProxy(
                "/ur5e_A/controller_manager/switch_controller",
                SwitchController,
            )
            # self.compliance_target_pub = tf2_ros.TransformBroadcaster()
            self.compliance_target_pub = rospy.Publisher(
                "/ur5e_A/motion_control_handle/ur5e_A/target_frame",
                PoseStamped,
                queue_size=100,
            )

    def force_callback(self, data: WrenchStamped) -> None:
        self.last_curr_force.append(vector3_msg_to_numpy(data.wrench.force))
        self.last_curr_torque.append(vector3_msg_to_numpy(data.wrench.torque))
        self.curr_force = np.mean(self.last_curr_force, axis=0)
        self.curr_torque = np.mean(self.last_curr_torque, axis=0)

    def set_tool_frame(self, frame_name: str):
        """Sets the frame that will be used as the tool frame. When calling a move operation,
        all functions will atempt to set 'frame_name' to the target pose.

        Args:
            frame_name (str): Name of the frame to set as the tool frame
        """
        self.tool_frame = frame_name
        if frame_name != None:
            self.tool_frame_offset, _, _, _ = get_transform(
                self.tf_buffer, self.mvit_tool_link, frame_name, rospy.Time(0)
            )
        else:
            self.tool_frame_offset = np.eye(4)

    def set_planning_frame(self, frame_name: str, max_wait_time: float = 10):
        """Sets the frame of reference for all moving operations.

        Args:
            frame_name (str): Name of the frame to be used as the planning frame.
        """
        self.planning_frame = frame_name
        if frame_name != None:
            new_frame, _, _, _ = get_transform(
                self.tf_buffer,
                frame_name,
                self.mvit_planning_frame,
                # time=rospy.Time(0),
                max_wait_time=max_wait_time,
            )
            if new_frame is not None:
                self.T_pframe_mvitpframe = new_frame
            else:
                return False
        else:
            self.T_pframe_mvitpframe = np.eye(4)
        return True

    def prepare_pose(
        self,
        target_pose: np.ndarray,
        pose_frame: str = None,
        T_posef_pframe: np.ndarray = None,
    ) -> np.ndarray:
        """
        Prepares pose for moveit planner
        """
        # If pose_frame is None we consider the pose to be in the same ref frame as the planning frame.
        if pose_frame is None:
            pose_frame = self.planning_frame

        # If pose_frame is not the planning frame, we need to transform it. If the transform is not given, we need to get it.
        if pose_frame != self.mvit_planning_frame:
            if T_posef_pframe is None:
                T_posef_pframe, _, _, _ = get_transform(
                    self.tf_buffer,
                    pose_frame,
                    self.planning_frame,
                    rospy.Time(0),
                )

                if T_posef_pframe is None:
                    rospy.logerr(
                        "Could not find a transform between the pose frame"
                        " '{}' to the planning frame '{}'. Aborting.".format(
                            pose_frame, self.mvit_planning_frame
                        )
                    )
                    return None
            T_pose_mvitpframe = np.matmul(
                T_posef_pframe, self.T_pframe_mvitpframe
            )
            target_pose = np.matmul(T_pose_mvitpframe, target_pose)

        # Apply the ee_link -> moveit's ee_link offset:
        target_pose = np.matmul(target_pose, self.tool_frame_offset)
        return target_pose

    def get_tool_pose(self, reference_frame=None, max_wait_time=5):
        if reference_frame is None:
            # Return in planning frame
            reference_frame = self.planning_frame
        return get_pose(
            self.tf_buffer,
            reference_frame,
            self.tool_frame,
            rospy.Time(0),
            max_wait_time=5,
        )

    def set_tool_pose(
        self,
        target_pose: np.ndarray,
        pose_frame: str = None,
        T_posef_pframe: np.ndarray = None,
        position_constraints: moveit_msgs.msg.Constraints = None,
        rotation_constraints: moveit_msgs.msg.Constraints = None,
    ):

        target_pose = self.prepare_pose(
            target_pose, pose_frame, T_posef_pframe
        )
        target_pose_msg = tarray_to_posemsg(
            target_pose, self.mvit_planning_frame
        )

        if (
            position_constraints is not None
            or rotation_constraints is not None
        ):
            constraint = Constraints()
            constraint.name = "movementconstraint"
        if position_constraints is not None:
            p_constraints = self.create_position_constraints(
                target_pose_msg.pose
            )
            constraint.position_constraints.append(p_constraints)
        if rotation_constraints is not None:
            r_constraints = self.create_rotation_constraints(
                target_pose_msg.pose
            )
            constraint.orientation_constraints.append(r_constraints)
        if (
            position_constraints is not None
            or rotation_constraints is not None
        ):
            self.move_group.set_path_constraints(constraint)

        # trajectory = self.move_group.plan(target_pose_msg.pose)
        # success = True
        (
            success,
            trajectory,
            planning_time,
            error_code,
        ) = self.move_group.plan(target_pose_msg.pose)
        # rospy.logwarn("planning_time: {}, error_code: {}".format(planning_time, error_code))
        if not success:
            rospy.logerr("Failed to generate plan. Aborting")
            if (
                position_constraints is not None
                or rotation_constraints is not None
            ):
                self.move_group.clear_path_constraints()
            return False

        # path_constr = self.move_group.get_path_constraints()
        # if path_constr is not None:
        #     print(f"Contraints: {path_constr}")
        move_success = self.move_group.execute(trajectory, wait=True)
        if (
            position_constraints is not None
            or rotation_constraints is not None
        ):
            self.move_group.clear_path_constraints()
        if not move_success:
            rospy.logerr("Failed to move robot to target pose.")
            return False
        return True

    def move_until(
        self,
        direction,
        step_size,
        force_limit=None,
        torque_limit=None,
        interval=0.05,
    ):
        self.switch_controllers(
            start_controllers=[
                "cartesian_compliance_controller",
            ],
            stop_controllers=[
                "pos_joint_traj_controller",
            ],
            strictness=2,
            start_asap=True,
        )
        rospy.sleep(1)

        initial_pos, _, _, _, = get_pose(
            tf_buffer=self.tf_buffer,
            reference_frame="ur5e_A/base_link",
            frame="ur5e_A_fingertip",
            time=rospy.Time(0),
            max_wait_time=5,
        )
        curr_dir_index = 0
        while not rospy.is_shutdown():
            if self.curr_force is None:
                rospy.sleep(0.01)
                continue
            # Limit reached in at least one axis, so we stop
            f_lim_reached = force_limit is not None and np.any(
                np.abs(self.curr_force) >= force_limit
            )
            t_lim_reached = torque_limit is not None and np.any(
                np.abs(self.curr_torque) >= torque_limit
            )
            if f_lim_reached or t_lim_reached:
                break
            if isinstance(direction, list):
                curr_direction = direction[curr_dir_index]
                curr_dir_index += 1
                if curr_dir_index >= len(direction):
                    curr_dir_index = 0
            else:
                curr_direction = direction
            initial_pos[:3, 3] += step_size * curr_direction

            # self.compliance_target_pub.sendTransform(
            #     tarray_to_tmsg(initial_pos, "ur5e_A/base_link", "next_pose")
            # )
            self.compliance_target_pub.publish(
                tarray_to_posemsg(initial_pos, "ur5e_A/base_link")
            )
            rospy.sleep(interval)

        self.switch_controllers(
            start_controllers=["pos_joint_traj_controller"],
            stop_controllers=["cartesian_compliance_controller"],
            strictness=2,
            start_asap=True,
        )
        rospy.sleep(1)

    def create_rotation_constraints(self, pose_msg):
        """
        Returns a moveit Constraints message, where the x and y axes are constrained, as well as the end effector orientation.
        Args:
            goal_transform: One of the picking transforms (grasping or pregrasping pose).

        Returns:
            moveit_msgs.msg.Constraints
        """
        # Create constraints msg
        # constraints = Constraints()
        # constraints.name = "picking constraint"

        # Create orientation constraint
        rot_constraint = OrientationConstraint()
        rot_constraint.header.frame_id = self.mvit_planning_frame
        rot_constraint.link_name = self.mvit_tool_link
        rot_constraint.orientation = pose_msg.orientation
        print(
            "Orientation of rotation contraint: {}".format(
                pose_msg.orientation
            )
        )
        # rot_constraint.orientation = Quaternion(pose_msg.orientation.x, pose_msg.orientation.y,
        #                                         pose_msg.orientation.z, pose_msg.orientation.w)
        rot_constraint.absolute_x_axis_tolerance = np.deg2rad(120)
        rot_constraint.absolute_y_axis_tolerance = np.deg2rad(360)
        rot_constraint.absolute_z_axis_tolerance = np.deg2rad(360)
        rot_constraint.weight = 2
        # constraints.orientation_constraints = [rot_constraint]

        return rot_constraint
        # return constraints

    def create_position_constraints(self, pose_msg):
        """
        Returns a moveit Constraints message, where the x and y axes are constrained, as well as the end effector orientation.
        Args:
            goal_transform: One of the picking transforms (grasping or pregrasping pose).

        Returns:
            moveit_msgs.msg.Constraints
        """
        # Create constraints msg
        # constraints = Constraints()
        # constraints.name = "position constraint"

        # Create orientation constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.mvit_planning_frame
        pos_constraint.link_name = self.mvit_tool_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        bouding_region = SolidPrimitive()
        bouding_region.type = 1  # SolidPrimitive.BOX
        bouding_region.dimensions = [0.005, 0.005, 0.005]
        pos_constraint.constraint_region.primitives.append(bouding_region)
        pos_constraint.constraint_region.primitive_poses.append(pose_msg)
        pos_constraint.weight = 1
        # constraints.position_constraints = [pos_constraint]

        return pos_constraint

    def create_joint_constraint(
        self, joint_name, joint_index, absolute_tolerance
    ):
        """
        Returns a moveit Constraints message, where the x and y axes are constrained, as well as the end effector orientation.
        Args:
            goal_transform: One of the picking transforms (grasping or pregrasping pose).

        Returns:
            moveit_msgs.msg.Constraints
        """
        # Create constraints msg
        # constraints = Constraints()
        # constraints.name = "position constraint
        jv = self.move_group.get_current_joint_values()

        # Create orientation constraint
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = jv[joint_index]
        joint_constraint.tolerance_above = absolute_tolerance
        joint_constraint.tolerance_below = absolute_tolerance

        return joint_constraint
