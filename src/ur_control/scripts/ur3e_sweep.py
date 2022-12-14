#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates

import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class ur_control(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, name = 'manipulator'):
        super(ur_control, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur_planner", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.name = name
        group_name = self.name
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # ros message
        self.sub_vector = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callbackVector)
        self.rpy = ModelStates()

    def callbackVector(self, msg):
        self.rpy = msg

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 1.57
        joint_goal[1] = -1.57
        joint_goal[2] = 1.30
        joint_goal[3] = -1.57
        joint_goal[4] = -1.57
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        while self.rpy.pose == []:
            pass
        current_pose = self.move_group.get_current_pose().pose
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position.x = self.rpy.pose[5].position.x
        # pose_goal.position.y = self.rpy.pose[5].position.y
        # pose_goal.position.z = self.rpy.pose[5].position.z
        pose_goal.position.x = current_pose.position.x
        pose_goal.position.y = current_pose.position.y
        pose_goal.position.z = current_pose.position.z
        # pose_goal.orientation.x = current_pose.orientation.x 
        # pose_goal.orientation.y = current_pose.orientation.y
        # pose_goal.orientation.z = current_pose.orientation.z
        # pose_goal.orientation.w = current_pose.orientation.w
        pose_goal.orientation.x = -0.978
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0.151
        print(pose_goal)
        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=0.1):
        while self.rpy.pose == []:
            pass
        move_group = self.move_group
        r = 5
        waypoints = []
        joint_goal = move_group.get_current_joint_values()
        wpose = move_group.get_current_pose().pose
        dx = self.rpy.pose[5].position.x - (wpose.position.x + self.rpy.pose[6].position.x)
        dy = self.rpy.pose[5].position.y - (wpose.position.y + self.rpy.pose[6].position.y)
        dz = self.rpy.pose[5].position.z - (wpose.position.z + self.rpy.pose[6].position.z - 0.15)
        print(dx,dy,dz)
        for i in range(r):
            wpose.position.x = wpose.position.x + dx/r
            wpose.position.y = wpose.position.y + dy/r
            wpose.position.z = wpose.position.z + dz/r
            waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )
        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def print_rpy(self):
        #time.sleep(0.1)
        #print("rpy= ", self.rpy.pose[4:7])

        while self.rpy.pose == []:
            pass
        print("rpy= ", self.rpy.pose[5].position.z)

    def current_pos(self):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        print("pose= ", wpose)


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        tutorial = ur_control("manipulator")
        tutorial.print_rpy()
        tutorial.current_pos()
        #tutorial.go_to_pose_goal()
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        tutorial.execute_plan(cartesian_plan)
        tutorial.current_pos()




        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()