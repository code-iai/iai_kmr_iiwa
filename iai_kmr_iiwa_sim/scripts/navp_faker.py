#!/usr/bin/env python

import numpy as np
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseGoal
from tf.transformations import quaternion_matrix, rotation_from_matrix
from tf2_geometry_msgs import do_transform_pose
from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class TfWrapper(object):
    def __init__(self, buffer_size=2):
        self.tfBuffer = Buffer(rospy.Duration(buffer_size))
        self.tf_listener = TransformListener(self.tfBuffer)
        rospy.sleep(0.1)

    def transformPose(self, target_frame, pose):
        try:
            transform = self.tfBuffer.lookup_transform(target_frame,
                                                       pose.header.frame_id,  # source frame
                                                       pose.header.stamp,
                                                       rospy.Duration(1.0))
            new_pose = do_transform_pose(pose, transform)
            return new_pose
        except ExtrapolationException as e:
            rospy.logwarn(e)


class NavpFaker(object):
    def __init__(self):
        name = 'nav_pcontroller/move_base'
        self.max_vel = 0.5
        self.freq = 0.2
        self.tf = TfWrapper()
        self.client = actionlib.SimpleActionClient('whole_body_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction)
        self.client.wait_for_server()

        self.server = actionlib.SimpleActionServer(name, MoveBaseAction,
                                                   execute_cb=self.cb, auto_start=False)
        self.server.register_preempt_callback(self.preempt_cb)
        self.simple_goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.simple_goal_cb)
        self.server.start()
        self.self_client = actionlib.SimpleActionClient(name, MoveBaseAction)

    def simple_goal_cb(self, goal_pose):
        g = MoveBaseGoal()
        g.target_pose = goal_pose
        self.self_client.send_goal(g)

    def preempt_cb(self):
        self.client.cancel_goal()

    def cb(self, move_base_goal):
        move_base_goal = self.tf.transformPose('map', move_base_goal.target_pose)
        goal_js = [move_base_goal.pose.position.x,
                   move_base_goal.pose.position.y,
                   rotation_from_matrix(quaternion_matrix([move_base_goal.pose.orientation.x,
                                                           move_base_goal.pose.orientation.y,
                                                           move_base_goal.pose.orientation.z,
                                                           move_base_goal.pose.orientation.w]))[0]]

        current_js = rospy.wait_for_message('whole_body_controller/state', JointTrajectoryControllerState)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header = move_base_goal.header
        goal.trajectory.joint_names = current_js.joint_names

        p = JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(0.1)
        p.positions = current_js.actual.positions
        goal.trajectory.points.append(p)
        last_p = p

        def next_p(goal_p, last_p, f):
            diff = (goal_p - last_p)
            if diff > 0:
                diff = min(diff, self.max_vel * f)
            else:
                diff = max(diff, -self.max_vel * f)
            return last_p + (diff)

        while not np.allclose(np.array(goal_js), p.positions[:3]):
            p = JointTrajectoryPoint()
            p.time_from_start = last_p.time_from_start + rospy.Duration(1 * self.freq)
            p.positions = list(current_js.actual.positions)

            p.positions[0] = next_p(goal_js[0], last_p.positions[0], self.freq)
            p.positions[1] = next_p(goal_js[1], last_p.positions[1], self.freq)
            p.positions[2] = next_p(goal_js[2], last_p.positions[2], self.freq)

            goal.trajectory.points.append(p)
            last_p = p

        self.client.send_goal(goal)
        while not self.client.wait_for_result(rospy.Duration(.1)):
            if self.server.is_preempt_requested():
                rospy.loginfo('new goal, cancel old one')
                self.client.cancel_all_goals()
                self.server.set_preempted(MoveBaseResult())
                break
        # result = self.client.get_result()
        # if result.error_code == result.SUCCESSFUL:
        self.server.set_succeeded(MoveBaseResult())
        # else:
        #     self.server.set_preempted(MoveBaseResult())


if __name__ == '__main__':
    rospy.init_node('nav_p')
    muh = NavpFaker()
    rospy.loginfo('navp faker running.....')
    rospy.spin()
