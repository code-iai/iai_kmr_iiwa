#!/usr/bin/env python
import rospy
from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from sensor_msgs.msg import JointState


class OmniPoseFollower(object):
    def __init__(self):
        self.server = SimpleActionServer('/whole_body_controller/refills_finger/follow_joint_trajectory',
                                         FollowJointTrajectoryAction,
                                         self.execute_cb,
                                         auto_start=False)
        self.state_pub = rospy.Publisher('refills_finger/state', JointTrajectoryControllerState, queue_size=10)
        self.js_sub = rospy.Subscriber('refills_finger/joint_states', JointState, self.js_cb, queue_size=10)
        self.server.start()

    def js_cb(self, data):
        msg = JointTrajectoryControllerState()
        msg.joint_names = data.name
        self.state_pub.publish(msg)

    def execute_cb(self, data):
        """
        :type data: FollowJointTrajectoryGoal
        :return:
        """
        self.server.set_succeeded()


if __name__ == '__main__':
    try:
        rospy.init_node('refills_finger_fake_traj_server')
        opf = OmniPoseFollower()
        rospy.loginfo('fake refills finger traj server running')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
