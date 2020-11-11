#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_vector3
from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener

tfBuffer = None # type: Buffer
tf_listener = None


def init(tf_buffer_size=15):
    """
    If you want to specify the buffer size, call this function manually, otherwise don't worry about it.
    :param tf_buffer_size: in secs
    :type tf_buffer_size: int
    """
    global tfBuffer, tf_listener
    tfBuffer = Buffer(rospy.Duration(tf_buffer_size))
    tf_listener = TransformListener(tfBuffer)
    rospy.sleep(5.0)

def transform_vector(target_frame, vector):
    """
    Transforms a pose stamped into a different target frame.
    :type target_frame: str
    :type vector: Vector3Stamped
    :return: Transformed pose of None on loop failure
    :rtype: Vector3Stamped
    """
    global tfBuffer
    if tfBuffer is None:
        init()
    try:
        transform = tfBuffer.lookup_transform(target_frame,
                                              vector.header.frame_id,  # source frame
                                              vector.header.stamp,
                                              rospy.Duration(5.0))
        new_pose = do_transform_vector3(vector, transform)
        return new_pose
    except ExtrapolationException as e:
        rospy.logwarn(e)

# def pub_debug(x):
#     y = np.array([1,0,0])
#     y = np.cross(x, y)
#     z = np.cross(x, y)
#     p = PoseStamped()
#     p.header.frame_id = 'gripper_tool_frame'
#     p.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[x[0], y[0], z[0], 0],
#                                                                       [x[1], y[1], z[1], 0],
#                                                                       [x[2], y[2], z[2], 0],
#                                                                       [0,0,0,1]])))
#     pub2.publish(p)

def publish_js():
    v = Vector3Stamped()
    v.header.frame_id = 'map'
    v.vector.z = -1
    v = transform_vector('gripper_tool_frame', v)
    g = np.array([v.vector.x, v.vector.y, v.vector.z])
    g = g / np.linalg.norm(g)
    goal_z = np.cross(g, [1,0,0])
    goal_z = goal_z / np.linalg.norm(goal_z)
    angle = np.arccos(np.array([0,0,1]).dot(goal_z))

    js = JointState()
    js.header = v.header
    js.name = ['refills_finger_joint']
    if goal_z[1] > 0:
        angle = -angle
    js.position = [angle]
    js.velocity = [0]
    js.effort = [0]
    pub.publish(js)
    # pub_debug(goal_z)

if __name__ == '__main__':
    rospy.init_node('refills_finger_js_publisher')
    init(2)
    pub = rospy.Publisher('refills_finger/joint_states', JointState, queue_size=10)
    # pub2 = rospy.Publisher('adsf', PoseStamped, queue_size=10)
    rate = rospy.Rate(120)
    while not rospy.is_shutdown():
        publish_js()
        rate.sleep()