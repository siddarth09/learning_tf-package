#!usr/bin/env
import rospy
import time
import math
import tf
import roslib

def listener():
    rospy.init_node("frame_a_to_frame_b_listener_node",anonymous=False)
    time.sleep(2)
    listener = tf.TransformListener()
    rate = rospy.Rate(1.0)
    listener.waitForTransform('/robot_camera','/robot_wheels', rospy.Time(), rospy.Duration(4.0),polling_sleep_duration=None)
    while not rospy.is_shutdown():
        try:
            (translation,rotation) = listener.lookupTransform('/robot_camera','/robot_wheels', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        quaternion = rotation
        rpy=tf.transformations.euler_from_quaternion(quaternion)
        print("TRANSFORMATION")
        print("-------------------")
        print("TRANSLATION =",translation[0],translation[1],translation[2])
        print("ROLL=",rpy[0], "PITCH=",rpy[1], "YAW=",rpy[2])
        rate.sleep()


if __name__=="__main__":
    listener()