#!usr/bin/env
import rospy
import time
import math
import tf
import roslib

def brodcaster(x,y,z,w):
    rospy.init_node('frame_a_to_frame_b_brodcaster_node',anonymous=False)
    time.sleep(0.5)
    bc=tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        # we need to brodcast translation,rotation and time
        quaternion=tf.transformations.quaternion_from_euler(x,y,z)
        translation=(1.0,2.0,3.0)
        Time=rospy.Time.now()

        bc.sendTransform(translation,quaternion,Time,"robot_camera","robot_wheels")

if __name__=="__main__":
    brodcaster(0.8,0.1,0.3,0.6)