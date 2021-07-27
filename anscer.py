import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


ranges=0
def callback(msg):
    global ranges
    ranges=msg.ranges[0]

def pub():
    global ranges
    velocity_msg=Twist()
    vel_publisher=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rate=rospy.Rate(1)
    print(ranges)
    rospy.loginfo('starting')
    while not rospy.is_shutdown():
        velocity_msg.linear.x=0.3
        velocity_msg.angular.z=0
        vel_publisher.publish(velocity_msg)
        rospy.loginfo(velocity_msg)
        if ranges<1.0:
            velocity_msg.linear.x=0
            velocity_msg.angular.z=math.pi/2
            vel_publisher.publish(velocity_msg)
            rospy.loginfo("OBSTACLE DETECTED")
        rate.sleep()

if __name__=="__main__":
    try: 
        rospy.init_node('scan_and_stop')
        rospy.Subscriber('/scan',LaserScan,callback=callback)
        pub()
    except rospy.ROSInterruptException:
        print("STOPPED")

