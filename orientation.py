#!user/bin/env
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

print("------------------------------------")
print("ROLL PITCH YAW CONVERSION")

x=math.radians(30)
y=math.radians(42)
z=math.radians(38)

print("ROLL= {0}, PITCH= {1}, YAW= {2}".format(math.degrees(x),math.degrees(y),math.degrees(z)))

quaternion=tf.transformations.quaternion_from_euler(x,y,z)
print("--------------------------------")
print("VALUES FO QUATERNION")
for i in range(4):
    print(quaternion[i])


print("--------------------------------------")
print("QUATERNION CONVERSION")
rpy=tf.transformations.euler_from_quaternion(quaternion)
roll=rpy[0]
pitch=rpy[1]
yaw=rpy[2]

print("ROLL= {0}, PITCH= {1}, YAW= {2}".format(math.degrees(roll),math.degrees(pitch),math.degrees(yaw)))
