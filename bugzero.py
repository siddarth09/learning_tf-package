#!usr/bin/env
import rospy 
import roslib
from geometry_msgs.msg import Pose,Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import std_srvs.srv
import math
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class bugzero():
    def __init__(self):
        self.goto_point_srv=None
        self.wall_follwer_srv=None
        self.yaw=0
        self.yaw_error= 5*(math.pi/180)
        self.position=Point()
        self.ini_pos=Point()
        self.ini_pos.x= rospy.get_param('initial_x')
        self.ini_pos.y=rospy.get_param('initial_y')
        self.ini_pos.z=0
        self.desired_position_ = Point()
        self.desired_position_.x = rospy.get_param('des_pos_x')
        self.desired_position_.y = rospy.get_param('des_pos_y')
        self.desired_position_.z = 0
        self.regions=None
        self.state_description=['go to goal','wall follwer']
        self.state=0

    def odom_callback(self,msg):
        
        self.position=msg.pose.pose.position
        #quaternion
        self.quaternion=(msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)
        self.rpy=tf.transformations.euler_from_quaternion(self.quaternion)
        self.Yaw=self.rpy[2]

    def laser_callback(self,msg):

        self.regions= {
            'right':  min(min(msg.ranges[0:143]), 10),
            'front_right': min(min(msg.ranges[144:287]), 10),
            'front':  min(min(msg.ranges[288:431]), 10),
            'front_left':  min(min(msg.ranges[432:575]), 10),
            'left':   min(min(msg.ranges[576:719]), 10),

            }

    def changed_state(self):
        self.change="state changed: %s" % self.state_description[self.state]
        rospy.loginfo(self.change)
        if self.state==0:
            self.response=self.goto_point_srv(True)
            self.response=self.wall_follwer_srv(False)
        elif self.state==1:
            self.response=self.goto_point_srv(False)
            self.response=self.wall_follwer_srv(True)

    def normalize_angle(self,angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def main(self):
        rospy.init_node('bug_0')
        #SUBCRIBING
        self.laser_subcriber=rospy.Subscriber('laser/scan',LaserScan,self.laser_callback)
        self.odom_subcriber=rospy.Subscriber('/odom',Odometry,self.odom_callback)

        rospy.wait_for_service('/go_to_point_switch')
        rospy.wait_for_service('/wall_follower_switch')
        rospy.wait_for_service('/gazebo/set_model_state')

        self.goto_point_srv=rospy.ServiceProxy('/go_to_point_switch')
        self.wall_follwer_srv=rospy.ServiceProxy('/wall_follower_switch')
        self.model_state=rospy.ServiceProxy('/gazebo/set_model_state')

        self.model=ModelState()
        self.model.model_name='turtlebot3_waffle'
        self.model.pose.position.x=self.ini_pos.x
        self.model.pose.position.x=self.ini_pos.y

        self.changed_state(0)

        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.regions==None:
                continue
            if self.state==0:
                if self.regions['front']>0.15 and self.regions['front']<1:
                    self.changed_state(1)
            elif self.state==1:
                self.desired_yaw= math.atan2(self.desired_position_.y-self.position.y,self.desired_position_.x-self.position.x)
                self.error_yaw=self.normalize_angle(self.desired_yaw-self.yaw)
                #getting the angle 
                #0<theta<30 degree
                if math.fabs(self.error_yaw)< (math.pi/6) and self.regions['front']>1.5 and self.regions['front_right']>1 and self.regions['front_left']>1:
                    print("LESS THE 30 degrees")
                    self.changed_state(0)
                 # between 30 and 90 (left)
                if self.error_yaw> 0 and math.fabs(self.error_yaw) > (math.pi / 6) and math.fabs(self.error_yaw) < (math.pi / 2) and self.regions['left'] > 1.5 and self.regions['front_left'] > 1:
                    print ('between 30 and 90 - to the left')
                    self.change_state(0)
                #betweeen 30 and 90 (right)
                if self.error_yaw < 0 and math.fabs(self.error_yaw) > (math.pi / 6) and math.fabs(self.error_yaw) < (math.pi / 2) and self.regions['right'] > 1.5 and self.regions['front_right'] > 1:
                    print('between 30 and 90 - to the right')
                    self.change_state(0)
            
        rate.sleep()

















