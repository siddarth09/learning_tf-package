import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import math
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

def to_goal():
    print("ENTER THE GOAL TO BE REACHED")
    x=float(input())
    y=float(input())
   
    ac=actionlib.SimpleActionClient('move_base',MoveBaseAction)

    goal=MoveBaseGoal()
    while (not ac.wait_for_server(rospy.Duration(5))):
        rospy.loginfo("WAITING FOR THE SERVER TO START")

    #setup frame parameters for the bot
    goal.target_pose.header.frame_id='map'
    goal.target_pose.header.stamp=rospy.Time.now()

    #setup intital pose
    goal.target_pose.pose.position=Point(x,y,0)#contains a point in free space
    goal.target_pose.pose.orientation.x=0.0
    goal.target_pose.pose.orientation.y=0.0
    goal.target_pose.pose.orientation.z=0.0
    goal.target_pose.pose.orientation.w=1.0

    rospy.loginfo("SENDING GOAL INFORMATION")
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))

    if (ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("REACHED THE DESTINATION")
        return True
    else:
        rospy.loginfo("THE BOT DIDN'T REACH ITS DESTINATION")
        return False

if __name__=="__main__":

    rospy.init_node('map_navigation',anonymous=False)
    to_goal()
    rospy.spin()

