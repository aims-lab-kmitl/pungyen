#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

cmd_vel_ = Twist()
pose_ = PoseWithCovarianceStamped()

global check_x_, check_y_

def commandPoseCallback(pose_msg_):
    global pose_
    pose_ = pose_msg_

def initData():
    global cmd_vel_, check_x_, check_y_
    cmd_vel_.linear.x = 0.0
    cmd_vel_.linear.y = 0.0
    cmd_vel_.linear.z = 0.0
    cmd_vel_.angular.x = 0.0
    cmd_vel_.angular.y = 0.0
    cmd_vel_.angular.z = 0.0
    check_x_ = True
    check_y_ = True

def gotogoal(x, y):
    global cmd_vel_, pose_, check_x_, check_y_
    
    if abs(x - pose_.pose.pose.position.x) >= 0.05:
        check_x_ = False
        if pose_.pose.pose.position.x - x <= 0.0:
            cmd_vel_.linear.x = 0.3
        elif pose_.pose.pose.position.x - x >= 0.0:
            cmd_vel_.linear.x = -0.3
    elif abs(x - pose_.pose.pose.position.x) <= 0.05:
        check_x_ = True
        cmd_vel_.linear.x = 0.0
        
    
    if abs(y - pose_.pose.pose.position.y) >= 0.03:
        check_y_ = False
        if pose_.pose.pose.position.y - y <= 0.0:
            cmd_vel_.linear.y = 0.3
        elif pose_.pose.pose.position.y - y >= 0.0:
            cmd_vel_.linear.y = -0.3
    elif abs(y - pose_.pose.pose.position.y) <= 0.03:
        check_y_ = True
        cmd_vel_.linear.y = 0.0

def main():
    global cmd_vel_, pose_, check_x_, check_y_
    
    rospy.init_node('robot_gotogoal_node', anonymous=True)
    rate = rospy.Rate(10)
    initData()
    gotox = 0.0
    gotoy = 0.0
    
    cmd_vel_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    odom_pose_sub_ = rospy.Subscriber('/odom_combined', PoseWithCovarianceStamped, commandPoseCallback)
    
    while not rospy.is_shutdown():
        if check_x_ and check_y_ :
            gotox = float(input("Set your x goal:"))
            gotoy = float(input("Set your y goal:"))
            gotogoal(gotox, gotoy)
        else:
            gotogoal(gotox, gotoy)
        
        cmd_vel_pub_.publish(cmd_vel_)
        print("Pose X: %f, Pose Y: %f" % (pose_.pose.pose.position.x, pose_.pose.pose.position.y))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass