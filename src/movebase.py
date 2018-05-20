#!/usr/bin/env python
import roslib; roslib.load_manifest('navigate')
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class goalNavigation():
    
    def selection(self):
        rospy.loginfo("")
        rospy.loginfo("Enter a selection of your choice from following options:")
        rospy.loginfo("Room.No:1 - 1")
        rospy.loginfo("Room.No:2 - 2")
        rospy.loginfo("Room.No:3 - 3")
        rospy.loginfo("IntialPose - 4")
        rospy.loginfo("")
        option = input()
        return option

    def __init__(self):
        positionRoom1 = {'x': 1.64, 'y' : 7.29}
        quaternionRoom1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}
        positionRoom2 = {'x': 5.76, 'y' : 8.32}
        quaternionRoom2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.700, 'r4' : 0.700}
        positionRoom3 = {'x': 8.02, 'y' : 2.39}
        quaternionRoom3 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        positionInitial = {'x': 1.5, 'y' : 0.5}
        quaternionInitial = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        option = self.selection()

        if (option == 1):
            self.goalReached = self.goToGoal(positionRoom1, quaternionRoom1)
        elif (option == 2):
            self.goalReached = self.goToGoal(positionRoom2, quaternionRoom2)
        elif (option == 3):
            self.goalReached = self.goToGoal(positionRoom3, quaternionRoom3)
        elif (option == 4):
            self.goalReached = self.goToGoal(positionInitial, quaternionInitial)

        if (option != 0):
            if (self.goalReached):
                rospy.loginfo("Successfully reached the goal")
        else:
            rospy.loginfo("Failed to reach the goal")

        while option != 0:
            option = self.selection()
            if (option == 1):
                self.goalReached = self.goToGoal(positionRoom1, quaternionRoom1)
            elif (option == 2):
                self.goalReached = self.goToGoal(positionRoom2, quaternionRoom2)
            elif (option == 3):
                self.goalReached = self.goToGoal(positionRoom3, quaternionRoom3)
            elif (option == 4):
                self.goalReached = self.goToGoal(positionInitial, quaternionInitial)

            if (option != 0):
                if (self.goalReached):
                    rospy.loginfo("Successfully reached the goal")
                else:
                    rospy.loginfo("Failed to reach the goal")

    def goToGoal(self, position, quat):
        # Action Client to send goal requests to the move_base server
	actionClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for 5 seconds for the action server to come up
        #self.move_base.wait_for_server(rospy.Duration(5))
        while(not actionClient.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to start")

        # Send a goal
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(position['x'], position['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        rospy.loginfo("Starting to move towards the goal")

	# Start moving towards the goal
        actionClient.send_goal(goal)

	# Waiting for a duration of 60 seconds for the robot to reach the goal
	success = actionClient.wait_for_result(rospy.Duration(60)) 

        if actionClient.get_state() ==  GoalStatus.SUCCEEDED:
            rospy.loginfo("The turtlebot has reached the goal")
            return True
        else:
            rospy.loginfo("The turtlebot has failed to reach the goal")

    def shutdown(self):
        rospy.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('navigator', anonymous=False)
        goalNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("navigation got terminated due to interrupt")
