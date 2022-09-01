#!/usr/bin/env python
import time
import rospy
import math
import matplotlib.pyplot as plt
import sys
import tf
import json
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

#Class that will control all the navigation, defined at main
class NavStack(object):

    def __init__(self):
        
        #All the variables and commands that need to be initialized on startup
        self.position = PoseWithCovarianceStamped() #current position
        self.goal = PoseStamped() #to send the waypoints in succession
        self.points = [] #list of points
        self.datajson = {} #json data

        with open("Route.json", "r+") as fi:
            self.datajson = json.load(fi)

        #Variables to help with different things in the code
        self.point_attack = 0 #Counter of the last point the robot passed through
        self.first = 1 #Variable to check if it's the first time running the loop

        #Declaring publishers and subscribers
        self.goal_publisher_action = rospy.Publisher("/goal", MoveBaseActionGoal, queue_size=10)
        self.goal_publisher_stamped = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.status_sub = rospy.Subscriber("/move_base_pose", PoseWithCovarianceStamped, self.tf_listener)

        #A little delay so the callback can have time to gather the position
        time.sleep(2)


    #The tf listener
    #The tf is actually being get from another code and published in the /move_base_pose topic
    def tf_listener(self, data):
        self.posx = data.pose.pose.position.x
        self.posy = data.pose.pose.position.y
        self.orz = data.pose.pose.orientation.z
        self.orw = data.pose.pose.orientation.w


    #The goto function will declare the next waypoint position of the robot and publish it as a goal
    #However, the way the function works depends on how the goal is published
    #The gazebo simulations used a PoseStamped() message to acknowledge the goal
    #But some real robots use the MoveBaseActionGoal() message
    #With that in mind, I made two functions, one for each type of message, since I don't know which will be used
    #The first "def goto" uses the PoseStamped(), the second uses the MoveBaseActionGoal()
    #Just comment and uncomment the right one to make it work as needed, and change the self.goal variable at init

    def goto(self,x,y,z,w):
        self.goal.header.seq = 0
        self.goal.header.stamp = rospy.Time.now()
        self.goal.header.frame_id = "map"
        
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.orientation.z = z
        self.goal.pose.orientation.w = w

        #print(self.goal)
        self.goal_publisher_stamped.publish(self.goal)

    '''
    def goto(self,x,y,z,w):
        self.goal.goal.target_pose.header.seq = 0
        self.goal.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.goal.target_pose.header.frame_id = "map"
        
        self.goal.goal.target_pose.pose.position.x = x
        self.goal.goal.target_pose.pose.position.y = y
        self.goal.goal.target_pose.pose.orientation.z = z
        self.goal.goal.target_pose.pose.orientation.w = w
        #print(self.goal)
        self.goal_publisher_action.publish(self.goal)
            pass
    '''

    #The function to get the list of points to make the polygon shape
    def polygon(self, sides, length, pose):

        for things in self.datajson:
            self.points.append(things.x, things.y)


    #The main loop function for the navigation
    def navigate(self):
        
        #If it's the first time on the loop, sets the first goal
        if self.first == 1:
            self.goto(self.points[0][0], self.points[0][1], 0, 0)

            #Defines the variable as zero so it never comes back
            self.first = 0

        #Every time the robot gets to the right point, it changes the goal to the next one
        #There's a small tolerance of 0.2 in case it doesn't reach the actual point
        if self.posx > self.points[self.point_attack][0] - 0.2 and  self.posx < self.points[self.point_attack][0] + 0.2:
            if self.posy > self.points[self.point_attack][1] - 0.2 and self.posy < self.points[self.point_attack][1] + 0.2:

                #Prepares the next point to be sent
                self.point_attack += 1

                #If it's the last point, send a little end message
                if self.point_attack >= len(self.points):
                    print("End!")

                #If it's not the last point, send the next goal
                else:
                    self.goto(self.points[self.point_attack][0], self.points[self.point_attack][1], 0, 1)


#Initialization of the code
if __name__ == '__main__':
    #Defining the node and rate
    rospy.init_node("nav")
    rate = rospy.Rate(10)

    #Class startup
    nav_stack_object = NavStack()
    print("Initializing...")

    #Before the navigation, the list of points must be collected
    #This function will, first of all, collect all the points and make a list
    #Using the side and length args from the command line
    nav_stack_object.polygon()

    #Eternal loop for the navigation
    while not rospy.is_shutdown():

        #Essentially the "main loop" of the code, where the navigation will always run
        nav_stack_object.navigate()
        rate.sleep()
