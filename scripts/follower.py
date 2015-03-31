#!/usr/bin/env python

import roslib
import rospy
import actionlib
import tf

import math

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# constants
DIST_FROM_PREVIOUS = .3 # how close is too close that robot won't send a new goal
DIST_FROM_TARGET = .5 # how far away the robot should stop from the target
DIST_NEXT_GOAL = .7 # how far the next goal should idealy be
PROXIMITY_RELIABILITY = .4 # how far from DIST_NEXT_GOAL is going to bring extra reliaility
RELIABILITY_MIN = .4 #minimum reliability of the position

class ListenerSingleton:
    created = False
    listener = None

    @staticmethod
    def new():
        if (ListenerSingleton.created):
            return ListenerSingleton.listener
        else:
            ListenerSingleton.created = True
            ListenerSingleton.listener = tf.TransformListener()
            rospy.loginfo("created new instance of listener")
            return ListenerSingleton.listener    

class HumanFollower:

    def __init__(self):
        self.previousGoal = None

    def callback(self,data):
        # publish to whatever message the driving is going to take place
        pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
        positionPub = rospy.Publisher("currentPosition", PoseStamped, queue_size = 10)
        
        if len(data.people) > 0:
            # selecting most probable person
            rospy.loginfo("Looking for suitble target")

            maxReliability = RELIABILITY_MIN
            personIndex = -1

            for i in range(len(data.people)):
                
                # reliability metric is based on a combination of leg_detector results
                # and how far this current goal is from the pervious goal. 
                # If there is no previous goal, then it's simply the leg_detector results
                
                if (self.previousGoal == None):
                    reliability = data.people[i].reliability
                else:
                    currPersonPosition = data.people[i].pos
                    distFromPreviousGoalX = currPersonPosition.x - self.previousGoal.pose.position.x
                    distFromPreviousGoalY = currPersonPosition.y - self.previousGoal.pose.position.y
                    distFromPreviousGoal = math.hypot(distFromPreviousX, distFromPreviousY)
                    distError = math.abs(distFromPreviousGoal - DIST_NEXT_GOAL) # how far this goal is from the preffered distance
                    reliability = data.people[i].reliability + (PROXIMITY_RELIABILITY - distError)


                if (reliability > maxReliability):
                    personIndex = i
            
            if (personIndex != -1):
                rospy.loginfo("Found target, generating goal")
                try:
                    listener = ListenerSingleton.new()
                    (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
                    rospy.loginfo("Transform obtained")

                    # forming a message of current position for visualization
                    curr_Position = PoseStamped()
                    curr_Position.pose.position.x = trans[0]
                    curr_Position.pose.position.y = trans[1]
                    curr_Position.pose.position.z = 0
                    curr_Position.pose.orientation.x = rot[0]
                    curr_Position.pose.orientation.y = rot[1]
                    curr_Position.pose.orientation.z = rot[2]
                    curr_Position.pose.orientation.w = rot[3]
                    curr_Position.header.frame_id = 'map'
                    curr_Position.header.stamp = rospy.Time.now()

                    # This is where the target person's legs are                                      
                    legPosition = data.people[i].pos

                    rospy.loginfo("Computing target")

                    # computing target point that is set distance away    
                    differenceX = legPosition.x - trans[0]
                    differenceY = legPosition.y - trans[1]
                    
                    # calculating target location
                    angle = math.atan2(differenceY, differenceX)
                    length = math.hypot(differenceX, differenceY)
                    
                    target_length = length - DIST_FROM_TARGET

                    targetX = target_length * math.cos(angle) + trans[0]
                    targetY = target_length * math.sin(angle) + trans[1]

                    # forming target goal
                    target_goal_simple = PoseStamped()
                    target_goal_simple.pose.position.x = targetX
                    target_goal_simple.pose.position.y = targetY
                    target_goal_simple.pose.position.z = 0
                    target_goal_simple.pose.orientation.w = 1
                    target_goal_simple.header.frame_id = 'map'
                    target_goal_simple.header.stamp = rospy.Time.now()

                    # sending goal
                    if (self.previousGoal == None):
                        rospy.loginfo("first goal woo hoo!")
                        self.previousGoal = target_goal_simple
                        rospy.loginfo("sending goal")
                        pub.publish(target_goal_simple)
                    else:
                        #calculating distance from previous goal
                        diffX = target_goal_simple.pose.position.x - self.previousGoal.pose.position.x
                        diffY = target_goal_simple.pose.position.y - self.previousGoal.pose.position.y
                        dist = math.hypot(diffX, diffY)
                        
                        if (dist > DIST_FROM_PREVIOUS):
                            self.previousGoal = target_goal_simple
                            rospy.loginfo("sending new goal")
                            pub.publish(target_goal_simple)
                        else:
                            rospy.loginfo("")
                            rospy.loginfo("too close to current goal!")

                    # publish current position for visualization
                    positionPub.publish(curr_Position)

                except Exception as expt:
                    print type(expt)
                    print expt.args
            

    def run(self):
        rospy.init_node("human_follower")
        rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        hf = HumanFollower()
        hf.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("oh no, he's dead!")
