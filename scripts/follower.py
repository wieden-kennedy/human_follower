#!/usr/bin/env python

import roslib
import rospy
import tf

import math
import sys, os

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# constants
DIST_FROM_PREVIOUS = .3 # how close is too close that robot won't send a new goal
DIST_FROM_TARGET = .6 # how far away the robot should stop from the target
PROXIMITY_MAX = .4 # how far from last known position leg detector should consider to be probable
R_SCALE = .5
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

class GoalEuler:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

class HumanFollower:

    def __init__(self):
        self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
        self.positionPub = rospy.Publisher("currentPosition", PoseStamped, queue_size = 10)

        self.previousGoal = None
        self.lastKnownPosition = None
        self.trackedObjectID = "Steve"


    def callback(self, data):
        # get transform
        listener = ListenerSingleton.new()
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
        rospy.loginfo("Transform obtained")

        # sends current position for visualization
        self.sendCurrentPosition(trans, rot)

        # process leg detector input
        if len(data.people) > 0:
            personIndex = self.findReliableTarget(data)

            # found someone more probable than the min probability.
            if (personIndex != -1):
                rospy.loginfo("Target Found")
                
                try:

                    # logs the start of goal computation
                    rospy.loginfo("Computing goal")

                    # This is where the target person's legs are                                      
                    legPosition = data.people[personIndex].pos

                    # setting last known position regardless of if the goal is sent or not
                    self.lastKnownPosition = GoalEuler(legPosition.x, legPosition.y, angle)                    

                    # computing target point that is set distance away    
                    differenceX = legPosition.x - trans[0]
                    differenceY = legPosition.y - trans[1]
                    
                    # calculating target location
                    angle = math.atan2(differenceY, differenceX)
                    length = math.hypot(differenceX, differenceY)
                    
                    # calculating the position of the goal
                    target_length = length - DIST_FROM_TARGET
                    goalX = target_length * math.cos(angle) + trans[0]
                    goalY = target_length * math.sin(angle) + trans[1]


                    # sending goal if it is sufficiently different
                    rospy.loginfo("judging goal")
                    if (self.previousGoal == None):
                        rospy.loginfo("first goal woo hoo!")

                        self.previousGoal = GoalEuler(goalX, goalY, angle)
                        self.trackedObjectID = data.people[personIndex].object_id

                        target_goal_simple = self.buildGoalQuaternion(goalX, goalY, angle) 

                        rospy.loginfo("sending goal")
                        self.pub.publish(target_goal_simple)
                    else:
                        #calculating distance from previous goal
                        distX = goalX - self.previousGoal.x
                        distY = goalY - self.previousGoal.y
                        dist = math.hypot(distX, distY)
                        
                        if (dist > DIST_FROM_PREVIOUS):
                            self.previousGoal = GoalEuler(goalX, goalY, angle)

                            target_goal_simple = self.buildGoalQuaternion(goalX, goalY, angle)

                            rospy.loginfo("sending new goal")
                            self.pub.publish(target_goal_simple)
                        else:
                            rospy.loginfo("new goal canceled: too close to current goal!")
                
                except Exception as expt:
                    #exc_type, exc_obj, exc_tb = sys.exc_info()
                    #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                    #print(exc_type, fname, exc_tb.tb_lineno)
                    #print type(expt)
                    print expt.args
                

    def buildGoalQuaternion(self, goalX, goalY, angle):
        rospy.loginfo("building final goal")
        # calculating the quaterion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)

        # forming target goal
        goal = PoseStamped()
        
        goal.pose.position.x = goalX
        goal.pose.position.y = goalY
        goal.pose.position.z = 0
        
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()

        return goal

            
    def findReliableTarget(self, data):
        # selecting most probable person
        rospy.loginfo("Looking for suitible target")

        maxReliability = RELIABILITY_MIN
        reliability = 0
        personIndex = -1

        for i in range(len(data.people)):
            
            # reliability metric is based on a combination of leg_detector results
            # and how far this current goal is from the pervious goal.
            # if the same person is still in sight, it is the most reliable
            # If there is no previous goal, then it's simply the leg_detector reliability
            
            if (self.previousGoal == None):
                reliability = data.people[i].reliability
            else:
                if (data.people[i].object_id == self.trackedObjectID):
                    reliability = 100
                else:
                    currPersonPosition = data.people[i].pos

                    # check distance from last known position of last most reliable person
                    distFromLastKnownX = currPersonPosition.x - self.lastKnownPosition.x
                    distFromLastKnownY = currPersonPosition.y - self.lastKnownPosition.y
                    distFromLastKnown = math.hypot(distFromLastKnownX, distFromLastKnownY)

                    if (distFromLastKnown < PROXIMITY_MAX):
                        reliability = data.people[i].reliability + ((PROXIMITY_MAX - distFromLastKnown) * R_SCALE)
                    else:
                        reliability = data.people[i].reliability
                    
            if (reliability > maxReliability):
                maxReliability = reliability
                personIndex = i

        rospy.loginfo("count: " + str(len(data.people)))
        rospy.loginfo("final R: " + str(reliability))

        return personIndex


    def sendCurrentPosition(self, trans, rot):
        curr_Position = PoseStamped()
        curr_Position.pose.position.x = trans[0]
        curr_Position.pose.position.y = trans[0]
        curr_Position.pose.position.z = 0
        curr_Position.pose.orientation.y = rot[1]
        curr_Position.pose.orientation.z = rot[2]
        curr_Position.pose.orientation.w = rot[3]
        curr_Position.header.frame_id = 'map'
        curr_Position.header.stamp = rospy.Time.now()

        # publishing current position for visualization
        self.positionPub.publish(curr_Position)

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
