#!/usr/bin/env python

import roslib
import rospy
import tf

import math
import sys, os

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker

# constants
DIST_MIN = .3 # how close is too close that robot won't send a new goal
DIST_MAX = 3 # how far is too far that robot should not consider it as new person
ANGLE_THRESHOLD = math.pi / 6 # how wide is too wide robot will send new goal
DIST_FROM_TARGET = .5 # how far away the robot should stop from the target
PROXIMITY_MAX = .4 # how far from last known position leg detector should consider to be probable
R_SCALE = .5 # scale from distance to reliability
RELIABILITY_MIN = .4 #minimum reliability of the position

class ListenerSingleton:
    created = False
    listener = None

    @staticmethod
    def new():
        if ListenerSingleton.created:
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
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 10)
        self.position_pub = rospy.Publisher('currentPosition', PoseStamped, queue_size = 10)
        self.marker_pub = rospy.Publisher('marker', Marker, queue_size = 10)

        self.previous_goal = None
        self.last_known_position = None
        self.tracked_object_id = "Steve"


    def callback(self, data):
        # get transform
        listener = ListenerSingleton.new()
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
        rospy.loginfo("Transform obtained")

        # sends current position for visualization
        self.send_current_position(trans, rot)

        # process leg detector input
        if data.people:
            #person_index = self.find_reliable_target(data, trans)
            person = self.find_reliable_target(data, trans)
            #rospy.loginfo('person_index**************')
            #rospy.loginfo(person_index)
            rospy.loginfo('data.people***************')
            rospy.loginfo(len(data.people))
            rospy.loginfo('robot pos***********************')
            rospy.loginfo(trans)
            rospy.loginfo(rot)

            # found someone more probable than the min probability.
            #if person_index != -1:
            if person:
                rospy.loginfo("Target Found")

                #try:
                # logs the start of goal computation
                rospy.loginfo("Computing goal")

                # This is where the target person's legs are
                #leg_position = data.people[person_index].pos
                leg_position = person.pos
                rospy.loginfo('person pos***************************************************************')
                rospy.loginfo(person.pos)

                # setting last known position regardless of if the goal is sent or not
                # angle is not important. Last Known position only needs the coordinates
                self.last_known_position = GoalEuler(leg_position.x, leg_position.y, 0)

                # computing target point that is set distance away
                difference_x = leg_position.x - trans[0]
                difference_y = leg_position.y - trans[1]

                #publish marker for robot
                #self.publish_marker(trans[0], trans[1], 0)
                #publish marker for target
                self.publish_marker(leg_position.x, leg_position.y, 0, 'person')
                self.publish_marker(0.0, 0.0, 0.0, 'robot')

                # calculating target location
                goal_angle = math.atan2(difference_y, difference_x)
                length = math.hypot(difference_x, difference_y)

                # calculating the position of the goal
                target_length = length - DIST_FROM_TARGET
                goalx = target_length * math.cos(goal_angle) + trans[0]
                goaly = target_length * math.sin(goal_angle) + trans[1]

                # sending goal if it is sufficiently different or the first goal
                rospy.loginfo("judging goal")
                if (not self.previous_goal or self.check_goal_difference(goalx, goaly, goal_angle)):

                    self.previous_goal = GoalEuler(goalx, goaly, goal_angle)
                    self.tracked_object_id = person.object_id

                    target_goal_simple = self.build_goal_quaternion(goalx, goaly, goal_angle) 

                    rospy.loginfo("sending goal")
                    #self.goal_pub.publish(target_goal_simple)
                else:
                    rospy.loginfo("new goal not sufficiently different. Canclled.")

            #except Exception as expt:
                #exc_type, exc_obj, exc_tb = sys.exc_info()
                #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                #print(exc_type, fname, exc_tb.tb_lineno)
                #print type(expt)
            #    print expt.args


    def build_goal_quaternion(self, goalx, goaly, goal_angle):
        rospy.loginfo("building final goal")
        # calculating the quaterion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, goal_angle)

        # forming target goal
        goal = PoseStamped()

        goal.pose.position.x = goalx
        goal.pose.position.y = goaly
        goal.pose.position.z = 0

        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()

        return goal

    def check_goal_difference(self, goalx, goaly, goal_angle):
        # check if distance is far enough
        dist_diff = math.hypot(goalx - self.previous_goal.x, goaly - self.previous_goal.y)
        angle_diff = math.fabs(goal_angle - self.previous_goal.angle)

        # if either is greather than threshold, we should send new goal
        return (dist_diff > DIST_MIN or angle_diff > ANGLE_THRESHOLD)

    def find_reliable_target(self, data, robot_position):
        # selecting most probable person
        rospy.loginfo("Filtering for suitible target")

        #max_reliability = RELIABILITY_MIN
        reliability = 0
        #person_index = -1

        for person in data.people:
            # reliability metric is based on a combination of leg_detector results
            # and how far this current goal is from the pervious goal.
            # if the same person is still in sight, it is the most reliable
            # If there is no previous goal, then it's simply the leg_detector reliability

            curr_person_position = person.pos

            if (not self.previous_goal):
                reliability = person.reliability
            else:
                dist_from_robot = math.hypot(curr_person_position.x - robot_position[0], curr_person_position.y - robot_position[1])
                dist_from_lastx = curr_person_position.x - self.last_known_position.x
                dist_from_lasty = curr_person_position.y - self.last_known_position.y
                dist_from_lastknown = math.hypot(dist_from_lastx, dist_from_lasty)

                if (person.object_id == self.tracked_object_id):
                    reliability = 100
                elif (dist_from_robot > DIST_MAX):
                    reliability = -100
                else:
                    # general case not the first goal
                    if (dist_from_lastknown < PROXIMITY_MAX):
                        reliability = person.reliability + ((PROXIMITY_MAX - dist_from_lastknown) * R_SCALE)
                    else:
                        relability = person.reliability

            #if ()reliability > max_reliability):
            if reliability > RELIABILITY_MIN:
                #max_reliability = reliability
                #person_index = i
                return person

        rospy.loginfo("count: " + str(len(data.people)))
        rospy.loginfo("final R: " + str(reliability))

        #return person_index
        return


    def send_current_position(self, trans, rot):

        curr_position = PoseStamped()
        curr_position.pose.position.x = trans[0]
        curr_position.pose.position.y = trans[0]
        curr_position.pose.position.z = 0
        curr_position.pose.orientation.x = rot[0]
        curr_position.pose.orientation.y = rot[1]
        curr_position.pose.orientation.z = rot[2]
        curr_position.pose.orientation.w = rot[3]
        curr_position.header.frame_id = 'map'
        curr_position.header.stamp = rospy.Time.now()

        # publishing current position for visualization
        self.position_pub.publish(curr_position)

    def publish_marker(self, x, y, z, ns):
        marker = Marker()
        #marker.header.frame_id = '/camera_rgb_optical_frame'
        marker.header.frame_id = 'map'
        #marker.header.stamp = Time()
        marker.ns = ns
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
#        marker.pose.orientation.x = 0.0
#        marker.pose.orientation.y = 0.0
#        marker.pose.orientation.z = 0.0
#        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.color.a = 0.8
        marker.color.r = 0.3
        marker.color.g = 0.4
        if ns == 'person':
            marker.color.b = 1.0
        else:
            marker.color.b = 0.2

        self.marker_pub.publish(marker)


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
