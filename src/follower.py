#!/usr/bin/env python

import math
import sys, os

from geometry_msgs.msg import *
from move_base_msgs.msg import move_base_action, move_base_goal
from people_msgs.msg import position_measurement_array
import roslib
import rospy
import tf


class Constant(object):
    # how close is too close that robot won't send a new goal
    dist_min = .3
    # how far is too far that robot should not consider it as new person
    dist_max = 3
    # how wide is too wide robot will send new goal
    angle_threshold = math.pi / 6
    # how far away the robot should stop from the target
    dist_from_target = .5
    # how far from last known position leg detector should consider to be probable
    proximity_max = .4
    # scale from distance to reliability
    r_scale = .5
    # minimum reliability of the position
    reliability_min = .4
    
class ListenerSingleton:
    created = false
    listener = none

    @staticmethod
    def new():
        if (ListenerSingleton.created):
            return ListenerSingleton.listener
        else:
            ListenerSingleton.created = true
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
        self._pub = rospy.publisher("move_base_simple/goal", pose_stamped, queue_size = 10)
        self.position_pub = rospy.publisher("current_position", pose_stamped, queue_size = 10)

        self.previous_goal = none
        self.last_known_position = none
        self.tracked_object_id = "steve"


    def callback(self, data):
        # get transform
        listener = ListenerSingleton.new()
        (trans, rot) = listener.lookup_transform('/map', '/base_link', rospy.time())
        rospy.loginfo("transform obtained")

        # sends current position for visualization
        self.send_current_position(trans, rot)

        # process leg detector input
        if len(data.people) > 0:
            person_index = self.find_reliable_target(data, trans)

            # found someone more probable than the min probability.
            if (person_index != -1):
                rospy.loginfo("target found")

                try:

                    # logs the start of goal computation
                    rospy.loginfo("computing goal")

                    # this is where the target person's legs are
                    leg_position = data.people[person_index].pos

                    # setting last known position regardless of if the goal is sent or not
                    # angle is not important. last known position only needs the coordinates
                    self.last_known_position = GoalEuler(leg_position.x, leg_position.y, 0)

                    # computing target point that is set distance away
                    difference_x = leg_position.x - trans[0]
                    difference_y = leg_position.y - trans[1]

                    # calculating target location
                    goal_angle = math.atan2(difference_y, difference_x)
                    length = math.hypot(difference_x, difference_y)

                    # calculating the position of the goal
                    target_length = length - Constant.dist_from_target
                    goal_x = target_length * math.cos(goal_angle) + trans[0]
                    goal_y = target_length * math.sin(goal_angle) + trans[1]


                    # sending goal if it is sufficiently different or the first goal
                    rospy.loginfo("judging goal")
                    if (self.previous_goal == none or self.check_goal_difference(goal_x, goal_y, goal_angle)):

                        self.previous_goal = GoalEuler(goal_x, goal_y, goal_angle)
                        self.tracked_object_id = data.people[person_index].object_id

                        target_goal_simple = self.build_goal_quaternion(goal_x, goal_y, goal_angle) 

                        rospy.loginfo("sending goal")
                        self.goal_pub.publish(target_goal_simple)
                    else:
                        rospy.loginfo("new goal not sufficiently different. canclled.")

                except exception as expt:
                    #exc_type, exc_obj, exc_tb = sys.exc_info()
                    #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                    #print(exc_type, fname, exc_tb.tb_lineno)
                    #print type(expt)
                    print expt.args


    def build_goal_quaternion(self, goal_x, goal_y, goal_angle):
        rospy.loginfo("building final goal")
        # calculating the quaterion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, goal_angle)

        # forming target goal
        goal = pose_stamped()

        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0

        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]

        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.time.now()

        return goal

    def check_goal_difference(self, goal_x, goal_y, goal_angle):
        # check if distance is far enough
        dist_diff = math.hypot(goal_x - self.previous_goal.x, goal_y - self.previous_goal.y)
        angle_diff = math.fabs(goal_angle - self.previous_goal.angle)

        # if either is greather than threshold, we should send new goal
        return (dist_diff > Constant.dist_min or angle_diff > Constant.angle_threshold)

    def find_reliable_target(self, data, robo_position):
        # selecting most probable person
        rospy.loginfo("filtering for suitible target")

        max_reliability = Constant.reliability_min
        reliability = 0
        person_index = -1

        for i in range(len(data.people)):

            # reliability metric is based on a combination of leg_detector results
            # and how far this current goal is from the pervious goal.
            # if the same person is still in sight, it is the most reliable
            # if there is no previous goal, then it's simply the leg_detector reliability

            curr_person_position = data.people[i].pos

            if (not self.previous_goal):
                reliability = data.people[i].reliability
            else:

                dist_from_robot = math.hypot(curr_person_position.x - robo_position[0], curr_person_position.y - robo_position[1])
                dist_from_last_x = curr_person_position.x - self.last_known_position.x
                dist_from_last_y = curr_person_position.y - self.last_known_position.y
                dist_from_last_known = math.hypot(dist_from_last_x, dist_from_last_y)

                if (data.people[i].object_id == self.tracked_object_id):
                    reliability = 100
                elif (dist_from_robot > Constant.dist_max):
                    reliability = -100
                else:
                    # general case not the first goal
                    if (dist_from_last_known < Constant.proximity_max):
                        reliability = data.people[i].reliability + ((Constant.proximity_max - dist_from_last_known) * Constant.r_scale)
                    else:
                        reliability = data.people[i].reliability

            if (reliability > max_reliability):
                max_reliability = reliability
                person_index = i

        rospy.loginfo("count: " + str(len(data.people)))
        rospy.loginfo("final r: " + str(reliability))

        return person_index


    def send_current_position(self, trans, rot):

        curr_position = pose_stamped()
        curr_position.pose.position.x = trans[0]
        curr_position.pose.position.y = trans[0]
        curr_position.pose.position.z = 0
        curr_position.pose.orientation.x = rot[0]
        curr_position.pose.orientation.y = rot[1]
        curr_position.pose.orientation.z = rot[2]
        curr_position.pose.orientation.w = rot[3]
        curr_position.header.frame_id = 'map'
        curr_position.header.stamp = rospy.time.now()

        # publishing current position for visualization
        self.position_pub.publish(curr_position)

    def run(self):
        rospy.init_node("HumanFollower")
        rospy.subscriber('people_tracker_measurements',position_measurement_array, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        hf = HumanFollower()
        hf.run()
    except rospy.rosinterrupt_exception:
        rospy.loginfo("oh no, he's dead!")

