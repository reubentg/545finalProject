#!/usr/bin/env python

import collections
import sys
import math
import time
import subprocess
import os

import rospy
import roslaunch
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Pose
from ackermann_msgs.msg import AckermannDriveStamped
# add this to python2.7 in settings
# /opt/ros/melodic/lib/python2.7/dist-packages

import utils

# The topics to get plan
MAP_TOPIC = 'static_map'  # The service topic that will provide the map
INIT_POSE_TOPIC = "/initialpose"
GOAL_POSE_TOPIC = "/move_base_simple/goal"
PLAN_POSE_ARRAY_TOPIC = "/planner_node/car_plan"
PLAN_POSE_ARRAY_TOPIC1 = "/planner_node/car_plan1"
PLAN_POSE_ARRAY_TOPIC2 = "/planner_node/car_plan2"
PLAN_POSE_ARRAY_TOPIC3 = "/planner_node/car_plan3"

def test(msg):
    print "initial pose msg: ", msg


def get_plan(initial_pose, goal_pose, counter):
    # Create a publisher to publish the initial pose
    init_pose_pub = rospy.Publisher(INIT_POSE_TOPIC, PoseWithCovarianceStamped,
                                    queue_size=1)  # to publish init position x=2500, y=640
    # Create a publisher to publish the goal pose
    goal_pose_pub = rospy.Publisher(GOAL_POSE_TOPIC, PoseStamped, queue_size=1)  # create a publisher for goal pose

    map_img, map_info = utils.get_map(MAP_TOPIC)  # Get and store the map
    PWCS = PoseWithCovarianceStamped()  # create a PoseWithCovarianceStamped() msg
    PWCS.header.stamp = rospy.Time.now()  # set header timestamp value
    PWCS.header.frame_id = "map"  # set header frame id value

    temp_pose = utils.map_to_world(initial_pose, map_info)  # init pose
    PWCS.pose.pose.position.x = temp_pose[0]
    PWCS.pose.pose.position.y = temp_pose[1]
    PWCS.pose.pose.position.z = 0
    PWCS.pose.pose.orientation = utils.angle_to_quaternion(temp_pose[
                                                               2])  # set msg orientation to [converted to queternion] value of the yaw angle in the look ahead pose from the path
    print "Pubplishing Initial Pose to topic", INIT_POSE_TOPIC
    print "Initial ", PWCS.pose
    init_pose_pub.publish(PWCS)  # publish initial pose, now you can add a PoseWithCovariance with topic of "/initialpose" in rviz

    if counter == 0:
        for i in range(0, 5):
            init_pose_pub.publish(PWCS)
            rospy.sleep(0.5)


    print "Initial Pose Set."
    # raw_input("Press Enter")



    PS = PoseStamped()  # create a PoseStamped() msg
    PS.header.stamp = rospy.Time.now()  # set header timestamp value
    PS.header.frame_id = "map"  # set header frame id value

    temp_pose = utils.map_to_world(goal_pose, map_info)  # init pose
    PS.pose.position.x = temp_pose[
        0]  # set msg x position to value of the x position in the look ahead pose from the path
    PS.pose.position.y = temp_pose[
        1]  # set msg y position to value of the y position in the look ahead pose from the path
    PS.pose.position.z = 0  # set msg z position to 0 since robot is on the ground
    PS.pose.orientation = utils.angle_to_quaternion(temp_pose[2])
    print "Pubplishing Goal Pose to topic", GOAL_POSE_TOPIC
    print "\nGoal ", PS.pose
    goal_pose_pub.publish(PS)

    print "Goal Pose Set"
    # raw_input("Press Enter")

    print "\nwaiting for plan ", str(counter), "\n"
    raw_plan = rospy.wait_for_message(PLAN_POSE_ARRAY_TOPIC, PoseArray)
    print "\nPLAN COMPUTED! of type:", type(raw_plan)

    for i in range(0, 5):
        path_part_pub = rospy.Publisher("/CoolPlan/plan" + str(counter), PoseArray,
                                         queue_size=1)  # create a publisher for plan lookahead follower
    rospy.sleep(1)
    path_part_pub.publish(raw_plan)
    rospy.sleep(1)
    return raw_plan


def main():
    rospy.init_node('plan_creator', anonymous=True)  # Initialize the node
    # time.sleep(12)
    # raw_input("\n\nPress enter if Ready to plan\n\n")
    sub = rospy.Subscriber(INIT_POSE_TOPIC, PoseWithCovarianceStamped, test)

    map_img, map_info = utils.get_map(MAP_TOPIC)  # Get and store the map

    individual_plan_parts = []

    # [x, y, theta (given in deg and coverted to rad)
    path_points =  [[2500, 640, -11.3099 * np.pi / 180.0],
                    [2600, 660, 30.0 * np.pi / 180.0],
                    [2410, 490, 160 * np.pi / 180.0],
                    [1880, 440,-150 * np.pi / 180.0],
                    [1435, 545, 160 * np.pi / 180.0],
                    [1250, 460, 160 * np.pi / 180.0],
                    [1050, 450, -160.0 * np.pi / 180.0],
                    [650.0, 650.0, -120 * np.pi / 180.0],
                    [540, 835, -120 * np.pi / 180.0]]

    for i in range(0, len(path_points) - 1):
        # raw_input("Enter to Continue")
        initial_pose = path_points[i]
        goal_pose = path_points[i + 1]
        individual_plan_parts.append(get_plan(initial_pose, goal_pose, i))
        os.system('rosnode kill planner_node')
        rospy.sleep(17) # wait for planner_node to

    # # Make Plan
    # initial_pose = [2500.0, 640.0, -0.1159]
    # # initial_pose = [540.0, 835.0, 0.0]
    # goal_pose = [2600.0, 660.0, 0.0]
    # cool_plan = get_plan(initial_pose, goal_pose)
    # individual_plan_parts.append(cool_plan)
    #
    # # Make Plan
    # initial_pose = [1880, 440.0, 0.0]
    # # initial_pose = [540.0, 835.0, 0.0]
    # goal_pose = [1435.0, 545.0, 0.0]
    # cool_plan = get_plan(initial_pose, goal_pose)
    # individual_plan_parts.append(cool_plan)
    #
    # # Make Plan
    # initial_pose = [1250.0, 460.0, 0.0]
    # # initial_pose = [540.0, 835.0, 0.0]
    # goal_pose = [540.0, 835.0, 0.0]
    # cool_plan = get_plan(initial_pose, goal_pose)
    # individual_plan_parts.append(cool_plan)

    # raw_input("Press Enter to when plan available...")  # Waits for ENTER key press

    # Use rospy.wait_for_message to get the plan msg
    # Convert the plan msg to a list of 3-element numpy arrays
    #     Each array is of the form [x,y,theta]
    # Create a LineFollower object

    # raw_plan = rospy.wait_for_message(plan_topic, PoseArray)
    plan_array = []
    path_part_pub = []
    for i, raw_plan in enumerate(individual_plan_parts):
        print "\nraw_plan ", i, " published of type: ", type(raw_plan)
        for pose in raw_plan.poses:
            plan_array.append(np.array([pose.position.x, pose.position.y, utils.quaternion_to_angle(pose.orientation)]))

    # rospy.spin()  # Prevents node from shutting down
    print "DONE"

if __name__ == '__main__':
    main()
