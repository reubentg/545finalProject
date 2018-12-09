#!/usr/bin/env python

import collections
import sys
import math
import time
import subprocess
import os
import pickle

import rospy
import roslaunch
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Pose
from ackermann_msgs.msg import AckermannDriveStamped
# add this to python2.7 in settings
# /opt/ros/melodic/lib/python2.7/dist-packages

import utils
import rospkg

# The topics to get plan
MAP_TOPIC = 'static_map'  # The service topic that will provide the map
INIT_POSE_TOPIC = "/initialpose"
GOAL_POSE_TOPIC = "/move_base_simple/goal"
PLAN_POSE_ARRAY_TOPIC = "/planner_node/car_plan"

rospack = rospkg.RosPack()
RACECAR_PKG_PATH = rospack.get_path('racecar')
PLANNER_PKG_PATH = rospack.get_path('planning_utils')

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

    path_part_pub = rospy.Publisher("/CoolPlan/plan" + str(counter), PoseArray,
                                    queue_size=1)  # create a publisher for plan lookahead follower
    for i in range(0, 4):
        path_part_pub.publish(raw_plan)
        rospy.sleep(0.4)

    return raw_plan


def main():
    rospy.init_node('plan_creator', anonymous=True)  # Initialize the node

    # launch nodes from python
    # https://answers.ros.org/question/263862/if-it-possible-to-launch-a-launch-file-from-python/
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    map_server_launch = \
        roslaunch.parent.ROSLaunchParent(uuid, [RACECAR_PKG_PATH + "/launch/includes/common/map_server.launch"])

    map_server_launch.start()

    # launch nodes from python
    # https://answers.ros.org/question/263862/if-it-possible-to-launch-a-launch-file-from-python/
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    planner_node_launch = roslaunch.parent.ROSLaunchParent(uuid, [PLANNER_PKG_PATH +
        "/launch/planner_node.launch"])
    planner_node_launch.start()

    # start rviz by opening a new terminal in a new tab and running command "rviz"
    subprocess.call('gnome-terminal --tab --execute rviz', shell=True)

    individual_plan_parts = []

    # [x, y, theta (given in deg and coverted to rad)
    path_points =  [
                    [2500, 640, -11.3099 * np.pi / 180.0], # INITIAL Point
                    # orange path
                    [2600, 660, 20.0 * np.pi / 180.0], # REQUIRED Point 1
                    # yellow path
                    [2615, 590, 125.0 * np.pi / 180.0],
                    # [2410, 485, 160 * np.pi / 180.0], # end of yellow path
                    # green path
                    [1965, 400, -150 * np.pi / 180.0], # end of green path
                    # blue path
                    [1880, 440,-150 * np.pi / 180.0], # REQUIRED Point 2
                    # purple path
                    [1660, 490, -150 * np.pi / 180.0], # end of purple path
                    [1550, 600, -171 * np.pi / 180.0],  # end of orange path
                    # orange path
                    [1520, 600, -175 * np.pi / 180.0], # end of yellow
                    [1435, 545, 160 * np.pi / 180.0], # REQUIRED Point 3, end of green
                    [1350, 440, 160 * np.pi / 180.0], # REQUIRED Point 4, end of cyan
                    [1050, 450, -160.0 * np.pi / 180.0], # end of purple
                    [650.0, 650.0, -120 * np.pi / 180.0], # end of violet
                    [540, 835, -120 * np.pi / 180.0] # end of cyan
                    ] # REQUIRED Point 5 (END)

    for i in range(0, len(path_points) - 1):
        rospy.sleep(15)  # wait for planner_node to load or respawnn
        # raw_input("\n\nPRESS ENTER WHEN READY TO PLAN\n\n")
        initial_pose = path_points[i]
        goal_pose = path_points[i + 1]
        individual_plan_parts.append(get_plan(initial_pose, goal_pose, i))
        if i != len(path_points) - 2:
            os.system('rosnode kill planner_node')
        # if planner_node doesn't autorespawn then use this:
        # rospy.sleep(1)
        #     # launch nodes from python
        #     # https://answers.ros.org/question/263862/if-it-possible-to-launch-a-launch-file-from-python/
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # planner_node_launch = roslaunch.parent.ROSLaunchParent(uuid, [PLANNER_PKG_PATH +
        #                                                               "/launch/planner_node.launch"])
        # planner_node_launch.start()



    plan_array = []
    # Read http://docs.ros.org/jade/api/geometry_msgs/html/msg/PoseArray.html
    PA = PoseArray()  # create a PoseArray() msg
    PA.header.stamp = rospy.Time.now()  # set header timestamp value
    PA.header.frame_id = "map"  # set header frame id value
    PA.poses = []
    for i, raw_plan in enumerate(individual_plan_parts):
        print "\nraw_plan ", i, " published of type: ", type(raw_plan)
        for pose in raw_plan.poses:
            plan_array.append(np.array([pose.position.x, pose.position.y, utils.quaternion_to_angle(pose.orientation)]))
            P = Pose()
            P.position.x = pose.position.x
            P.position.y = pose.position.y
            P.position.z = 0
            P.orientation = pose.orientation

            PA.poses.append(P)

    PA_pub = rospy.Publisher(PLAN_POSE_ARRAY_TOPIC, PoseArray, queue_size=1)
    for i in range(0, 5):
        rospy.sleep(0.5)
        PA_pub.publish(PA)
    os.system('rosnode kill planner_node')

    # save plan_array to file
    # save plan PoseArray msg to file
    file_temp = open('/home/tim/car_ws/src/final/saved_plans/plan_12_9_2018', 'w')
    pickle.dump([plan_array, PA], file_temp)
    file_temp.close()

    print "\n\n DONE \n\n"

if __name__ == '__main__':
    main()
