#!/usr/bin/env python

import pickle
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Pose


import utils

rospack = rospkg.RosPack()
RACECAR_PKG_PATH = rospack.get_path('racecar')
PLANNER_PKG_PATH = rospack.get_path('planning_utils')
CURRENT_PKG_PATH = rospack.get_path('final')

# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PUB_TOPIC_2 = '/plan_lookahead_follower/pose' # to publish plan lookahead follower to assist with troubleshooting
INIT_POSE_TOPIC = "/initialpose"
WINDOW_WIDTH = 5


def main():

    rospy.init_node('line_follower', anonymous=True)  # Initialize the node

    plan_relative_path = "/saved_plans/plan2"
    # load plan_array
    # load raw_plan msg (PoseArray)
    loaded_vars = pickle.load(open(CURRENT_PKG_PATH + plan_relative_path, "r"))
    plan_array = loaded_vars[0]
    # raw_plan = loaded_vars[1]

    plan_array_new = []

    total_distance = 0.0
    for i in range(len(plan_array) - 1):
        total_distance += np.sqrt(np.square(plan_array[i][0] - plan_array[i+1][0]) + np.square(plan_array[i][1] - plan_array[i+1][1]))

    avg_distance = total_distance / len(plan_array)

    array_len = len(plan_array)
    i = 0
    while i < array_len - 1:
        current_distance = np.sqrt(np.square(plan_array[i][0] - plan_array[i+1][0]) + np.square(plan_array[i][1] - plan_array[i+1][1]))
        if current_distance < avg_distance * 0.5:
            plan_array_new.append(plan_array.pop(i+1))
            i -= 1
            array_len -= 1
        i += 1

    PA = PoseArray()  # create a PoseArray() msg
    PA.header.stamp = rospy.Time.now()  # set header timestamp value
    PA.header.frame_id = "map"  # set header frame id value
    PA.poses = []
    for pose in plan_array:
        P = Pose()
        P.position.x = float(pose[0])
        P.position.y = float(pose[1])
        P.position.z = 0
        P.orientation = utils.angle_to_quaternion(float(pose[2]))

        PA.poses.append(P)


    # visualize edited loaded plan
    PA_pub = rospy.Publisher("/LoadedPlan", PoseArray, queue_size=1)
    for i in range(0, 5):
        rospy.sleep(0.5)
        PA_pub.publish(PA)

    # save plan_array to file
    # save plan PoseArray msg to file
    file_temp = open(CURRENT_PKG_PATH + plan_relative_path + "_clean", 'w')
    pickle.dump([plan_array_new, PA], file_temp)
    file_temp.close()

if __name__ == '__main__':
    main()
