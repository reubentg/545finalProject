#!/usr/bin/env python


import pickle
import rospkg

rospack = rospkg.RosPack()
RACECAR_PKG_PATH = rospack.get_path('racecar')
PLANNER_PKG_PATH = rospack.get_path('planning_utils')
CURRENT_PKG_PATH = rospack.get_path('final')
BLUE_FILTER_TOPIC = '/cv_node/blue_data'
RED_FILTER_TOPIC = '/cv_node/red_data'

import collections
import math
import time

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64



import utils

# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PUB_TOPIC_2 = '/plan_lookahead_follower/pose'  # to publish plan lookahead follower to assist with troubleshooting
WINDOW_WIDTH = 5
INIT_POSE_TOPIC = "/initialpose"

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''


class LineFollower:
    """
    Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
    """

    def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
                 rotation_weight, kp, ki, kd, error_buff_length, speed):
        # print "inside line_follower, constructor"
        # Store the passed parameters
        self.plan = plan
        self.plan_lookahead = plan_lookahead
        # Normalize translation and rotation weights
        self.translation_weight = translation_weight / (translation_weight + rotation_weight)
        self.rotation_weight = rotation_weight / (translation_weight + rotation_weight)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # The error buff stores the error_buff_length most recent errors and the
        # times at which they were received. That is, each element is of the form
        # [time_stamp (seconds), error]. For more info about the data struct itself, visit
        # https://docs.python.org/2/library/collections.html#collections.deque
        self.error_buff = collections.deque(maxlen=error_buff_length)
        self.speed = speed
        self.found_closest_point = False
        self.total_error_list = []

        self.angle_from_computer_vision = None
        # # print "line_follower Initialized!"
        # # print "plan[0]", self.plan[0]
        # # print "plan[plan_lookahead]", self.plan[plan_lookahead]
        # # print "error_buff length: ", len(self.error_buff)
        # # print "error_buff: ", self.error_buff

        # YOUR CODE HERE
        self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped,
                                       queue_size=10)  # Create a publisher to PUB_TOPIC
        self.goal_pub = rospy.Publisher(PUB_TOPIC_2, PoseStamped,
                                        queue_size=10)  # create a publisher for plan lookahead follower

        self.delete_pose_pub = rospy.Publisher("MaybeDelete", PoseStamped,
                                        queue_size=10)  # create a publisher for plan lookahead follower


        self.robot = rospy.Publisher("Robot", PoseStamped,
                                        queue_size=10)  # create a publisher for plan lookahead follower


        self.deleted = rospy.Publisher("Deleted", PoseStamped,
                                        queue_size=10)  # create a publisher for plan lookahead follower

        self.float_pub = rospy.Publisher("angle_from_line_follower", Float64, queue_size=1)

        self.selected_pub = rospy.Publisher("Selected", PoseStamped,
                                            queue_size=1)  # create a publisher to visualize some pose from selected rollout

        self.line_follower_angle_pub = rospy.Publisher("LineFollowerAngle", PoseStamped,
                                                       queue_size=1)  # create a publisher to visualize some pose from selected rollout

        self.float_blue_sub = rospy.Subscriber(BLUE_FILTER_TOPIC, Float64, self.float_cb_blue)
        self.float_red_sub = rospy.Subscriber(RED_FILTER_TOPIC, Float64, self.float_cb_red)

        # Create a publisher to publish the initial pose
        init_pose_pub = rospy.Publisher(INIT_POSE_TOPIC, PoseWithCovarianceStamped,
                                        queue_size=1)  # to publish init position x=2500, y=640
        PWCS = PoseWithCovarianceStamped()  # create a PoseWithCovarianceStamped() msg
        PWCS.header.stamp = rospy.Time.now()  # set header timestamp value
        PWCS.header.frame_id = "map"  # set header frame id value

        PWCS.pose.pose.position.x = plan[0][0]
        PWCS.pose.pose.position.y = plan[0][1]
        PWCS.pose.pose.position.z = 0
        PWCS.pose.pose.orientation = utils.angle_to_quaternion(plan[0][2])
        for i in range(0, 1):
            rospy.sleep(0.5)
            init_pose_pub.publish(
                PWCS)  # publish initial pose, now you can add a PoseWithCovariance with topic of "/initialpose" in rviz


        # Create a subscriber to pose_topic, with callback 'self.pose_cb'
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)
        # print "inside line_follower, constructor end"

        self.new_init_pos = rospy.Subscriber(INIT_POSE_TOPIC, PoseWithCovarianceStamped, self.new_init_pose_cb)

    def float_cb_red(self, msg):
        pass

    def float_cb_blue(self, msg):
        # print "BLUE cb", msg.data
        self.angle_from_computer_vision = msg.data

    def new_init_pose_cb(self, msg):
        if len(self.plan) > 0:

            rot_mat = utils.rotation_matrix(-1 * self.curr_pose[2])
            while len(self.plan) > 0:
                distance = np.sqrt(np.square(self.curr_pose[0] - self.plan[0][0]) + np.square(self.curr_pose[1] - self.plan[0][1]))
                # Figure out if self.plan[0] is in front or behind car
                offset = rot_mat * ((self.plan[0][0:2] - self.curr_pose[0:2]).reshape(2, 1))
                offset.flatten()
                if offset[0] > 0.0 or distance > 1.0:
                    break
                self.plan.pop(0)
    '''
    Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
    Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
    '''


    def compute_error(self, cur_pose):
        """
        Find the first element of the plan that is in front of the robot, and remove
        any elements that are behind the robot. To do this:
        Loop over the plan (starting at the beginning) For each configuration in the plan
            If the configuration is behind the robot, remove it from the plan
              Will want to perform a coordinate transformation to determine if
              the configuration is in front or behind the robot
            If the configuration is in front of the robot, break out of the loop
        """
        # # print "Computing error..."
        # check the leftmost pose in the plan pose-array and if it is behind the car then delete it
        if len(self.plan) > 0:

            rot_mat = utils.rotation_matrix(-1 * cur_pose[2])
            while len(self.plan) > 0:
                distance = np.sqrt(np.square(cur_pose[0] - self.plan[0][0]) + np.square(cur_pose[1] - self.plan[0][1]))
                # Figure out if self.plan[0] is in front or behind car
                offset = rot_mat * ((self.plan[0][0:2] - cur_pose[0:2]).reshape(2, 1))
                offset.flatten()
                if offset[0] > 0.0 or distance > 1.0:
                    break
                self.plan.pop(0)
            # left_edge = (cur_pose[2] + np.pi / 2) * 180 / 3.14  # deg
            # right_edge = (cur_pose[2] - np.pi / 2) * 180 / 3.14  # deg
            # # if cur_pose[2] < 0:
            # #     left_edge += 360.0
            # #     right_edge += 360.0
            # angle_robot_path_point = math.atan2(cur_pose[1] - self.plan[0][1],
            #                                     cur_pose[0] - self.plan[0][0]) * 180 / 3.14  # deg
            #
            #
            # # for troubleshooting if path points are not deleted correctly
            # # converted angles from rad to deg for easier troubleshooting
            # # # # print("robot position: ", cur_pose)
            # # # # print("path point position: ", self.plan[0])
            # # # # print("left_edge: ", left_edge)
            # # # # print("right_edge: ", right_edge)
            # # # # print("path point to robot vector: ",cur_pose[1] - self.plan[0][1], cur_pose[0] - self.plan[0][0])
            # # # # print("angle of path point to robot vector",angle_robot_path_point)
            # # # # print("path_point yaw",self.plan[0][2] * 180 / 3.14)
            #
            #
            # # if left_edge <= 0:
            # #     left_edge += 2.0 * np.pi
            # #     if angle_robot_path_point <= 0:
            # #         angle_robot_path_point += 2.0 * np.pi
            #
            # behind = (angle_robot_path_point > right_edge and angle_robot_path_point < left_edge)  # is path point behind robot?
            #
            # # print cur_pose, behind , left_edge, angle_robot_path_point, right_edge
            #
            # PS = PoseStamped()  # create a PoseStamped() msg
            # PS.header.stamp = rospy.Time.now()  # set header timestamp value
            # PS.header.frame_id = "map"  # set header frame id value
            # PS.pose.position.x = self.plan[0][0]  # set msg x position to value of the x position in the look ahead pose from the path
            # PS.pose.position.y = self.plan[0][1]  # set msg y position to value of the y position in the look ahead pose from the path
            # PS.pose.position.z = 0  # set msg z position to 0 since robot is on the ground
            # PS.pose.orientation = utils.angle_to_quaternion(self.plan[0][2])  # set msg orientation to [converted to queternion] value of the yaw angle in the look ahead pose from the path
            #
            # self.delete_pose_pub.publish(
            #     PS)  # publish look ahead follower, now you can add a Pose with topic of PUB_TOPIC_2 value in rviz
            #
            #
            #
            # path_pose_similar_direction = (self.plan[0][2] > right_edge and self.plan[0][
            #     2] < left_edge)  # is path point in similar direction as robot?
            # path_pose_similar_direction = True
            # if behind and path_pose_similar_direction and len(
            #         self.plan) > 0:  # delete point if behind robot, similar direction, and not last point in path
            #     # # print "delete element: ", len(self.plan)  # for troubleshooting, show path points before deleting
            #
            #     PS = PoseStamped()  # create a PoseStamped() msg
            #     PS.header.stamp = rospy.Time.now()  # set header timestamp value
            #     PS.header.frame_id = "map"  # set header frame id value
            #     PS.pose.position.x = self.plan[0][
            #         0]  # set msg x position to value of the x position in the look ahead pose from the path
            #     PS.pose.position.y = self.plan[0][
            #         1]  # set msg y position to value of the y position in the look ahead pose from the path
            #     PS.pose.position.z = 0  # set msg z position to 0 since robot is on the ground
            #     PS.pose.orientation = utils.angle_to_quaternion(self.plan[0][
            #                                                         2])  # set msg orientation to [converted to queternion] value of the yaw angle in the look ahead pose from the path
            #
            #     self.deleted.publish(
            #         PS)
            #
            #     PS = PoseStamped()  # create a PoseStamped() msg
            #     PS.header.stamp = rospy.Time.now()  # set header timestamp value
            #     PS.header.frame_id = "map"  # set header frame id value
            #     PS.pose.position.x = cur_pose[0]
            #     PS.pose.position.y = cur_pose[1]
            #     PS.pose.position.z = 0  # set msg z position to 0 since robot is on the ground
            #     PS.pose.orientation = utils.angle_to_quaternion(cur_pose[2])  # set msg orientation to [converted to queternion] value of the yaw angle in the look ahead pose from the path
            #     # print "CURR POSE", cur_pose, "DELETED", self.plan[0][:]
            #     self.robot.publish(
            #         PS)
            #
            #
            #
            #
            #
            #     self.plan.pop(
            #         0)  # delete the first element in the path, since that point is behind robot and it's direction is similar to robot
            #     # # print "element deleted? : ", len(self.plan)  # for troubleshooting, show path points after deleting

            if len(self.plan) > 0:
                PS = PoseStamped()  # create a PoseStamped() msg
                PS.header.stamp = rospy.Time.now()  # set header timestamp value
                PS.header.frame_id = "map"  # set header frame id value
                goal_idx = min(0 + self.plan_lookahead,
                               len(self.plan) - 1)  # get goal index for looking ahead this many indices in the path
                PS.pose.position.x = self.plan[goal_idx][
                    0]  # set msg x position to value of the x position in the look ahead pose from the path
                PS.pose.position.y = self.plan[goal_idx][
                    1]  # set msg y position to value of the y position in the look ahead pose from the path
                PS.pose.position.z = 0  # set msg z position to 0 since robot is on the ground
                PS.pose.orientation = utils.angle_to_quaternion(self.plan[goal_idx][
                                                                    2])  # set msg orientation to [converted to queternion] value of the yaw angle in the look ahead pose from the path

                self.goal_pub.publish(
                    PS)  # publish look ahead follower, now you can add a Pose with topic of PUB_TOPIC_2 value in rviz

        # Check if the plan is empty. If so, return (False, 0.0)
        # YOUR CODE HERE
        if len(self.plan) == 0:
            return False, 0.0

        # At this point, we have removed configurations from the plan that are behind
        # the robot. Therefore, element 0 is the first configuration in the plan that is in
        # front of the robot. To allow the robot to have some amount of 'look ahead',
        # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
        # We call this index the goal_index
        goal_idx = min(0 + self.plan_lookahead, len(self.plan) - 1)

        # Compute the translation error between the robot and the configuration at goal_idx in the plan
        # YOUR CODE HERE
        # # print "cur_pose: ", cur_pose
        # # print "lookahead pose: ", self.plan[goal_idx]
        look_ahead_position = np.array([self.plan[goal_idx][0], self.plan[goal_idx][1]]).reshape([2, 1])
        translation_robot_to_origin = np.array([-cur_pose[0], -cur_pose[1]]).reshape([2, 1])
        look_ahead_position_translated = look_ahead_position + translation_robot_to_origin
        rotation_matrix_robot_to_x_axis = utils.rotation_matrix(-cur_pose[2])
        look_ahead_position_translated_and_rotated = rotation_matrix_robot_to_x_axis * look_ahead_position_translated
        # # print "look_ahead_position_translated_and_rotated: ", look_ahead_position_translated_and_rotated
        x_error = float(look_ahead_position_translated_and_rotated[0][
                            0])  # This is the distance that the robot is behind the lookahead point parallel to the path
        y_error = float(look_ahead_position_translated_and_rotated[1][
                            0])  # This is the distance away from the path, perpendicular from the path to the robot
        translation_error = -y_error  # math.tan(y_error / x_error) * math.pi / 180 # angle in rad to drive along hypotenuse toward the look ahead point
        # translation_error *= 10 #float(y_error/x_error) # make the robot turn more sharply if far away from path

        # translation_error = np.sqrt(np.square(cur_pose[0] - self.plan[goal_idx][0]) + np.square(cur_pose[1] - self.plan[goal_idx][1]))

        # # print "Translation error: ", translation_error

        # Compute the total error
        # Translation error was computed above
        # Rotation error is the difference in yaw between the robot and goal configuration
        #   Be careful about the sign of the rotation error
        # YOUR CODE HERE
        rotation_error = cur_pose[2] - self.plan[goal_idx][2]
        if rotation_error > np.pi:
            rotation_error -= np.pi*2.0
        elif rotation_error < -np.pi:
            rotation_error += np.pi*2.0
        #ToDo: Fix rotation error when moving right to left, it only calculates correctly left to right
        # # print "Rotation error: ", rotation_error

        error = self.translation_weight * translation_error + self.rotation_weight * rotation_error
        # # print "Overall error: ", error
        self.total_error_list.append(error)

        return True, error


    '''
    Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
    Returns: The steering angle that should be executed
    '''


    def compute_steering_angle(self, error):
        # # print "Computing steering angle..."
        now = rospy.Time.now().to_sec()  # Get the current time

        # Compute the derivative error using the passed error, the current time,
        # the most recent error stored in self.error_buff, and the most recent time
        # stored in self.error_buff
        # YOUR CODE HERE

        deriv_error = 0  # for the first iteration, this is true
        integ_error = 0
        # # print "setting deriv and integ error to 0"
        # # print "error_buff len", len(self.error_buff)
        if len(self.error_buff) > 0:
            time_delta = now - self.error_buff[-1][1]  # -1 means peeking the rightmost element (most recent)
            error_delta = error - self.error_buff[-1][0]

            deriv_error = error_delta / time_delta
            # # print "computed deriv error: ", deriv_error

        # Add the current error to the buffer
        self.error_buff.append((error, now))

        # Compute the integral error by applying rectangular integration to the elements
        # of self.error_buff:
        # ://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
        # YOUR CODE HERE
        error_array = []
        if len(self.error_buff) > 0:
            for err in self.error_buff:
                error_array.append(err[0])
            integ_error = np.trapz(error_array)
            # # print "computed integ error: ", integ_error

        # Compute the steering angle as the sum of the pid errors
        # YOUR CODE HERE


        return -(self.kp * error + self.ki * integ_error + self.kd * deriv_error)


    '''
    Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
    '''


    def pose_cb(self, msg):
        # print "inside line_follower ,pose_cb"
        time.sleep(0)
        # # print "Callback received current pose. "
        cur_pose = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             utils.quaternion_to_angle(msg.pose.orientation)])
        # print "Current pose: ", cur_pose

        # # # # print "plan[:,[0,1]]", type(np.array(self.plan)), np.array(self.plan)[:,[0,1]]
        # # find closest point and delete all points before it in the plan
        # # only done once at the start of following the plan
        # if self.found_closest_point == False:
        #     min_path_distance = np.Infinity  # to find closest path point and delete all points before it
        #     for count, position in enumerate(np.array(self.plan)[:, [0, 1]]):
        #         distance = np.sqrt(np.square(cur_pose[0] - position[0]) + np.square(cur_pose[1] - position[1]))
        #         if distance < min_path_distance:
        #             self.found_closest_point = True
        #             min_path_distance = distance
        #             if count > 0:
        #                 self.plan.pop(0)

        success, error = self.compute_error(cur_pose)
        # print "Success, Error: ", success, error

        if not success:
            # We have reached our goal
            self.pose_sub = None  # Kill the subscriber
            self.speed = 0.0  # Set speed to zero so car stops
            if False: # show error plot?
                # plot the error here
                title_string = "Error plot with kp=%.2f, kd=%.2f, ki=%.2f t_w=%.2f r_w=%.2f" % \
                               (self.kp, self.kd, self.ki, self.translation_weight, self.rotation_weight)

                fig = plt.figure()
                ax = fig.add_subplot(111)  #
                ax.plot(self.total_error_list)
                plt.title(title_string)
                plt.text(0.5, 0.85, 'Total error = %.2f' % np.trapz(abs(np.array(self.total_error_list))),
                         horizontalalignment='center',
                         verticalalignment='center', transform=ax.transAxes)
                plt.xlabel('Iterations')
                plt.ylabel('Error')
                plt.show()

                np.savetxt("/home/joe/Desktop/Error_1.csv", np.array(self.total_error_list), delimiter=",")

            return 0
        # if computer vision angle is published then use that angle
        # if self.angle_from_computer_vision is not None and self.angle_from_computer_vision > -98.0:
        if True:
            delta = self.angle_from_computer_vision
            print "CV ANGLE: ", delta
        else:   # if computer vision angle is not published then use pid controller angle
            delta = self.compute_steering_angle(error)
            print "PID ANGLE:", delta

        # print "delta is %f" % delta
        #
        if True: # not using laser_wanderer_robot.launch
            # Setup the control message
            ads = AckermannDriveStamped()
            ads.header.frame_id = '/map'
            ads.header.stamp = rospy.Time.now()
            ads.drive.steering_angle = delta
            ads.drive.speed = self.speed
            self.cmd_pub.publish(ads)


        # Send the control message to laser_wanderer_robot.launch
        else:
            float_msg = Float32()
            float_msg.data = delta
            self.float_pub.publish(float_msg)


def main():
    rospy.init_node('line_follower', anonymous=True)  # Initialize the node
    """
    Load these parameters from launch file
    We provide suggested starting values of params, but you should
    tune them to get the best performance for your system
    Look at constructor of LineFollower class for description of each var
    'Default' values are ones that probably don't need to be changed (but you could for fun)
    'Starting' values are ones you should consider tuning for your system
    """
    # YOUR CODE HERE
    plan_topic = rospy.get_param('~plan_topic')  # Default val: '/planner_node/car_plan'
    pose_topic = rospy.get_param('~pose_topic')  # Default val: '/sim_car_pose/pose'
    if True: # if on robot? else in rviz
        pose_topic = "/pf/viz/inferred_pose"
    plan_lookahead = rospy.get_param('~plan_lookahead')  # Starting val: 5
    translation_weight = rospy.get_param('~translation_weight')  # Starting val: 1.0
    rotation_weight = rospy.get_param('~rotation_weight')  # Starting val: 0.0
    kp = rospy.get_param('~kp')  # Startinig val: 1.0
    ki = rospy.get_param('~ki')  # Starting val: 0.0
    kd = rospy.get_param('~kd')  # Starting val: 0.0
    error_buff_length = rospy.get_param('~error_buff_length')  # Starting val: 10
    speed = rospy.get_param('~speed')  # Default val: 1.0

    loadFinalPlan = True # set this to True to use preexisting plan instead of creating a new one in RVIZ
    if not loadFinalPlan: # make new plan in RVIZ by setting initial and goal poses
        raw_input("Press Enter to when plan available...")  # Waits for ENTER key press

        # Use rospy.wait_for_message to get the plan msg
        # Convert the plan msg to a list of 3-element numpy arrays
        #     Each array is of the form [x,y,theta]
        # Create a LineFollower object

        raw_plan = rospy.wait_for_message(plan_topic, PoseArray)

        # raw_plan is a PoseArray which has an array of geometry_msgs/Pose called poses

        plan_array = []

        for pose in raw_plan.poses:
            plan_array.append(np.array([pose.position.x, pose.position.y, utils.quaternion_to_angle(pose.orientation)]))
    else: # use preexisting plan from plan_creator.launch and plan_cleanup.launch
        # # # print "Len of plan array: %d" % len(plan_array)
        # # # # print plan_array
        plan_relative_path = "/saved_plans/plan4"
        # load plan_array
        # load raw_plan msg (PoseArray)
        loaded_vars = pickle.load(open(CURRENT_PKG_PATH + plan_relative_path, "r"))
        plan_array = loaded_vars[0]
        raw_plan = loaded_vars[1]

    # visualize loaded plan
    PA_pub = rospy.Publisher("/LoadedPlan", PoseArray, queue_size=1)
    for i in range(0, 5):
        rospy.sleep(0.5)
        PA_pub.publish(raw_plan)

    # # print "LoadedPlan Published"
    try:
        if raw_plan:
            pass
    except rospy.ROSException:
        exit(1)

    lf = LineFollower(plan_array, pose_topic, plan_lookahead, translation_weight,
                      rotation_weight, kp, ki, kd, error_buff_length, speed)  # Create a Line follower

    rospy.spin()  # Prevents node from shutting down


if __name__ == '__main__':
    main()
