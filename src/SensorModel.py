#!/usr/bin/env python

import numpy as np
import rospy
import range_libc
import time
from threading import Lock
from nav_msgs.srv import GetMap
import rosbag
import matplotlib.pyplot as plt
import utils as Utils
from sensor_msgs.msg import LaserScan
import math


THETA_DISCRETIZATION = 112  # Discretization of scanning angle
INV_SQUASH_FACTOR = 0.2  # Factor for helping the weight distribution to be less peaked

# YOUR CODE HERE (Set these values and use them in precompute_sensor_model)
Z_SHORT = 0.05  # Weight for short reading
Z_MAX = 0.05  # Weight for max reading
Z_RAND = 0.05  # Weight for random reading
SIGMA_HIT = 7  # Noise value for hit reading
Z_HIT = 0.85  # Weight for hit reading


''' 
  Weights particles according to their agreement with the observed data
'''


class SensorModel:
    '''
    Initializes the sensor model
      scan_topic: The topic containing laser scans
      laser_ray_step: Step for downsampling laser scans
      exclude_max_range_rays: Whether to exclude rays that are beyond the max range
      max_range_meters: The max range of the laser
      map_msg: A nav_msgs/MapMetaData msg containing the map to use
      particles: The particles to be weighted
      weights: The weights of the particles
      state_lock: Used to control access to particles and weights
    '''

    def __init__(self, scan_topic, laser_ray_step, exclude_max_range_rays,
                 max_range_meters, map_msg, particles, weights, state_lock=None):
        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        self.particles = particles[:]  # shape (13027200, 3)
        self.weights = weights[:]  # shape (13027200,)

        self.LASER_RAY_STEP = laser_ray_step  # Step for downsampling laser scans (18 by default)
        # Whether to exclude rays that are beyond the max range (true by default)
        self.EXCLUDE_MAX_RANGE_RAYS = exclude_max_range_rays
        self.MAX_RANGE_METERS = max_range_meters  # The max range of the laser (5.6 by default)

        # Get a copy of the map as a type: range_libc.PyOMap
        oMap = range_libc.PyOMap(map_msg)  # A version of the map that range_libc can understand
        # The max range in pixels of the laser (int of value 280)
        max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution)
        # The range method that will be used for ray casting
        self.range_method = range_libc.PyCDDTCast(oMap, max_range_px, THETA_DISCRETIZATION)
        # self.range_method = range_libc.PyRayMarchingGPU(oMap, max_range_px) # The range method that will be used for ray casting
        self.range_method.set_sensor_model(
            self.precompute_sensor_model(max_range_px))  # Load the sensor model expressed as a table
        self.queries = None  # Do not modify this variable
        self.ranges = None  # Do not modify this variable
        self.laser_angles = None  # The angles of each ray
        self.downsampled_angles = None  # The angles of the downsampled rays
        self.do_resample = False  # Set so that outside code can know that it's time to resample

        # Subscribe to laser scansssssssssssssssws
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_cb, queue_size=1)

    '''
      Downsamples laser measurements and applies sensor model
        msg: A sensor_msgs/LaserScan
    '''

    def lidar_cb(self, msg):
        # print "LIDAR CB START"
        self.state_lock.acquire()

        # Compute the observation obs
        #   obs is a a two element tuple
        #   obs[0] is the downsampled ranges
        #   obs[1] is the downsampled angles
        #   Note it should be the case that obs[0].shape[0] == obs[1].shape[0]
        #   Each element of obs must be a numpy array of type np.float32
        #   Use self.LASER_RAY_STEP as the downsampling step
        #   Keep efficiency in mind, including by caching certain things that won't
        #   change across future iterations of this callback
        #   and vectorizing computations as much as possible
        #   Set all range measurements that are NAN or 0.0 to self.MAX_RANGE_METERS
        #   You may choose to use self.laser_angles and self.downsampled_angles here

        # msg type: sensor_msgs.msg._LaserScan.LaserScan
        # msg doc: http://docs.ros.org/jade/api/sensor_msgs/html/msg/LaserScan.html
        # msg.angle_min, msg.angle_max, msg.angle_increment: -2.08621382713, 2.09234976768, 0.00613592332229
        # len(msg.ranges) : 682, so indices of ranges[] are 0 - 681
        downsampled_ranges = np.array(msg.ranges[::self.LASER_RAY_STEP], dtype=np.float32)  # shape (38,)
        # Set all range measurements that are NAN or 0.0 to self.MAX_RANGE_METERS
        downsampled_ranges[np.logical_or(np.isnan(downsampled_ranges), downsampled_ranges == 0)] = self.MAX_RANGE_METERS
        # cache downsampled angles and laser angles on first iteration
        if self.downsampled_angles is None or self.laser_angles is None:
            # angle shape (682,), angle step 0.00613592304672, not a perfect match to msg.angle_increment
            # but good enough, angles_step is unused - just to check how close it is to msg.angle_increment
            self.laser_angles, angles_step = np.linspace(start=msg.angle_min, stop=msg.angle_max, num=len(msg.ranges),
                                                         endpoint=True, retstep=True, dtype=np.float32)
            self.downsampled_angles = np.array(self.laser_angles[::self.LASER_RAY_STEP],
                                               dtype=np.float32)  # shape (38,)
        obs = (downsampled_ranges, self.downsampled_angles)
        # print "obs", obs[0].shape[0], obs[1].shape[0]

        ###
        #  Plot for Testing
        ###

        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        # ax.plot(np.arange(0, len(self.downsampled_angles)), self.downsampled_angles, 'r+', markersize=6,
        #         linewidth=2, label='downsampled angles')
        # ax.plot(np.arange(0, len(downsampled_ranges)), downsampled_ranges, 'bx', markersize=6,
        #         linewidth=2, label='downsampled ranges')
        # ax.set_xlim(0,round(682/self.LASER_RAY_STEP))
        # plt.title('Lidar Scan')
        # ax.legend(loc='best')
        # plt.show()

        # print "self.weights 1", self.weights
        self.apply_sensor_model(self.particles, obs, self.weights)
        # print "self.weights 2", np.min(self.weights), np.max(self.weights)
        self.weights /= np.sum(self.weights)
        # print "self.weights 3", np.min(self.weights), np.max(self.weights)

        self.last_laser = msg
        self.do_resample = True
        self.state_lock.release()
        # print "LIDAR CB COMPLETE - measures lasers and applies sensor model"

    '''
      Compute table enumerating the probability of observing a measurement 
      given the expected measurement
      Element (r,d) of the table is the probability of observing measurement r (in pixels)
      when the expected measurement is d (in pixels)
      max_range_px: The maximum range in pixels
      Returns the table (which is a numpy array with dimensions [max_range_px+1, max_range_px+1]) 
    '''

    def precompute_sensor_model(self, max_range_px):

        table_width = int(max_range_px) + 1
        sensor_model_table = np.zeros((table_width, table_width))

        # Populate sensor_model_table according to the laser beam model specified
        # in CH 6.3 of Probabilistic Robotics
        # Note: no need to use any functions from utils.py to compute between world
        #       and map coordinates here
        # YOUR CODE HERE
        # Pseudo-code
        # for d in xrange(table_width):
        #   possibly some stuff here
        #   for r in xrange(table_width):
        #     Populate the sensor model table at (r,d) with the probability of
        #     observing measurement r (in pixels)
        #     when the expected measurement is d (in pixels)
        # Note that the 'self' parameter is completely unused in this function
        print "Precomputing sensor model"

        for d in xrange(table_width): # d is [ztk*] the true distance in the range [0 - max range in pixels]
            for r in xrange(table_width): # r is [ztk] the measured distance in the range [0 - max range in pixels]
                # See probroboticsBook-ch1-8.pdf 6.3.1

                # 1. Correct range with local measurement noise
                if 0 <= r and r <= max_range_px:
                    p_hit = (1 / math.sqrt(2 * math.pi * math.pow(SIGMA_HIT, 2)) *
                             math.exp(-0.5 * math.pow((r - d), 2) / math.pow(SIGMA_HIT, 2))) # normal distribution
                else:
                    p_hit = 0

                # 2. Unexpected objects
                if 0 <= r and r <= d:
                    v_exp_mult= 0.7 # vertical expansion multiple
                    h_comp_mult = 0.03 # horizonal compression multiple
                    p_short = v_exp_mult * math.exp(-h_comp_mult * r) # decaying exponential distribution
                else:
                    p_short = 0

                # 3. Failures
                if r == max_range_px:
                    p_max = 1 # 100% chance that 0s and NaNs will be at the max range because that's what we set them to
                else:
                    p_max = 0

                # 4. Random measurements
                if 0 <= r and r < max_range_px:
                    p_rand = 1.0 / max_range_px # uniform distribution
                else:
                    p_rand = 0

                probability = Z_HIT * p_hit + Z_SHORT * p_short + Z_MAX * p_max + Z_RAND * p_rand
                sensor_model_table[r, d] = probability

        # normalize - divide all values by the sum of their respective columns
        sensor_model_table /= np.sum(sensor_model_table, axis=0)

        # print "self.sensor_model_table 1", sensor_model_table

        return sensor_model_table


    '''
      Updates the particle weights in-place based on the observed laser scan
        proposal_dist: The particles
        obs: The most recent observation
        weights: The weights of each particle
    '''


    def apply_sensor_model(self, proposal_dist, obs, weights):
        # print "applying sensor model"
        obs_ranges = obs[0]
        obs_angles = obs[1]
        num_rays = obs_angles.shape[0]

        # Only allocate buffers once to avoid slowness
        if not isinstance(self.queries, np.ndarray):
            self.queries = np.zeros((proposal_dist.shape[0], 3), dtype=np.float32)
            self.ranges = np.zeros(num_rays * proposal_dist.shape[0], dtype=np.float32)

        self.queries[:, :] = proposal_dist[:, :]

        # Raycasting to get expected measurements
        self.range_method.calc_range_repeat_angles(self.queries, obs_angles, self.ranges)

        # Evaluate the sensor model
        self.range_method.eval_sensor_model(obs_ranges, self.ranges, weights, num_rays, proposal_dist.shape[0])

        # # Squash weights to prevent too much peakiness
        np.power(weights, INV_SQUASH_FACTOR, weights)

        # print "done applying sensor model"

'''
  Code for testing SensorModel
'''

MAP_TOPIC = 'static_map'


def main():
    rospy.init_node("sensor_model", anonymous=True)  # Initialize the node

    bag_path = rospy.get_param("~bag_path", '/home/tim/car_ws/src/lab2/bags/laser_scans/laser_scan1.bag')
    scan_topic = rospy.get_param("~scan_topic", "/scan")  # The topic containing laser scans
    laser_ray_step = int(rospy.get_param("~laser_ray_step"))  # Step for downsampling laser scans (18 by default)
    # Whether to exclude rays that are beyond the max range (true by default)
    exclude_max_range_rays = bool(rospy.get_param("~exclude_max_range_rays"))
    max_range_meters = float(rospy.get_param("~max_range_meters"))  # The max range of the laser (5.6 by default)

    print 'Bag path: ' + bag_path

    # Use the 'static_map' service (launched by MapServer.launch) to get the map
    print("Getting map from service: ", MAP_TOPIC)
    rospy.wait_for_service(MAP_TOPIC)
    map_msg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map  # The map, will get passed to init of sensor model
    map_info = map_msg.info  # Save info about map for later use

    # map_info example:
    # map_load_time:
    #     secs: 1541298847
    #     nsecs: 661081098
    # resolution: 0.019999999553
    # width: 3200
    # height: 3200
    # origin:
    #     position:
    #         x: -34.92
    #         y: -33.64
    #         z: 0.0
    #     orientation:
    #         x: 0.0
    #         y: 0.0
    #         z: 0.0
    #         w: 1.0

    print 'Creating permissible region'
    # Create numpy array representing map for later use
    # By default map has values 0: permissible, -1: unmapped, 100: blocked
    array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))  # shape (3200, 3200)
    permissible_region = np.zeros_like(array_255, dtype=bool)  # zeros - shape (3200, 3200)
    # Set the values of the permissible region to 1 and everything else remains 0s
    permissible_region[array_255 == 0] = 1  # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
    # With values 0: not permissible, 1: permissible
    permissible_x, permissible_y = np.where(permissible_region == 1)  # both shapes (521088,)

    # Potentially downsample permissible_x and permissible_y here

    print 'Creating particles'
    angle_step = 25
    particles = np.zeros((angle_step * permissible_x.shape[0], 3))  # particles.shape (13027200, 3)
    for i in xrange(angle_step):
        particles[i * (particles.shape[0] / angle_step):(i + 1) * (particles.shape[0] / angle_step), 0] = permissible_y[
                                                                                                          :]
        particles[i * (particles.shape[0] / angle_step):(i + 1) * (particles.shape[0] / angle_step), 1] = permissible_x[
                                                                                                          :]
        particles[i * (particles.shape[0] / angle_step):(i + 1) * (particles.shape[0] / angle_step), 2] = i * (
                2.0 * np.pi / angle_step)

    Utils.map_to_world(particles, map_info)
    weights = np.ones(particles.shape[0]) / float(particles.shape[0])  # weights.shape (13027200,)

    print 'Initializing sensor model'
    sm = SensorModel(scan_topic, laser_ray_step, exclude_max_range_rays,
                     max_range_meters, map_msg, particles, weights)

    # Give time to get setup
    rospy.sleep(1.0)

    # Load laser scan from bag
    bag = rosbag.Bag(bag_path)
    for _, msg, _ in bag.read_messages(topics=['/scan']):
        laser_msg = msg
        break

    w_min = np.amin(weights)
    w_max = np.amax(weights)

    pub_laser = rospy.Publisher(scan_topic, LaserScan, queue_size=1)  # Publishes the most recent laser scan
    print("Starting analysis, this could take awhile...")
    while not isinstance(sm.queries, np.ndarray):
        pub_laser.publish(laser_msg)
        rospy.sleep(1.0)

    rospy.sleep(1.0)  # Make sure there's enough time for laserscan to get lock

    print 'Going to wait for sensor model to finish'
    sm.state_lock.acquire()
    print 'Done, preparing to plot'
    print "weights before", weights
    weights = weights.reshape((angle_step, -1))
    print "weights after", weights
    weights = np.amax(weights, axis=0)
    print map_msg.info.height
    print map_msg.info.width
    print weights.shape
    w_min = np.amin(weights)
    w_max = np.amax(weights)
    print 'w_min = %f' % w_min
    print 'w_max = %f' % w_max
    weights = 0.9 * (weights - w_min) / (w_max - w_min) + 0.1

    img = np.zeros((map_msg.info.height, map_msg.info.width))
    for i in xrange(len(permissible_x)):
        img[permissible_y[i], permissible_x[i]] = weights[i]
    plt.imshow(img)
    plt.show()


if __name__ == '__main__':
    main()
