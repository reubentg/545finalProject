#!/usr/bin/env python

import rospy
import numpy as np
import utils as Utils
from std_msgs.msg import Float64
from threading import Lock
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
import matplotlib.pyplot as plt
# helper function created to get gaussian samples
from get_nrand_samples import get_nrand_samples
# helper function created to model the resulting pose [x, y, theta] one iteration
# into the future after applying control [speed, angle] to an initial pose [x, y, theta]
from kinematic_model_step import kinematic_model_step

# Set these values and use them in motion_cb
KM_V_NOISE = 0.04  # Kinematic car velocity noise std dev
KM_DELTA_NOISE = 0.12  # Kinematic car delta noise std dev
KM_X_FIX_NOISE = 0.03  # Kinematic car x position constant noise std dev
KM_Y_FIX_NOISE = 0.03  # Kinematic car y position constant noise std dev
KM_THETA_FIX_NOISE = 0.03  # Kinematic car theta constant noise std dev

# Set this value to max amount of particles to start with
MAX_PARTICLES = 1000

'''
  Propagates the particles forward based on the velocity and steering angle of the car
'''
class KinematicMotionModel:
    '''
      Initializes the kinematic motion model
        motor_state_topic: The topic containing motor state information (controls speed)
        servo_state_topic: The topic containing servo state information (controls angle)
        speed_to_erpm_offset: Offset conversion param from rpm to speed (to convert speed to rpm)
        speed_to_erpm_gain: Gain conversion param from rpm to speed (to convert speed to rpm)
        steering_angle_to_servo_offset: Offset conversion param from servo position to steering angle
            (to convert angle to servo position)
        steering_angle_to_servo_gain: Gain conversion param from servo position to steering angle
            (to convert angle to servo position)
        car_length: The length of the car (0.33)
        particles: The particles to propagate forward (1000 particles each with pose [x, y, theta]
        state_lock: Controls access to particles (to prevent multiple threads from using particles
            at the same time)
    '''

    def __init__(self, motor_state_topic, servo_state_topic, speed_to_erpm_offset,
                 speed_to_erpm_gain, steering_to_servo_offset,
                 steering_to_servo_gain, car_length, particles, state_lock=None):
        self.last_servo_cmd = None  # The most recent servo command
        self.last_vesc_stamp = None  # The time stamp from the previous vesc state msg
        self.particles = particles[:] # [1000 x 3] , 1000 particles each with pose [x, y, theta], set to 0s
        self.SPEED_TO_ERPM_OFFSET = speed_to_erpm_offset  # Offset conversion param from rpm to speed
        self.SPEED_TO_ERPM_GAIN = speed_to_erpm_gain  # Gain conversion param from rpm to speed
        self.STEERING_TO_SERVO_OFFSET = steering_to_servo_offset  # Offset conversion param from servo position to steering angle
        self.STEERING_TO_SERVO_GAIN = steering_to_servo_gain  # Gain conversion param from servo position to steering angle
        self.CAR_LENGTH = car_length  # The length of the car
        self.count = 0 # for testing

        # This just ensures that two different threads are not changing the particles
        # array at the same time. You should not have to deal with this.
        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        # This subscriber just caches the most recent servo position command (angle command)
        self.servo_pos_sub = rospy.Subscriber(servo_state_topic, Float64,
                                              self.servo_cb, queue_size=1)
        # Subscribe to the state of the vesc (speed command)
        self.motion_sub = rospy.Subscriber(motor_state_topic, VescStateStamped,
                                           self.motion_cb, queue_size=1)

    '''
      Caches the most recent servo command
        msg: A std_msgs/Float64 message
        
        Definition
         float64 data
    '''
    def servo_cb(self, msg):
        self.last_servo_cmd = msg.data  # Update servo command


    '''
      Converts messages to controls and applies the kinematic car model to the
      particles
        msg: a vesc_msgs/VescStateStamped message
        
        Definition
         Header  header
         VescState state
         
        Example
         header: 
          seq: 112263
          stamp: 
            secs: 1541215450
            nsecs: 601167917
          frame_id: ''
         state: 
          voltage_input: 0.0
          temperature_pcb: 0.0
          current_motor: 0.0
          current_input: 0.0
          speed: 0.0
          duty_cycle: 0.0
          charge_drawn: 0.0
          charge_regen: 0.0
          energy_drawn: 0.0
          energy_regen: 0.0
          displacement: 0.0
          distance_traveled: 0.0
          fault_code: 0
    '''
    def motion_cb(self, msg):
        # This function will return at the beginning the very first time it is called
        # This is because we need at least two calls to measure the change in time
        # and we need to ensure the angle has been published by servo_pub in main()
        # and received by our subscriber self.servo_pos_sub and stored in self.last_servo_cmd
        self.state_lock.acquire()
        if self.last_servo_cmd is None:
            self.state_lock.release()
            return
        if self.last_vesc_stamp is None:
            self.last_vesc_stamp = msg.header.stamp
            self.state_lock.release()
            return

        # At this point the motion_cb function has been called at least 2 times
        # and the timestamp from the previous call is stored in self.last_vesc_stamp
        # and the timestamp from the current call is stored in msg.header.stamp
        # DT (change in time) is current timestamp - previous timestamp
        msg_dt = (msg.header.stamp.to_sec() - self.last_vesc_stamp.to_sec()) # TODO: Find out why this is sometimes negative
        self.count += 1
        # msg_dt *= 0.000000001 # convert nsecs to secs
        # if msg.header.stamp.nsecs < self.last_vesc_stamp.nsecs: # current timestamp resets at 1,000,000,000 nsecs
        #    msg_dt = msg.header.stamp.nsecs + 1000000000.0 - self.last_vesc_stamp.nsecs

        # print "msg_dt #", self.count, "current", msg.header.stamp.to_sec(), "last", self.last_vesc_stamp.to_sec(), ", dt:", msg_dt

        # Convert raw msgs to controls
        # Note that control_val = (raw_msg_val - offset_param) / gain_param
        # E.g: curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
        curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
        curr_angle = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
        # curr_angle = -1 * (np.pi / 180)

        # if curr_speed != 0.0:
        #     print ".curr_speed, curr_angle", curr_speed, curr_angle
        # rospy.sleep(1)
        # Propagate particles forward in place
        # Sample control noise and add to nominal control
        # Make sure different control noise is sampled for each particle
        # Propagate particles through kinematic model with noisy controls

        # nominal controls of shape (3,) - can also be shape(3, 1)

        if 0: # True for Testing
            nominal_controls = np.array(TEST_CONTROLS)
        else:
            nominal_controls = np.array([curr_speed, curr_angle, msg_dt])
        # nominal controls of shape (MAX_PARTICLES, 3)
        nominal_controls_max_particles = np.tile(nominal_controls.T, (MAX_PARTICLES, 1))
        # control noise of shape (3,) - can also be shape(3, 1). noise is std_dev for [speed, angle, 0 for dt]
        control_noise_std_dev = np.array([KM_V_NOISE, KM_DELTA_NOISE, 0.0]) # only speed and angle noise, no dt noise
        # apply gaussian noise to our nominal controls for all MAX_PARTICLES
        # noisy controls of shape (MAX_PARTICLES, 3) - function arguments are shape (MAX_PARTICLES, 3) & (3,)
        noisy_controls_max_particles = get_nrand_samples(nominal_controls_max_particles, control_noise_std_dev)

        self.particles[:] = kinematic_model_step(self.particles, noisy_controls_max_particles, self.CAR_LENGTH)[:]

        # Sample model noise for each particle
        # Limit particle theta to be between -pi and pi
        # Vectorize your computations as much as possible
        # All updates to self.particles should be in-place

        # nominal model poses - shape (MAX_PARTICLES, 3)
        # these are the poses that each particle would have if our model is perfect
        # so all particles would have the same values [x, y, theta]

        # nominal_model_poses_max_particles = kinematic_model_step(self.particles, nominal_controls_max_particles,
        #                                                          self.CAR_LENGTH)[:]
        model_noise_std_dev = np.array([KM_X_FIX_NOISE, KM_Y_FIX_NOISE, KM_THETA_FIX_NOISE])
        # noisy_model_poses_max_particles = get_nrand_samples(nominal_model_poses_max_particles, model_noise_std_dev)
        noisy_model_poses_max_particles = get_nrand_samples(self.particles, model_noise_std_dev)
        self.particles[:] = noisy_model_poses_max_particles[:]#-nominal_model_poses_max_particles[:])

        self.last_vesc_stamp = msg.header.stamp
        self.state_lock.release()



'''
  Code for testing motion model
'''

TEST_SPEED = 1.0  # meters/sec
TEST_STEERING_ANGLE = 0.34 #0.34  # radians
TEST_DT = 1.0  # seconds
TEST_CONTROLS = [TEST_SPEED, TEST_STEERING_ANGLE, TEST_DT]

def main():
    rospy.init_node("odometry_model", anonymous=True)  # Initialize the node
    particles = np.zeros((MAX_PARTICLES, 3))  # [1000 x 3] , 1000 particles each with pose [x, y, theta]

    # Load paramsparticles
    motor_state_topic = rospy.get_param("~motor_state_topic",
                                        "/vesc/sensors/core")  # The topic containing motor state information
    servo_state_topic = rospy.get_param("~servo_state_topic",
                                        "/vesc/sensors/servo_position_command")  # The topic containing servo state information
    speed_to_erpm_offset = float(
        rospy.get_param("/vesc/speed_to_erpm_offset", 0.0))  # Offset conversion param from rpm to speed
    speed_to_erpm_gain = float(
        rospy.get_param("/vesc/speed_to_erpm_gain", 4350))  # Gain conversion param from rpm to speed
    steering_angle_to_servo_offset = float(rospy.get_param("/vesc/steering_angle_to_servo_offset",
                                                           0.5))  # Offset conversion param from servo position to steering angle
    steering_angle_to_servo_gain = float(rospy.get_param("/vesc/steering_angle_to_servo_gain",
                                                         -1.2135))  # Gain conversion param from servo position to steering
    car_length = float(rospy.get_param("/car_kinematics/car_length", 0.33))  # The length of the car

    # Going to fake publish controls
    servo_pub = rospy.Publisher(servo_state_topic, Float64, queue_size=1)  # for angle
    vesc_state_pub = rospy.Publisher(motor_state_topic, VescStateStamped, queue_size=1)  # for speed

    kmm = KinematicMotionModel(motor_state_topic, servo_state_topic, speed_to_erpm_offset,
                               speed_to_erpm_gain, steering_angle_to_servo_offset,
                               steering_angle_to_servo_gain, car_length, particles)

    # Give time to get setup
    rospy.sleep(1.0)

    # Send initial position and vesc state
    # Send angle
    servo_msg = Float64()
    servo_msg.data = steering_angle_to_servo_gain * TEST_STEERING_ANGLE + steering_angle_to_servo_offset
    servo_pub.publish(servo_msg)
    rospy.sleep(1.0)

    # Send speed
    vesc_msg = VescStateStamped()
    vesc_msg.header.stamp = rospy.Time.now()
    vesc_msg.state.speed = speed_to_erpm_gain * TEST_SPEED + speed_to_erpm_offset
    vesc_state_pub.publish(vesc_msg)
    rospy.sleep(TEST_DT)

    # Send speed again?
    # I think this is so we can measure DT using the time stamps
    vesc_msg.header.stamp = rospy.Time.now()
    vesc_state_pub.publish(vesc_msg)
    rospy.sleep(1.0)

    kmm.state_lock.acquire() # kinematic motion model method
    # Visualize particles
    fig = plt.figure(figsize=(15,10)) # set figure size in inches (width, height)
    ax = fig.add_subplot(111)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(('KM_V_NOISE: %0.2f, KM_DELTA_NOISE: %0.2f, KM_X_FIX_NOISE: %0.2f, KM_Y_FIX_NOISE: %0.2f, KM_THETA_FIX_NOISE: %0.2f\n'
               'Speed: %0.2f m/sec, Steering Angle %0.2f rad, Time Interval %0.2f sec'
               % (KM_V_NOISE,
                  KM_DELTA_NOISE,
                  KM_X_FIX_NOISE,
                  KM_Y_FIX_NOISE,
                  KM_THETA_FIX_NOISE,
                  TEST_SPEED, TEST_STEERING_ANGLE, TEST_DT
                  )), fontsize=12) # subtitle
    plt.suptitle('Kinematic Motion Model with Stochasticity', fontsize=18) # title
    ax.scatter([0], [0], c='r', label='Initial Positions')
    ax.scatter(particles[:, 0], particles[:, 1], c='b', label='Progogated Positions') # [x y] for 1000 particles
    ax.legend(loc='best')
    plt.show()
    kmm.state_lock.release() # kinematic motion model method

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

if __name__ == '__main__':
    main()