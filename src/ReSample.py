#!/usr/bin/env python

import rospy
import numpy as np
from threading import Lock

'''
  Provides methods for re-sampling from a distribution represented by weighted samples
'''


class ReSampler:
    '''
      Initializes the resampler
      particles: The particles to sample from
      weights: The weights of each particle
      state_lock: Controls access to particles and weights
    '''

    def __init__(self, particles, weights, state_lock=None):
        self.particles = particles[:]
        self.weights = weights[:]

        # For speed purposes, you may wish to add additional member variable(s) that
        # cache computations that will be reused in the re-sampling functions
        # YOUR CODE HERE?



        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

        # self.resample_naiive()

    '''
      Performs independently, identically distributed in-place sampling of particles
    '''

    def resample_naiive(self):
        # print "RESAMPLE NAIIVE START"
        self.state_lock.acquire()

        # YOUR CODE HERE
        # draw random numbers between 0 and 1.0 - draw between 0-101 (not inclusive so 0 - 100) and divide by 100
        # rand_samples = np.random.choice(1001, len(self.particles)) / 1000.0 # draw a sample for each particle
        # print "Particles inside resample_naiive", self.particles.shape, self.particles
        particle_indices = np.arange(len(self.particles), dtype=np.float) # [0, 1, ..., 98, 99]
        selected_indices = np.random.choice(particle_indices, len(self.particles), p=self.weights)
        self.particles[:, 0] = selected_indices[:]
        self.particles[:, 1] = selected_indices[:]
        self.particles[:, 2] = selected_indices[:]

        # set weights to uniform distribution
        # weights: <type 'numpy.ndarray'> shape (100,)
        self.weights[:] = np.sum(self.weights) / self.weights.shape[0]


        self.state_lock.release()
        # print "RESAMPLE COMPLETE"

    '''
      Performs in-place, lower variance sampling of particles
      (As discussed on pg 110 of Probabilistic Robotics)
    '''

    def resample_low_variance(self):

        np.set_printoptions(suppress=True)

        
        self.state_lock.acquire()
        particle_indices = np.arange(len(self.particles), dtype=np.float)  # [0, 1, ..., 98, 99]
        one_over_m = 1.0 / len(self.particles)
        random_num = np.random.uniform(0, one_over_m)
        random_values = np.append(random_num, random_num + particle_indices[1::1] * one_over_m  )
        # print "random_values", random_values
        # print "one_over_m", one_over_m

        bin_boundaries = np.append([0], np.cumsum(self.weights))
        # print "bin_boundaries", bin_boundaries
        bins_selected = np.digitize(random_values, bin_boundaries, right=False) - 1
        # print "bins_selected", len(bins_selected), bins_selected
        self.particles[:, 0] = self.particles[bins_selected,0][:]
        self.particles[:, 1] = self.particles[bins_selected,1][:]
        self.particles[:, 2] = self.particles[bins_selected,2][:]

        # set weights to uniform distribution
        # weights: <type 'numpy.ndarray'> shape (100,)
        self.weights[:] = np.sum(self.weights) / self.weights.shape[0]

        # YOUR CODE HERE

        self.state_lock.release()


import matplotlib.pyplot as plt


def main():
    rospy.init_node("sensor_model", anonymous=True)  # Initialize the node

    n_particles = int(rospy.get_param("~n_particles", 100))  # The number of particles
    k_val = int(rospy.get_param("~k_val", 80))  # Number of particles that have non-zero weight
    resample_type = rospy.get_param("~resample_type", "naiive")  # Whether to use naiive or low variance sampling
    trials = int(rospy.get_param("~trials", 10))  # The number of re-samplings to do

    histogram = np.zeros(n_particles, dtype=np.float)  # Keeps track of how many times
    # each particle has been sampled
    # across trials

    for i in xrange(trials):
        # shape (100, 3), 100 poses of [x, y, theta]
        particles = np.repeat(np.arange(n_particles)[:, np.newaxis], 3, axis=1)  # Create a set of particles
        # print "particles", particles
        # Here their value encodes their index
        # Have increasing weights up until index k_val
        weights = np.arange(n_particles, dtype=np.float) # shape (100,)

        # print "weights", weights.shape
        # assign weights[80:100] to 0
        weights[k_val:] = 0.0
        # normalize weights - divide each element by sum of all elements
        weights[:] = weights[:] / np.sum(weights)

        # arguments are shapes (100, 3), (100,)
        rs = ReSampler(particles, weights)  # Create the Resampler

        # Resample
        if resample_type == "naiive":
            rs.resample_naiive()
        elif resample_type == "low_variance":
            rs.resample_low_variance()
        else:
            print "Unrecognized resampling method: " + resample_type

            # Add the number times each particle was sampled
        for j in xrange(particles.shape[0]):
            histogram[particles[j, 0]] = histogram[particles[j, 0]] + 1

    # Display as histogram
    plt.bar(np.arange(n_particles), histogram)
    plt.xlabel('Particle Idx')
    plt.ylabel('# Of Times Sampled')
    plt.show()


if __name__ == '__main__':
    main()
