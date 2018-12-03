import numpy as np
import time
# This code is written to both test and use the helper function get_r_samples,
# which is used inside MotionModel.py
# You can run this file after uncommenting the lines at the bottom to
# verify the function works with a print statement

'''
Helper function to get random normal sample with mean mu and standard deviation mu.
Inputs can be a numpy array or single values
'''
def get_nrand_samples(mu, std_dev):
    # https://www.numpy.org/devdocs/reference/generated/numpy.random.randn.html
    # * symbol: https://stackoverflow.com/questions/38860095/create-random-numpy-matrix-of-same-size-as-another
    if type(mu) is np.ndarray:
        samples = std_dev * np.random.randn(*mu.shape) + mu
    else: # assume single value such as float or int
        samples = std_dev * np.random.randn() + mu
    return samples


###
# UNCOMMENT THIS FOR TESTING
###

# # enter test nominal controls of shape (3,) - default, or (3,1) - i.e. np.array([[1], [2], [3]])
# nominal_controls = np.array([1, 3, 7])
# # enter test noise - std_dev for normal distribution. must be same number of elements as controls
# noise = np.array([0, 0.1, 0])
# # enter max number of particles to test
# MAX_PARTICLES = 10
# # reshape nominal controls into shape (MAX_PARTICLES, 3)
# nominal_controls = np.tile(nominal_controls.T,(MAX_PARTICLES,1))
# # verify that noise is being applied to correct dimension - easy to see if noise is [zero, non-zero, zero]
# print get_nrand_samples(np.array(nominal_controls), np.array(noise))