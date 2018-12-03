import timeit
import numpy as np
# This file is used for timing get_nrand_samples to find the most efficient way to write it
# The idea is to copy paste the function into here and instead of passing arguments define them
# inside the function so that it can be called by timeit.timeit()
# The conlusion is that there is not a significant different between using controls [speed, angle, dt]
# and controls [speed, angle]. Since we don't need to

def get_nrand_samples_3_d_zeros():
    # enter test nominal controls of shape (2,) - default, or (2,1) - i.e. np.array([[1], [2]])
    nominal_controls = np.array([1, 2, 3])
    # enter test noise - std_dev for normal distribution. must be same number of elements as controls
    noise = np.array([0, 0.1,0])
    # enter max number of particles to test
    MAX_PARTICLES = 1000
    # reshape nominal controls into shape [MAX_PARTICLES, 2, 1)
    nominal_controls = np.zeros(([MAX_PARTICLES,nominal_controls.shape[0]]))
    nominal_controls[:] = np.array([1, 2, 3])
    # print "nominal_controls.shape", nominal_controls.shape
    # nominal_controls = np.tile(nominal_controls.T, (MAX_PARTICLES, 1))
    # verify that noise is being applied to correct dimension - easy to see if noise is [zero, non-zero]
    mu = np.array(nominal_controls)
    std_dev = np.array(noise)
    # https://www.numpy.org/devdocs/reference/generated/numpy.random.randn.html
    # * symbol: https://stackoverflow.com/questions/38860095/create-random-numpy-matrix-of-same-size-as-another
    if type(mu) is np.ndarray:
        samples = std_dev * np.random.randn(*mu.shape) + mu
    else: # assume single value such as float or int
        samples = std_dev * np.random.randn() + mu
    # print samples.shape, samples
    return samples

def get_nrand_samples_3_d_tiles():
    # enter test nominal controls of shape (2,) - default, or (2,1) - i.e. np.array([[1], [2]])
    nominal_controls = np.array([1, 2, 3])
    # enter test noise - std_dev for normal distribution. must be same number of elements as controls
    noise = np.array([0, 0.1, 0])
    # enter max number of particles to test
    MAX_PARTICLES = 1000
    # reshape nominal controls into shape [MAX_PARTICLES, 2, 1)
    nominal_controls = np.tile(nominal_controls.T, (MAX_PARTICLES, 1))
    # print "nominal_controls.shape", nominal_controls.shape
    # verify that noise is being applied to correct dimension - easy to see if noise is [zero, non-zero]
    mu = np.array(nominal_controls)
    std_dev = np.array(noise)
    # https://www.numpy.org/devdocs/reference/generated/numpy.random.randn.html
    # * symbol: https://stackoverflow.com/questions/38860095/create-random-numpy-matrix-of-same-size-as-another
    if type(mu) is np.ndarray:
        samples = std_dev * np.random.randn(*mu.shape) + mu
    else: # assume single value such as float or int
        samples = std_dev * np.random.randn() + mu
    # print samples.shape, samples
    return samples
print timeit.timeit(get_nrand_samples_3_d_zeros, number=1)
print timeit.timeit(get_nrand_samples_3_d_tiles, number=1)