import math
import numpy as np

'''
Apply the kinematic model to the passed pose and control
  poses: particle set   eg; 1000 x 3
  controls: Set of controls [v, delta, dt] to be applied to each particle
  car_length: The length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(poses, controls, car_length):
    # Apply the kinematic model
    # Make sure your resulting theta is between 0 and 2*pi
    # Consider the case where delta == 0.0
    v = controls[:, 0] # get velocity column. shape (MAX_PARTICLES,)
    delta = controls[:, 1] # get angle column. shape (MAX_PARTICLES,)
    dt = controls[:, 2] # get time change column, shape (MAX_PARTICLES,)

    xt_minus_1 = poses[:, 0] # only one value for initial pose position x
    yt_minus_1 = poses[:, 1] # only one value for initial pose position y
    theta_t_minus_1 = poses[:, 2] # only one value for initial pose orientation theta
    beta = np.arctan((1.0 / 2.0) * np.tan(delta)) # beta shape (1000,)

    theta = theta_t_minus_1 + (v / car_length) * np.sin(2 * beta * dt)  # theta shape (1000,)TODO: dt inside or outside?

    ## Basically the part below is a just a vectorized version of this
    # if beta == 0: # case where delta == 0.0
    #     xt = xt_minus_1 + v * dt  # distance = speed * time
    #     yt = 0  # no change in y because we are goiparticlesng straight along the x axis in the car's frame
    # else:
    #     xt = xt_minus_1 + car_length / math.sin(2 * beta) * (math.sin(theta) - math.sin(theta_t_minus_1))
    #     # print "xt,", xt
    #     yt = yt_minus_1 + car_length / math.sin(2 * beta) * (-math.cos(theta) + math.cos(theta_t_minus_1))
    #     # print "yt", yt

    xt = np.zeros(controls.shape[0]) # xt shape (MAX_PARTICLES,)
    yt = np.zeros(controls.shape[0])  # yt shape (MAX_PARTICLES,)

    zero_indices = np.argwhere(beta == 0) # case where delta == 0.0
    non_zero_indices = np.argwhere(beta != 0)

    xt[zero_indices] = xt_minus_1[zero_indices] + v[zero_indices] * dt[zero_indices]
    yt[zero_indices] = 0

    xt[non_zero_indices] = xt_minus_1[non_zero_indices] + (car_length / np.sin(2 * beta[non_zero_indices])) * (
        np.sin(theta[non_zero_indices]) - np.sin(theta_t_minus_1[non_zero_indices]))

    yt[non_zero_indices] = yt_minus_1[non_zero_indices] + (car_length / np.sin(2 * beta[non_zero_indices])) * (
            -np.cos(theta[non_zero_indices]) + np.cos(theta_t_minus_1[non_zero_indices]))

    # noisy_particles = np.zeros(controls.shape) # shape (MAX_PARTICLES, 3)
    # noisy_particles[:, 0] = xt
    # noisy_particles[:, 1] = yt
    # noisy_particles[:, 2] = theta
    poses[:, 0] = xt[:]
    poses[:, 1] = yt[:]
    poses[:, 2] = theta[:]
    return poses