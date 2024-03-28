import numpy as np
from scipy.optimize import minimize


def sum_of_squares(params, own_positions, bearings, time_steps):
    L1_distance, crs, spd = params
    crs_rad = np.radians(crs)
    brg = np.radians(bearings)
    
    # target speed components
    u = spd * np.sin(crs_rad)
    v = spd * np.cos(crs_rad)
    
    # target position at first bearing
    m0, n0 = own_positions[0]
    a = m0 + L1_distance * np.sin(brg[0])
    b = n0 + L1_distance * np.cos(brg[0])
    
    # sum of errors
    f_sum = 0
    for idx in range(len(time_steps)):
        m, n = own_positions[idx]
        x = a + u * time_steps[idx]
        y = b + v * time_steps[idx]
        f_t = (y - n) * np.sin(brg[idx]) - (x - m) * np.cos(brg[idx])
        f_sum += f_t ** 2
    return f_sum


# Input data
own_positions = [(262, 505), (255, 505), (248, 505), (240, 505)]  # Example positions at each time interval
bearings = [153.61687, 154.629, 155.75335, 156.89733]  # Example bearings in degrees
time_steps = [0, 60, 120, 180]  # Example 60 time intervals between bearings

# Initial guess for L1_distance, crs, and spd
initial_guess = [5000.0, 180.0, 5.0]
res = minimize(sum_of_squares, initial_guess, args=(own_positions, bearings, time_steps), method='BFGS')
error = sum_of_squares(res.x, own_positions, bearings, time_steps)

L1_distance, crs, spd = res.x
print('Optimized Target Distance at First Bearing: {0} pixel'.format(L1_distance))
print('Optimized Target Course: {0} deg'.format(crs))
print('Optimized Target Speed: {0} pixel/s'.format(spd))
print('Error: {0}, should be less than 1e-6'.format(error))
