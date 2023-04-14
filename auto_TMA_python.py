import numpy as np
from scipy.optimize import minimize

def bearing_vector(idx, t, crs, spd, L1_distance, own_positions, bearings):
    brg = np.radians(bearings)
    u = spd * np.sin(crs)
    v = spd * np.cos(crs)
    
    a = L1_distance * np.sin(brg[0])
    b = L1_distance * np.cos(brg[0])
    
    m, n = own_positions[idx]
    
    x = a + u * t
    y = b + v * t
    
    return (y - n) * np.sin(brg[idx]) - (x - m) * np.cos(brg[idx])


def sum_of_squares(params, own_positions, bearings, time_intervals):
    L1_distance, crs, spd = params
    f_sum = 0
    
    for idx, t in enumerate(time_intervals):
        f_t = bearing_vector(idx, t, crs, spd, L1_distance, own_positions, bearings)
        f_sum += f_t ** 2
    
    return f_sum

# Input data
own_positions = [(0, 0), (0, 0), (0, 0), (300, 300)]  # Example positions at each time interval
bearings = [0, 10, 20, 20]  # Example bearings in degrees
time_intervals = [0, 120, 240, 360]  # Example time intervals between bearings

# Initial guess for L1_distance, crs, and spd
initial_guess = [5000, 0, 3]

# Optimize the sum of squares using the BFGS algorithm
res = minimize(sum_of_squares, initial_guess, args=(own_positions, bearings, time_intervals),
                method='BFGS')

L1_distance, crs, spd = res.x
print('Optimized Target Distance at First Bearing:', L1_distance)
print('Optimized Target Course:', np.degrees(crs))
print('Optimized Target Speed:', spd * 1.94384449)