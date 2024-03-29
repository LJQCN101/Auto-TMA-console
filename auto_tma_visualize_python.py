import numpy as np
import cv2
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
        f_sum += f_t**2
    return f_sum


own_positions = [
    [434.0743034055728, 0],
    [450, 0],
    [462.9021739130435, 0],
    [474.9492385786802, 0],
    [486.7004830917874, 0],
    [498.9080459770115, 0],
    [509.88528138528136, 0],
    [520.8677685950413, 0],
    [530.8467583497053, 0],
    [541.8310412573674, 0],
    [551.6338582677165, 0],
    [559.7956777996071, 0],
    [568.7819253438114, 0],
    [576.7662082514735, 0],
    [584.7529644268775, 0],
    [593.7401574803149, 0],
    [600.7263779527559, 0],
]

bearings = [
    314.2503569900685,
    314.93626723483635,
    315.78493675953024,
    316.7281349399127,
    317.3478189561612,
    317.85852547187903,
    318.5372185316053,
    319.1048205725291,
    319.7833934218179,
    320.27202035896585,
    320.8210555892058,
    321.5216998480398,
    322.0785806689714,
    322.57333104472843,
    323.11191839353603,
    323.5464883564747,
    324.0596276579347,
]

time_steps = [
    0,
    60,
    120,
    180,
    240,
    300,
    360,
    420,
    480,
    540,
    600,
    660,
    720,
    780,
    840,
    900,
    960,
]

# Initial guess for L1_distance, crs, and spd
initial_guess = [5000.0, 180.0, 5.0]
res = minimize(
    sum_of_squares,
    initial_guess,
    args=(own_positions, bearings, time_steps),
    method="BFGS",
)
error = sum_of_squares(res.x, own_positions, bearings, time_steps)

L1_distance, crs, spd = res.x
print("Optimized Target Distance at First Bearing: {0} pixel".format(L1_distance))
print("Optimized Target Course: {0} deg".format(crs))
print("Optimized Target Speed: {0} pixel/s".format(spd))
print("Error: {0}".format(error))

# prepare to draw solution on a blank canvas
# target speed components
crs_rad = np.radians(crs)
u = spd * np.sin(crs_rad)
v = spd * np.cos(crs_rad)

# target position at first bearing
brg = np.radians(bearings)
m0, n0 = own_positions[0]
a = m0 + L1_distance * np.sin(brg[0])
b = n0 + L1_distance * np.cos(brg[0])

# target position at last bearing
ax = a + u * time_steps[-1]
bx = b + v * time_steps[-1]

# Create a blank canvas
canvas = np.zeros((600, 800, 3), dtype=np.uint8)

# Connect the points and draw lines on the canvas
for i in range(len(time_steps)):
    m, n = own_positions[i]
    x = m + 2000.0 * np.sin(brg[i])
    y = n + 2000.0 * np.cos(brg[i])
    cv2.line(canvas, (int(m), int(n)), (int(x), int(y)), (0, 255, 0), 2)

cv2.line(canvas, (int(a), int(b)), (int(ax), int(bx)), (0, 0, 255), 2)

# Display the canvas
cv2.imshow("Canvas", canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
