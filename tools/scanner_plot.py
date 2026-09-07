import serial
import matplotlib.pyplot as plt
import math
import time
from matplotlib.animation import FuncAnimation

def update(frame):
    now = time.time()

    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 5:
            continue
        pan, tilt, dist, temp, valid = (int(p) for p in parts)
        if not valid:
            continue
        points.append((math.radians(pan), dist, now))

    points[:] = [p for p in points if now - p[2] < 4.5]

    angles = [p[0] for p in points]
    dists  = [p[1] for p in points]
    alphas = [1.0 - (now - p[2]) / 4.5 for p in points]

    ax.clear()
    ax.set_ylim(0, 500)
    ax.scatter(angles, dists, s=4, alpha=alphas)
    

ser=  serial.Serial('COM3',115200,timeout=1)
points= []


fig= plt.figure()
ax= fig.add_subplot(projection='polar')
ani=FuncAnimation(fig, update, interval=50)
plt.show()