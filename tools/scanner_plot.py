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
        try:
            pan, tilt, dist, temp, valid = (int(p) for p in parts)
        except ValueError:
            continue
        if not valid:
            continue
        points.append((math.radians(pan), dist, now))

    points[:] = [p for p in points if now - p[2] < 4.5]

    angles = [p[0] for p in points]
    dists  = [p[1] for p in points]
    alphas = [1.0 - (now - p[2]) / 4.5 for p in points]

    ax.clear()
    ax.set_facecolor('#020402')
    ax.set_title('LIDAR SCAN', color='#39FF6A', fontsize=12, fontweight='bold',
                  fontfamily='monospace', pad=18)
    ax.tick_params(colors='#04C334', labelsize=8)
    ax.grid(color='#04C334', alpha=0.2, linewidth=0.6)
    ax.spines['polar'].set_color('#04C334')
    ax.spines['polar'].set_alpha(0.6)
    ax.set_ylim(0, 500)
    ax.set_rlabel_position(135)
    # Soft phosphor glow beneath a brighter core, both using the same
    # age-based fade so the bloom decays with the point.
    ax.scatter(angles, dists, s=60, alpha=[a * 0.15 for a in alphas],
               color='#04C334', edgecolors='none')
    ax.scatter(angles, dists, s=6, alpha=alphas, color='#39FF6A', edgecolors='none')


ser=  serial.Serial('COM3',115200,timeout=1)
points= []


fig= plt.figure(facecolor='#020402', figsize=(8, 8))
ax= fig.add_subplot(projection='polar',facecolor='#020402')
ani=FuncAnimation(fig, update, interval=50)
plt.show()