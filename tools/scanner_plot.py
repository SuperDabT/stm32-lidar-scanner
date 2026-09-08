import serial
import matplotlib.pyplot as plt
import math
import time
import numpy as np
from matplotlib.animation import FuncAnimation

from serial.tools import list_ports


def find_stm32():
    for port in list_ports.comports():
        if port.vid == 0x0483 and port.pid == 0x374B:
            return port.device
    return None


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

    # Same positions for both layers, so build the Nx2 array once.
    coords = np.column_stack([angles, dists])

    # Soft phosphor glow beneath a brighter core, both using the same
    # age-based fade so the bloom decays with the point.
    glow.set_offsets(coords)
    glow.set_alpha([a * 0.15 for a in alphas])
    core.set_offsets(coords)
    core.set_alpha(alphas)

    return (glow, core)


port = find_stm32()

if port is None:
    raise SystemExit("No ST-LINK found — check the board is plugged in")

ser = serial.Serial(port, 115200, timeout=1)
points = []

fig = plt.figure(facecolor='#020402', figsize=(8, 8))
ax = fig.add_subplot(projection='polar', facecolor='#020402')
ax.set_facecolor('#020402')
ax.set_title('LIDAR SCAN', color='#39FF6A', fontsize=12, fontweight='bold',
             fontfamily='monospace', pad=18)
ax.tick_params(colors='#04C334', labelsize=8)
ax.grid(color='#04C334', alpha=0.2, linewidth=0.6)
ax.spines['polar'].set_color('#04C334')
ax.spines['polar'].set_alpha(0.6)

ax.set_thetamin(0)
ax.set_thetamax(360)
ax.set_ylim(0, 300)
ax.set_rlabel_position(135)

# Created once, empty. glow first so it draws underneath core.
glow = ax.scatter([], [], s=60, color='#04C334', edgecolors='none')
core = ax.scatter([], [], s=6,  color='#39FF6A', edgecolors='none')

ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
plt.show()