import serial
import matplotlib.pyplot as plt
import math
import time
import numpy as np
import csv
import sys
from typing import cast
from matplotlib.animation import FuncAnimation
from matplotlib.projections.polar import PolarAxes

from serial.tools import list_ports

REPLAY_ROWS_PER_FRAME = 2
RMAX_CM=500    # Plot ceiling. TF-Luna reaches 800cm, but nothing in this
               # project's actual use range goes past ~400 — 500 leaves margin
               # without wasting most of the chart on empty space.

def load_csv(path):
    rows= []
    with open(path) as f:
        reader= csv.DictReader(f)
        for row in reader:
            rows.append(row)
        return rows

def on_key(event):
    global paused
    if event.key==' ':
        paused= not paused

def find_stm32():
    for port in list_ports.comports():
        if port.vid == 0x0483 and port.pid == 0x374B:
            return port.device
    return None

def draw_layer(objects,time_now,glowing,coring):
    angles = [p[0] for p in objects]
    dists  = [p[1] for p in objects]
    alphas = [1.0 - (time_now - p[-1]) / 4.5 for p in objects]
    
        # Same positions for both layers, so build the Nx2 array once.
    if objects:
            coords = np.column_stack([angles, dists])
            # Soft phosphor glow beneath a brighter core, both using the same
            # age-based fade so the bloom decays with the point.
            glowing.set_offsets(coords)
            glowing.set_alpha([a * 0.15 for a in alphas])
            coring.set_offsets(coords)
            coring.set_alpha(alphas)
    else: 
            glowing.set_offsets(np.empty((0,2)))
            coring.set_offsets(np.empty((0,2)))
    
    


def update(frame):
    now = time.time()

    if replay:
        global cursor
        if not paused:
            for row in rows[cursor:cursor+REPLAY_ROWS_PER_FRAME]:
                pan = int(row['pan'])
                dist = int(row['distance'])
                valid = int(row['valid'])
                if not valid:
                    continue
                points.append((math.radians(pan), dist, now))
            cursor += REPLAY_ROWS_PER_FRAME
            if cursor>= len(rows):
                cursor=0
            
    else:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            parts = line.split(',')
            if line.startswith('DET,'):
                bearing=int(parts[1])
                distance=int(parts[2])
                width=int(parts[3])
                detections.append((math.radians(bearing),distance,width,now))
                continue

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
    detections[:]=[d for d in detections if now-d[3]<4.5]
    
    draw_layer(points,now,glow,core)
    draw_layer(detections,now,det_glow,det_core)

    return(glow,core,det_glow,det_core)



   



replay=len(sys.argv)>1

if replay:
    rows=load_csv(sys.argv[1])
    cursor=0
    paused=False
else:
    port= find_stm32()
    if port is None:
        raise SystemExit("No ST-LINK found — check the board is plugged in")
    ser= serial.Serial(port,115200,timeout=1)

points=[]
detections=[]
         
fig = plt.figure(facecolor='#020402', figsize=(8, 8))
fig.canvas.mpl_connect('key_press_event',on_key)
ax = cast(PolarAxes,fig.add_subplot(projection='polar', facecolor='#020402'))
ax.set_facecolor('#020402')
ax.set_title('LIDAR SCAN', color='#39FF6A', fontsize=12, fontweight='bold',
            fontfamily='monospace', pad=18)
ax.tick_params(colors='#04C334', labelsize=8)
ax.grid(color='#04C334', alpha=0.2, linewidth=0.6)
ax.spines['polar'].set_color('#04C334')
ax.spines['polar'].set_alpha(0.6)
ax.set_thetamin(0)
ax.set_thetamax(360)
ax.set_ylim(0, RMAX_CM)
ax.set_rlabel_position(135) 
# Created once, empty. glow first so it draws underneath core.
glow = ax.scatter([], [], s=60, color='#04C334', edgecolors='none')
core = ax.scatter([], [], s=6,  color='#39FF6A', edgecolors='none')

det_glow=ax.scatter([], [], s=100, color='#B3001B', edgecolors='none')
det_core=ax.scatter([], [], s=12, color='#FF3B3B', edgecolors='none')

ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
plt.show() 

