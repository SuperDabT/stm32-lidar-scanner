# stm32-lidar-scanner

A two-axis pan-tilt LiDAR scanner on an STM32F446RE Nucleo. Two servos sweep a TF-Luna time-of-flight rangefinder across a room; the board tags every distance reading with the angle it was taken at, learns what the empty room looks like, and reports anything that wasn't there before — on the board, with no PC in the loop. A PC can listen in and draw the scan as a live radar display.

The goal is a tracker: find a person, turn the head to follow them, and recover when it loses them.

Firmware in C on STM32 HAL, no RTOS. Host visualiser in Python.

<!-- PHOTO: the assembled rig, wide shot, whole gimbal visible. -->
![Assembled pan-tilt gimbal with TF-Luna mounted](rig.jpeg)

<!-- SCREENSHOT: the radar plot mid-sweep, fade visible. This is the money shot. -->
![Live polar plot](radar.png)

## Status

**Working**

- Dual-axis servo control from one timer, 50 Hz PWM, verified on a logic analyzer
- Non-blocking scheduler — both axes and the sensor run on independent intervals in a single main loop
- TF-Luna driver: interrupt-driven UART receive, sync-word frame parser, checksum validation, error recovery
- Calibrated geometry — pulse range, pan zero offset and tilt level all measured, not assumed
- Sensor temperature guard with hysteresis
- **Person detection running on the board** — self-calibrating empty-room baseline, background subtraction, candidates reported every sweep. Verified live against recorded captures (see [Detection](#detection))
- Python prototype of the detector, replayed against recorded captures
- CSV telemetry over USART2, live polar plot in Python

**Next**

- Tracking state machine (searching / tracking / coasting), scanning only near the target while tracking
- Kalman filter on Cartesian x/y, then the PID controller (written and tested off-target, not yet connected)
- Object width in real centimetres, to reject things that aren't person-sized
- UART command interface: recalibrate, point at a bearing, status
- FreeRTOS migration

## Hardware

| Part | Notes |
|---|---|
| NUCLEO-F446RE | STM32F446RE, 180 MHz Cortex-M4 with FPU |
| 2 × MG90S servo | Metal gear, 4.8–6.0 V |
| Benewake TF-Luna | 0.2–8 m, 2° beam, UART at 115200 |
| Pan-tilt bracket | Adafruit mini kit |
| 5 V supply | Dedicated rail for servos and sensor; the Luna's ceiling is 5.2 V with no over-voltage protection |

<!-- PHOTO: breadboard and wiring. Only worth including if it's tidy. -->

### Pin assignment

| Signal | Pin | Peripheral |
|---|---|---|
| Pan servo | PA1 | TIM2_CH2 |
| Tilt servo | PA0 | TIM2_CH1 |
| LiDAR TX → MCU RX | PA10 | USART1_RX |
| MCU TX → LiDAR RX | PA9 | USART1_TX |
| PC telemetry | PA2 / PA3 | USART2 (ST-Link virtual COM) |

### Measured limits

| Axis | Range | Notes |
|---|---|---|
| Pan | 0–165 (true bearing) | 90 is straight ahead. Command = bearing + 15. Linear across the range. Held to 165 because bearing 180 is command 195, past the calibrated pulse range, and the servo heated pushing against its stop |
| Tilt | 89–125 | 111 is level, lower numbers point up. Fixed at level during detection |

Lens height above the floor is 74.7 cm; every tilt calculation below uses it.

## How it works

Every constant in the firmware was measured on the rig rather than taken from a datasheet or a tutorial. Each section below gives the experiment, the math and the raw numbers behind it.

### Servo control

TIM2 runs at 90 MHz from APB1. A servo wants one pulse every 20 ms, with the pulse width encoding the angle:

```math
f_\text{tick} = \frac{90\ \text{MHz}}{\text{PSC} + 1} = \frac{90\ \text{MHz}}{90} = 1\ \text{MHz} \quad\Rightarrow\quad \text{PSC} = 89
```

```math
T = (\text{ARR} + 1) \cdot 1\ \mu\text{s} = 20\,000\ \mu\text{s} \quad\Rightarrow\quad \text{ARR} = 19\,999
```

Both carry a "+1" because the counter starts at 0: a counter running 0…89 takes 90 ticks, not 89. Because one tick is one microsecond, the compare register holds the pulse width directly:

```
CCR = 500 + (angle / 180) * 2000
```

Both channels share the prescaler and period but have independent compare registers, so one timer drives both axes.

<!-- SCREENSHOT: two-channel logic analyzer capture, both PWM channels rising together with different widths. -->

### Angle calibration

The pulse range was measured rather than assumed. The usual 1000–2000 µs figure produced only about 130° of travel on these servos, so every recorded bearing was roughly 30% short of reality — an error that would have been invisible until it corrupted object detection.

Calibration checked three things:

- **Range** — commanding 0 and 180 should put the beam on a straight line through the pivot. At 500–2500 µs it does.
- **Linearity** — 90 must land midway between the endpoints, or no single scale factor can be correct.
- **Offset** — handled separately below, because it turned out not to be zero.

#### Pan zero offset

The first attempt swept the head across a flat wall, converted the readings to x/y, and fitted a straight line through them (total least squares: the principal direction of the points' 2×2 covariance matrix). The fit was excellent — 1.5 cm RMS residual over 1106 readings — and put the wall's perpendicular at command 140.4 (141.1 outbound, 139.8 on the return, split by the dwell lag). But that answers where the head faces *that wall*, and the wall wasn't square to the base, so it couldn't give the offset.

The base itself is square, so a straightedge along it marks the true forward axis. The command that lines the head up with it read 103 at first, then 105 when refined and checked by a second person. That's a **+15°** offset. `servo_write` adds it internally, so the scan loop, telemetry and detection all work in true bearings where 90 is straight ahead.

#### Pan travel limit

With the offset applied, true bearing 180 is command 195:

```math
t_\text{pulse} = 500 + \frac{195}{180} \cdot 2000 = 2667\ \mu\text{s} \qquad (\text{calibrated ceiling: } 2500\ \mu\text{s})
```

The head physically got there, but the servo base heated up: it was pushing against its internal stop while still being commanded further, so its position error never reached zero. The limit is **true 165**, which is command 180 and exactly 2500 µs.

### Tilt level

The original notes said tilt 115 was level. A target at 170 cm read 126 cm, a 26% range error, because the beam was angled down and hitting the bed before it reached the target. Level was re-measured two independent ways.

**The geometry.** With the lens at height $h$ and the beam angled $\theta$ below horizontal, the beam, the vertical drop and the floor form a right triangle. The drop is *opposite* the angle, the beam is the hypotenuse, and the floor run is *adjacent*:

```math
\sin\theta = \frac{h}{d_\text{beam}} \qquad \tan\theta = \frac{h}{d_\text{floor}}
```

where $d_\text{beam}$ is what the LiDAR reads and $d_\text{floor}$ is the horizontal distance along the floor.

It's $\sin$, not $\cos$, because $\theta$ is measured from the horizontal. Measured from vertical, it would be $\cos$, and the two angles add to 90°.

**Method A: one floor hit.** At tilt command 150 the beam landed on bare floor and read 119 cm:

```math
\theta = \arcsin\left(\frac{74.7}{119}\right) = 38.9^\circ \qquad \text{level} \approx 150 - 38.9 = 111.1
```

39 command units produce 38.9° of real tilt, so command units map 1:1 to degrees.

**Method B: a wall and its knee.** The rig pointed at a flat wall 344.3 cm away (laser distance meter), with bare floor in front of it, in a dark room:

| Tilt | Reading (cm) | Hitting |
|---|---|---|
| 100 | 355–356 | wall, beam angled up |
| 105 | 348–350 | wall |
| **111** | **346–348** | **wall: shortest reading** |
| 115 | 348–349 | wall |
| 120 | 358–360 | wall, beam angled down |
| 122 | 357–358 | wall |
| **123** | **350–352** | **wall, grazing the floor at its base** |
| **124** | **321–324** | **floor: the knee** |
| 125 | 299–301 | floor |
| 130 | 224–226 | floor |

That one table gives two answers:

- **Shortest path.** A level beam takes the shortest path to a wall, and the reading grows as the beam tilts either way ($d = D / \cos\theta$). The minimum is at **111**.
- **The knee.** The beam first lands on the floor before the wall between 123 and 124. At that angle it drops exactly 74.7 cm over 344.3 cm:

  ```math
  \theta_\text{knee} = \arctan\left(\frac{74.7}{344.3}\right) = 12.24^\circ \qquad \text{level} = 123.5 - 12.24 = 111.3
  ```

The knee is the sharper of the two. Near level, $\cos\theta$ is almost flat (4° off level stretches a 350 cm reading by only about 1 cm), so the minimum alone can't resolve a few degrees. The knee is a cliff: the reading drops 30 cm in one step.

**Result: level is 111.** The two methods agree within a quarter of a degree, and 115 had been pointing 4° down. The tenth of a degree is dropped because the knee is only bracketed to whole command units. (An earlier attempt had seemed to show a non-1:1 mapping; that bearing turned out to point at a window, which the TF-Luna doesn't read reliably, and the point was discarded.)

**How far each tilt can see.** Once the beam angles down, the floor caps its range at $d_\text{floor} = h / \tan\theta$:

| Tilt | Below level | Floor reached at |
|---|---|---|
| 112 | 1° | 42.8 m (beyond the 8 m sensor limit) |
| 113 | 2° | 21.4 m |
| 115 | 4° | 10.7 m |
| 116 | 5° | 8.5 m |
| 120 | 9° | 4.7 m |
| 123 | 12° | 3.5 m |
| 125 | 14° | 3.0 m |

So the floor alone doesn't explain the 126 cm reading at 115. The beam was hitting the bed, which sits much closer to lens height than the floor does.

### Scan pattern

Pan steps 2° every 50 ms across its range, about 4 seconds per sweep, reversing at either limit. For detection the tilt is held at level, so every sweep is the same horizontal slice of the room and can be compared against a baseline. The firmware can also raster — advancing tilt 3° at each reversal for stacked horizontal rows — but that's switched off while detecting.

The 50 ms dwell is measured, not guessed. A servo commanded to a new angle is still moving when the next reading arrives, so the reading belongs to a position the code has already left. The size of that error is visible in the data: a sharp edge in the scene lands at different bearings depending on sweep direction, and the split is the lag doubled.

| Dwell | Split |
|---|---|
| 40 ms | 6° |
| 45 ms | 4° |
| 50 ms | 2° |
| 65 ms | 0° |

50 ms is the fastest interval holding the split within one step — the floor, since the true edge always falls between two samples. The remaining 2° wobble between sweep directions shows up in every later capture, which is a useful sign that nothing else is moving the bearings.

A related bug came first: the original loop printed the pan angle *after* commanding the next move, so each reading carried the angle the head was heading to. That produced a 16° split. Reporting before commanding fixed it.

### LiDAR

The TF-Luna free-runs at 100 Hz, pushing a nine-byte frame:

```
59 59  Dist_L Dist_H  Amp_L Amp_H  Temp_L Temp_H  Checksum
```

Bytes arrive one at a time by interrupt. A state machine hunts the `0x59 0x59` sync word, collects the remaining seven bytes, and verifies the checksum — the low eight bits of the sum of bytes 0–7 — before decoding.

Two header bytes rather than one, because `0x59` occurs in real data: a distance of 89 cm produces it. Two consecutive occurrences are rare enough to sync on, and the checksum catches the rest.

A reading is rejected when amplitude is below 100, when amplitude reads 65535 (overexposure), or when distance falls outside the sensor's trustworthy range. A distance of 0 means the sensor could not measure, not that nothing is there.

The sensor's temperature is watched too. Its ceiling is 60 °C and it normally runs 43–52 °C, so scanning halts at 60 and resumes at 55 — two thresholds rather than one, so it doesn't flap on and off at the boundary. While halted, the telemetry prints a warning once a second instead of going quiet, so the stream says why it stopped.

UART errors get their own callback. An overrun or framing error aborts the receive, and without re-arming it the sensor goes silent until reset — a failure that looks like dead hardware. The parser resynchronises on the next header by itself.

<!-- SCREENSHOT: logic analyzer UART decode showing 59 59 and a decoded frame. -->

### Timing

Everything runs off one non-blocking helper:

```c
bool elapsed(uint32_t *last, uint32_t interval);
```

It takes a pointer so it can update the caller's timestamp, letting each timed activity keep its own schedule in the same loop. No blocking delays in normal operation.

Readings are emitted before the next move is commanded, so each one carries the angle the head was actually sitting at rather than the angle it was heading toward.

### Detection

Detection lives in `detect.c` and has two phases.

**Calibrating.** At power-up the room must be empty (there's a 10-second delay to leave). Three sweeps are recorded, and the median of the three readings at each angle becomes the baseline — the "before photo" of the room. Three because the median of an even count averages the middle two, so one bad reading blends in instead of being outvoted; three is the smallest count that protects you.

**Watching.** Each new reading is compared to the baseline at its angle. A reading at least 40 cm closer than the empty room is "something there." Consecutive readings like that form a run; a run of at least three is a candidate. Each candidate reports:

- **bearing** — the midpoint of the run's first and last angle
- **distance** — the run's *closest* reading, not its average. The readings at the edges of a person are the beam half on them and half on the wall behind, and averaging drags the answer toward the wall. At the 15° test spot the run reads 229, 203, 170, 170, 170, 172, 174. The average is 184 cm, which nothing in the room is; the minimum is 170, where the person stood.

Because readings are evenly spaced 2° apart, the average of every bearing in a run equals the average of just the first and last. So runs are tracked as six numbers — first and last bearing and distance, the running minimum, a count — rather than a buffer of readings, and there's no overflow case to handle.

The 40 cm threshold was measured, not chosen, by sweeping it against recorded captures (an empty room, and a person standing at two known bearings):

| Threshold | Empty room: false alarms | Person at 15° found | Person at 95° found |
|---|---|---|---|
| 5 cm | 22 sweeps | 26/26 | 23/24 |
| 10 cm | 10 sweeps | 26/26 | 23/24 |
| 15–60 cm | **0** | **26/26** | **23/24** |
| 65 cm | 0 | 26/26 | 22/24 |
| 70 cm | 0 | 26/26 | 1/24 |
| 80 cm | 0 | 14/26 | 1/24 |

Below 15 cm the sensor's own wobble (about 3 cm on a still wall) trips it. Above 65 cm the weaker signal is missed: at 95° the person stood 65 cm in front of the wall. 40 cm sits in the middle of the band, with room on both sides.

**Width** (next step on the board, done in the Python prototype) is measured in centimetres rather than angles. The same person fills 6–11 angles at 170 cm but about half that at twice the distance, so angle counts lie about size. Converting the run's two ends to floor coordinates and measuring between them gives a size that doesn't depend on range — 43 cm and 47 cm median at the two test spots:

```math
x = d\cos\phi \qquad y = d\sin\phi \qquad w = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
```

where $\phi$ is the bearing and $d$ the distance at each end of the run.

**Why it reports candidates instead of picking one.** A single sweep can't tell a person from a chair someone moved since calibration — same width, both standing still. What separates them is movement over several sweeps, and detection has no memory between sweeps. So it hands back every candidate (up to four) and leaves the choice to the tracker. Candidates beyond four are counted rather than silently lost.

Only readings the sensor marks valid reach the detector: an invalid reading reports 0 cm, which would otherwise look like something right in front of the lens.

#### Verification

The detector was prototyped in Python against recorded captures, then ported to C, then checked three ways at the same spot:

| | bearing | distance |
|---|---|---|
| Recorded capture, Python prototype | 17 ↔ 20 | ~170 cm |
| Same capture replayed through the C code on a PC | 20.0 | 169 cm |
| Live on the board | 17/18 ↔ 20/21 | 164–167 cm |

The same two-step wobble shows up in all three: odd and even sweeps see the target from opposite directions, which is the servo lag from the dwell-time table. No detections in the empty room.

Replaying the C code before flashing also caught a bug the board would have hit: if calibration starts partway through a sweep, some angles never get a reading, the median of `0, 0, x` is 0, and those angles go permanently blind. On a replay that started mid-sweep, the person at 95° went from 23/24 to 0/24. Calibration must start with the head at the edge of the sweep. The median function itself was checked exhaustively on the host: every combination of three values from 0 to 40, plus edge cases, 68,935 cases with no failures.

#### Why tracking needs a narrower sweep

A slow walk across a measured 1.417 m stretch, about 155 cm from the rig, should span $2\arctan(0.7085 / 1.55) = 49.1^\circ$; it was observed spanning 52°, an independent check of the pan offset and distances. But 14–15 passes in 90 s is one every ~6.2 s, while a full sweep took 4.55 s:

```math
\frac{4.55\ \text{s}}{6.2\ \text{s}} \approx 0.73 \ \text{of a crossing between two looks}
```

The scanner samples a walking person less than twice per crossing, so their position appears to jump around. That's the measured reason the tracker will narrow its sweep to a window around the target once it's found.

### Telemetry and display

One CSV line per pan step:

```
pan,tilt,distance,temperature,valid
```

and, at the end of each sweep, one line per detected candidate:

```
DET,bearing,distance
```

The `DET` prefix keeps detection lines distinguishable from readings, so the plotter can't mistake one for the other.

`tools/scanner_plot.py` locates the board by USB vendor and product ID rather than a hardcoded port name, so it runs unchanged on Windows and Linux. Scatter artists are created once and updated per frame rather than clearing the axes, which keeps the redraw cheap.

Points carry a timestamp and fade over 4.5 seconds, so the current sweep stays bright and earlier rows trail behind it. The fade matters because every tilt row draws on the same plot — without it, readings taken at different elevations would be indistinguishable.

## Layout

```
Core/
  Inc/
    servo.h         pan and tilt control interface, measured limits
    tfluna.h        LiDAR interface
    detect.h        detection interface and result type
    pid.h           PID controller interface
  Src/
    main.c          application loop, scan pattern, timing
    servo.c         PWM generation, angle limits, pan offset
    tfluna.c        UART receive, frame parser, accessors
    detect.c        baseline calibration, run finding, candidates
    pid.c           velocity-form PID (not yet connected)
tools/
  scanner_plot.py   live polar display
  scan_data.py      capture loader
  detect.py         Python detection prototype
captures/           recorded scans: empty room, standing and walking
                    person, live board runs
cmake/
  files.cmake       source list
```

Each module keeps its state private and exposes only what callers need. The frame parser and the UART callback are absent from `tfluna.h` — nothing outside the driver has any business feeding bytes to it. Likewise `detect.h` exposes four functions and one struct; the baseline, the run being built and the candidate shelf are all private to `detect.c`.

## Building

Requires STM32CubeCLT and the STM32 VS Code extension.

```
cmake --preset Debug
cmake --build build/Debug
```

Peripheral configuration lives in `Iron_Dome.ioc` and is edited with STM32CubeMX. Regenerating preserves everything inside the `USER CODE` markers.

The visualiser needs `pyserial` and `matplotlib`:

```
pip install pyserial matplotlib
python tools/scanner_plot.py
```

Close any serial terminal first — only one program can hold the port.

## Lessons learned

**Check the port against recorded data before flashing.** The C detector was replayed against the same captures the Python prototype was tested on, before it ever ran on the board. It matched — and the replay turned up a calibration bug that would have left part of the room permanently blind.

**Calibrate before you build on top.** The servo pulse range, the pan zero, the tilt level, and the settling time were all wrong in the original notes, and none of the errors were visible in normal operation. A 16° bearing split between sweep directions only showed up because the same scene was captured twice and compared.

**One branch per state.** The parser bug that survived longest was a condition mixing two questions — "am I still hunting for a header?" and "is this byte a header?" — in one test. When the byte was not a header the whole branch failed and control fell through to the collecting branch, which started assembling a frame from the middle of the previous one.
