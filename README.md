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

## How it works

### Servo control

TIM2 runs at 90 MHz from APB1. A prescaler of 89 gives one tick per microsecond; ARR of 19999 wraps the counter every 20 ms, producing the 50 Hz frame hobby servos expect. Because one tick is one microsecond, the compare register holds the pulse width directly:

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
- **Offset** — the servo's zero does not agree with the chassis. Command 105 points straight along the base's forward axis, measured with a straightedge against the square base and checked by a second person. `servo_write` adds the 15° internally, so everything above it — scan loop, telemetry, detection — works in true bearings where 90 is straight ahead. (A sweep across a flat wall fits the wall to within 1.5 cm RMS and finds its perpendicular to a tenth of a degree, but only tells you the offset if the wall is known to be square to the base; this one wasn't, which is why the final number came from the straightedge.)

### Tilt level

The original notes said tilt 115 was level. It wasn't: a target at 170 cm read 126 cm, because the beam was angled down into the floor before reaching it. Level was re-measured two independent ways against a wall 344 cm away, with the lens 74.7 cm above the floor:

- **Shortest reading.** A level beam takes the shortest path to a wall, so wall distance is smallest at level. The minimum sits at tilt 111, rising on both sides.
- **Floor knee.** Tilting down, the beam starts hitting floor before the wall between commands 123 and 124. `atan(74.7 / 344)` puts that 12.2° below level, which gives level at 111.3.

Both give 111, and command units map 1:1 to degrees. 115 had been pointing 4° down.

### Scan pattern

Pan steps 2° every 50 ms across its range, about 4 seconds per sweep, reversing at either limit. For detection the tilt is held at level, so every sweep is the same horizontal slice of the room and can be compared against a baseline. The firmware can also raster — advancing tilt 3° at each reversal for stacked horizontal rows — but that's switched off while detecting.

The 50 ms dwell is measured, not guessed. A servo commanded to a new angle is still moving when the next reading arrives, so the reading belongs to a position the code has already left. The size of that error is visible in the data: a sharp edge in the scene lands at different bearings depending on sweep direction, and the split is the lag doubled.

| Dwell | Split |
|---|---|
| 40 ms | 6° |
| 45 ms | 4° |
| 50 ms | 2° |
| 65 ms | 0° |

50 ms is the fastest interval holding the split within one step — the floor, since the true edge always falls between two samples.

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
- **distance** — the run's *closest* reading, not its average. The readings at the edges of a person are the beam half on them and half on the wall behind, and averaging drags the answer toward the wall. At the person_15 spot the run reads 229, 203, 170, 170, 170, 172, 174 — the average is 190, which nothing in the room is.

Runs are tracked as six numbers — first and last bearing and distance, the running minimum, a count — rather than a buffer of readings, so there's no overflow case to handle.

The 40 cm threshold was measured, not chosen. Sweeping it from 5 to 100 cm against the recorded captures: below 15 the sensor's own wobble triggers false alarms in the empty room; above 65 a person gets missed. 40 is the middle of that band.

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

Replaying the C code before flashing also caught a bug the board would have hit: if calibration starts partway through a sweep, some angles never get a reading, the median of `0, 0, x` is 0, and those angles go permanently blind. Calibration must start with the head at the edge of the sweep.

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
