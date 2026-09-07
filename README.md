# stm32-lidar-scanner

A two-axis pan-tilt LiDAR scanner built on an STM32F446RE Nucleo board. Two servos sweep a TF-Luna time-of-flight rangefinder in a raster pattern; the board tags every distance reading with the angle it was taken at and streams the result to a PC, which draws it as a live radar display.

Firmware in C using STM32 HAL, no RTOS. Host visualiser in Python.

<!-- PHOTO: the assembled gimbal with the LiDAR mounted. Wide shot, whole rig visible. -->
![Assembled pan-tilt gimbal with TF-Luna mounted](rig.jpeg)
* Nucleo-F446RE, buck converter and the pan-tilt gimbal with the TF-Luna mounted.*

<!-- SCREENSHOT: the radar plot running — green on black, mid-sweep with the fade visible. This is the money shot; put it high. -->
![the radar plot running — green on black, mid-sweep with the fade visible](radar.png)
* Live polar plot: each point is a distance reading at its pan bearing, fading over 4.5 seconds so the current sweep stays bright.*

## Status

Working:

- Dual-axis servo control from a single timer, 50 Hz PWM, verified on a logic analyzer
- Raster scan: pan sweeps continuously, tilt advances one step per sweep
- Non-blocking scheduler — both axes and the sensor run on independent intervals in one main loop
- TF-Luna driver: interrupt-driven UART receive, frame parser with sync-word detection and checksum validation
- Signal-quality filtering per the sensor datasheet
- CSV telemetry over USART2
- Live polar plot in Python with time-based fade

Next:

- Object detection from scan data
- Closed-loop tracking
- FreeRTOS
- Heatmap view for full 3D scans

## Hardware

| Part | Notes |
|---|---|
| NUCLEO-F446RE | STM32F446RE, 180 MHz Cortex-M4 |
| 2 × MG90S servo | Metal gear, 4.8–6.0 V |
| Benewake TF-Luna | 0.2–8 m, 2° FOV, UART at 115200 |
| Pan-tilt bracket | Adafruit mini kit, 38 × 36 mm sensor platform |
| Buck converter | 12 V in, 5.07 V out — the Luna's ceiling is 5.2 V with no over-voltage protection |

<!-- PHOTO: the breadboard and wiring. Optional, but useful if it's tidy. -->

### Pin assignment

| Signal | Pin | Peripheral |
|---|---|---|
| Pan servo | PA1 | TIM2_CH2 |
| Tilt servo | PA0 | TIM2_CH1 |
| LiDAR TX to MCU RX | PA10 | USART1_RX |
| MCU TX to LiDAR RX | PA9 | USART1_TX |
| PC telemetry | PA2 / PA3 | USART2 (ST-Link virtual COM port) |

### Measured limits

Tilt is constrained by the bracket, not the servo. Level sits at 130 degrees in servo terms; lower numbers point further up, and the mechanical stop is at 135.

| Axis | Range | Notes |
|---|---|---|
| Pan | 0-180 | Full servo travel, no mechanical constraint |
| Tilt | 100-130 | 130 is horizontal, 100 is 30 degrees above it |

## How it works

### Servo control

TIM2 is clocked at 90 MHz from APB1. A prescaler of 89 divides it to 1 MHz, giving one timer tick per microsecond. ARR is 19999, so the counter wraps every 20 000 ticks — a 20 ms frame at 50 Hz, which is what hobby servos expect.

Because one tick is one microsecond, the compare register holds the pulse width in microseconds directly:

```
CCR = 1000 + (angle / 180) * 1000
```

Both channels share the timer's prescaler and period but have independent compare registers, so one timer drives both axes. Their pulses rise together and fall at different points.

<!-- SCREENSHOT: the two-channel logic analyzer capture showing both PWM channels rising simultaneously with different widths. Good technical evidence. -->

### Scan pattern

Pan steps every 20 ms across its full range. When it reaches either limit it reverses, and tilt advances one step. That gives a raster — horizontal rows stacked vertically — rather than the diagonal smear you get from stepping both axes independently.

At 2 degree pan steps and 3 degree tilt steps over a 30 degree tilt range, a full raster is 11 rows and takes about 20 seconds.

### LiDAR

The TF-Luna free-runs at 100 Hz, pushing a nine-byte frame:

```
59 59  Dist_L Dist_H  Amp_L Amp_H  Temp_L Temp_H  Checksum
```

Bytes arrive one at a time via interrupt. A small state machine hunts for the `0x59 0x59` sync word, collects the remaining seven bytes, and verifies the checksum — the low eight bits of the sum of bytes 0 through 7 — before decoding.

Two header bytes rather than one, because `0x59` appears in real data: a distance of 89 cm produces it. Two consecutive occurrences are rare enough to sync on, and the checksum catches the cases where they are not.

Distance is little-endian across two bytes. A reading is treated as invalid when amplitude is below 100, when amplitude reads 65535 (overexposure), or when the distance falls outside the sensor's trustworthy range. The application checks validity before using a reading; a distance of 0 means the sensor could not measure, not that nothing is there.

<!-- SCREENSHOT: the logic analyzer UART decode showing 59 59 and the decoded bytes of a frame. -->

### Timing

Everything runs off one non-blocking helper:

```c
bool elapsed(uint32_t *last, uint32_t interval);
```

It takes a pointer so it can update the caller's timestamp, which lets each timed activity keep its own independent schedule in the same main loop. No blocking delays anywhere in normal operation.

### Telemetry and display

The firmware emits one CSV line per pan step:

```
pan,tilt,distance,temperature,valid
```

`tools/scanner_plot.py` reads the port, converts pan angle to radians, and draws each reading on a polar plot. Points carry a timestamp and fade over 4.5 seconds — roughly two and a half sweeps — so the current row is bright and recent rows trail behind it. Anything stale is dropped.

The fade matters because every tilt row is drawn on the same plot. Without it, readings taken at different elevations would be indistinguishable.

## Layout

```
Core/
  Inc/
    servo.h         pan and tilt control interface
    tfluna.h        LiDAR interface
  Src/
    main.c          application loop, scan pattern, timing
    servo.c         PWM generation, angle limits, homing
    tfluna.c        UART receive, frame parser, accessors
tools/
  scanner_plot.py   live polar display
cmake/
  files.cmake       source list
```

Each module keeps its state private and exposes only what callers need. The frame parser and the UART callback are not in `tfluna.h` — nothing outside the driver has any business feeding bytes to it.

## Building

Requires STM32CubeCLT and the STM32 VS Code extension.

```
cmake --preset Debug
cmake --build build/Debug
```

Peripheral configuration is in `Iron_Dome.ioc` and edited with STM32CubeMX. Regenerating preserves everything inside the `USER CODE` markers.

The visualiser needs `pyserial` and `matplotlib`:

```
pip install pyserial matplotlib
python tools/scanner_plot.py
```

Close any serial terminal first — only one program can hold the COM port.

## Notes from the build

Two dead servos out of seven, and a dead logic analyzer, cost most of a day early on. The lesson that stuck: measure at the destination, not the source. Voltage present at the rail is not voltage present at the connector, and a conclusion reached by eliminating everything else can still be wrong.

The parser took several attempts. The bug that survived longest was a condition mixing two questions — "am I still hunting for a header?" and "is this byte a header?" — in one test. When the byte was not a header the whole branch failed and control fell through to the collecting branch, which happily started assembling a frame from the middle of the previous one. One branch per state, and each branch handles every case within it.