# stm32-lidar-scanner

A two-axis pan-tilt LiDAR scanner built on an STM32F446RE Nucleo board. Two servos sweep a TF-Luna time-of-flight rangefinder across a hemisphere; the board reads distance over UART and streams telemetry to a PC.

Written bare-metal in C with STM32 HAL. No Arduino libraries.

## Status

Working:

- Dual-axis servo control on a single timer, 50 Hz PWM, verified on a logic analyzer
- Non-blocking timing so both axes and the sensor run independently
- Smooth homing sequence on reset
- TF-Luna driver: interrupt-driven UART receive, frame parser with checksum validation
- Distance, signal validity and sensor temperature exposed to the application
- Telemetry over USART2 to a serial terminal

Next:

- Mechanical assembly and mounting
- Tagging each distance reading with the angle it was taken at
- FreeRTOS
- Closed-loop tracking

## Hardware

| Part | Notes |
|---|---|
| NUCLEO-F446RE | STM32F446RE, 180 MHz Cortex-M4 |
| 2 × MG90S servo | Metal gear, 4.8–6.0 V |
| Benewake TF-Luna | 0.2–8 m, 2° FOV, UART at 115200 |
| Buck converter | 12 V in, 5.07 V out — the Luna's ceiling is 5.2 V with no over-voltage protection |

### Pin assignment

| Signal | Pin | Peripheral |
|---|---|---|
| Pan servo | PA1 | TIM2_CH2 |
| Tilt servo | PA0 | TIM2_CH1 |
| LiDAR TX → MCU RX | PA10 | USART1_RX |
| MCU TX → LiDAR RX | PA9 | USART1_TX |
| PC telemetry | PA2 / PA3 | USART2 (ST-Link virtual COM port) |

## How it works

### Servo control

TIM2 is clocked at 90 MHz from APB1. A prescaler of 89 divides it to 1 MHz, giving one timer tick per microsecond. ARR is 19999, so the counter wraps every 20 000 ticks — a 20 ms frame at 50 Hz, which is what hobby servos expect.

Because one tick is one microsecond, the compare register holds the pulse width in microseconds directly:

```
CCR = 1000 + (angle / 180) * 1000
```

Both channels share the timer's prescaler and period but have independent compare registers, so one timer drives both axes. Their pulses rise simultaneously and fall at different points.

### LiDAR

The TF-Luna free-runs at 100 Hz, pushing a nine-byte frame:

```
59 59  Dist_L Dist_H  Amp_L Amp_H  Temp_L Temp_H  Checksum
```

Bytes arrive one at a time via interrupt. A small state machine hunts for the `0x59 0x59` sync word, collects the remaining seven bytes, and verifies the checksum — the low eight bits of the sum of bytes 0 through 7 — before decoding.

Distance is little-endian across two bytes. A reading is treated as invalid when amplitude is below 100, when amplitude reads 65535 (overexposure), or when the distance falls outside the sensor's trustworthy 20–800 cm range. The application checks validity before using a reading; a distance of 0 means the sensor could not measure, not that nothing is there.

### Timing

Everything runs off one non-blocking helper:

```c
bool elapsed(uint32_t *last, uint32_t interval);
```

It takes a pointer so it can update the caller's timestamp, which lets each timed activity keep its own independent schedule in the same main loop. No blocking delays anywhere in normal operation.

## Layout

```
Core/
  Inc/
    servo.h       pan and tilt control interface
    tfluna.h      LiDAR interface
  Src/
    main.c        application loop and timing
    servo.c       PWM generation, angle limits, homing
    tfluna.c      UART receive, frame parser, accessors
cmake/
  files.cmake     source list
```

Each module keeps its state private and exposes only what callers need. The frame parser and the UART callback are not in `tfluna.h` — nothing outside the driver has any business feeding bytes to it.

## Building

Requires STM32CubeCLT and the STM32 VS Code extension.

```
cmake --preset Debug
cmake --build build/Debug
```

Peripheral configuration is in `Iron_Dome.ioc` and edited with STM32CubeMX. Regenerating preserves everything inside the `USER CODE` markers.

