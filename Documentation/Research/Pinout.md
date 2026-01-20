# Pinout information for the project 

---

**Project:** Grabby\
**Author:** Julius Ortstadt\
**Date:** 19.01.2026\
**Description:** Pinout information for the project.

---

## Arduino Nano Pinout for Motors

### Motor 1

| Signal      | Function / Description        | Arduino Nano Pin |
|-------------|-------------------------------|------------------|
| IN1         | Motor direction control       | D4               |
| IN2         | Motor direction control       | D7               |
| ENA         | Motor enable (PWM)            | D5 (PWM)         |
| Encoder A   | Encoder channel A (Interrupt) | D2               |
| Encoder B   | Encoder channel B             | D8               |
| Encoder GND | Ground                        | GND              |
| Encoder VCC | Power                         | +5V              |

### Motor 2

| Signal      | Function / Description        | Arduino Nano Pin |
|-------------|-------------------------------|------------------|
| IN3         | Motor direction control       | D9               |
| IN4         | Motor direction control       | D10              |
| ENB         | Motor enable (PWM)            | D6 (PWM)         |
| Encoder A   | Encoder channel A (Interrupt) | D3               |
| Encoder B   | Encoder channel B             | D11              |
| Encoder GND | Ground                        | GND              |
| Encoder VCC | Power                         | +5V              |


## Arduino Nano Pinout for Power Information

### Screen

| Signal | Description        | Arduino Nano Pin |
|--------|--------------------|------------------|
| GND    | Ground             | GND              |
| VCC    | Power              | 5V               |
| SDA    | I2C Data           | A4               |
| SCL    | I2C Clock          | A5               |

### Voltage Sensor – 12V

| Signal | Description        | Arduino Nano Pin |
|--------|--------------------|------------------|
| GND    | Ground             | GND              |
| IN     | Voltage input      | 12V              |
| OUT    | Analog output      | A1               |

### Voltage Sensor – 5V

| Signal | Description        | Arduino Nano Pin |
|--------|--------------------|------------------|
| GND    | Ground             | GND              |
| IN     | Voltage input      | 5V               |
| OUT    | Analog output      | A0               |

### Current Sensor – 12V

| Signal | Description        | Arduino Nano Pin |
|--------|--------------------|------------------|
| GND    | Ground             | GND              |
| VCC    | Power              | 5V               |
| OUT    | Analog output      | A3               |

### Current Sensor – 5V

| Signal | Description        | Arduino Nano Pin |
|--------|--------------------|------------------|
| GND    | Ground             | GND              |
| VCC    | Power              | 5V               |
| OUT    | Analog output      | A2               |
