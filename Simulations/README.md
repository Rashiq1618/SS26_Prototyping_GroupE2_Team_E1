# Tinkercad Prototype Documentation

## Overview

This folder contains the first prototype simulation of the autonomous Arduino-based line-following and obstacle-avoidance vehicle developed in Tinkercad.

The simulation was created to test the system logic before implementing the real hardware prototype.

---

## Simulation Access

The full interactive simulation can be accessed via the following link:

[View Interactive Simulation](PASTE_YOUR_TINKERCAD_LINK_HERE)

---

# Repository Contents

* README.md
* Circuit image (tinkercad-circuit.png)
* Circuit schematic (tinkercad-schematic.pdf)
* Bill of materials (tinkercad-bill-of-materials.csv)
* Arduino simulation code (tinkercad-code.cpp)
* Serial monitor demonstration video (tinkercad-simulationvideo.mp4)

---

# Simulation Adjustments

## IR Sensor Simulation Using Switches

Tinkercad does not support realistic black-line surface tracking using IR sensors.

Therefore, switches were used to simulate the IR sensor outputs.

The switches represent whether the sensor detects:

* the black line
* the white surface

### Sensor Logic

| Left Sensor | Right Sensor | Detected Condition               | Vehicle Action     |
| ----------- | ------------ | -------------------------------- | ------------------ |
| HIGH        | HIGH         | Line detected under both sensors | Move forward       |
| HIGH        | LOW          | Line detected on left sensor     | Turn left          |
| LOW         | HIGH         | Line detected on right sensor    | Turn right         |
| LOW         | LOW          | No line detected                 | Stop / search line |

---

## Motor Driver Replacement

The real hardware prototype uses an L298N motor driver module.

However, the exact L298N module was not available in Tinkercad.

Therefore, an equivalent H-bridge IC motor driver simulation was used to reproduce the same motor control behavior.

The simulation still supports:

* forward movement
* left/right turning
* motor direction control
* PWM speed control

---

# Obstacle Detection Logic

Two ultrasonic sensors were implemented to simulate obstacle detection and avoidance.

When an obstacle is detected below the defined threshold distance:

1. The vehicle stops.
2. The system compares left and right free space.
3. The vehicle turns toward the clearer side.
4. The system searches for the black line again.
5. The vehicle resumes line following.

---

# Key Note

This simulation represents the first functional prototype of the system. It is used to validate decision-making logic before transitioning to the physical hardware implementation.

