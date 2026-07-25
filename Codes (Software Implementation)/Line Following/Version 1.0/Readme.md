# Line Following
The car uses two IR sensors to detect the line and adjusts the left and right motor speeds accordingly. It also remembers the last detected direction to help recover when both sensors lose the line.

## Features

* Two IR sensor inputs
* Independent left and right motor control
* Adjustable straight, gentle-turn, and sharp-turn speeds
* Sensor sampling to reduce unstable readings
* Lost-line recovery
* Configurable sensor polarity and steering direction

## Control Method

This implementation uses simple rule-based motor speed control.
**PID control is not used.**
The steering response is determined directly from the states of the left and right IR sensors.

## Main Configuration

The following settings can be adjusted at the top of the Arduino file:

* Motor and sensor pin assignments
* Driving and turning speeds
* Sensor polarity
* Steering inversion
* Sensor and motor swapping
* Sensor sampling delay
