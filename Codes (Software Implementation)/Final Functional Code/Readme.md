<img src="https://readme-typing-svg.demolab.com?font=Orbitron&size=36&duration=3000&pause=700&color=DC2626&center=true&vCenter=true&width=1000&lines=Autonomous+Vehicle+Code" alt="Animated Title" />

## Overview

This folder contains the final functional Arduino code used for the autonomous vehicle.

The program integrates all separately developed control modules into one complete system:

* Line-following control
* Lost-line recovery
* First-obstacle detection and avoidance
* Reverse movement
* Repeated 180-degree turning for later obstacles
* Ultrasonic obstacle confirmation
* IR-based line recovery
* Motor-speed and direction control

## Integrated Operation

During normal operation, the vehicle follows the black line using the two infrared sensors and proportional-derivative correction.

When the first confirmed obstacle is detected, the line-following routine is interrupted and the obstacle-avoidance sequence is executed. After the vehicle finds the line again, normal line following resumes.

For the second and every later confirmed obstacle, the vehicle reverses and performs a controlled 180-degree turn. The infrared sensors then locate the black line before the vehicle resumes line following in the opposite direction.

## Module Switches

The final program includes module switches that allow the main functions to be enabled or disabled:

```cpp
const bool ENABLE_LINE_FOLLOWING = true;
const bool ENABLE_OBSTACLE_HANDLING = true;
```

These switches make it possible to test or disable a module without removing its code from the complete program.

## Included Modules

| Module | Function |
| --- | --- |
| Line following | Follows the black track and corrects the vehicle direction |
| Lost-line recovery | Searches in the last detected line direction |
| Obstacle detection | Measures the front distance using two ultrasonic sensors |
| Obstacle confirmation | Verifies an obstacle using several close readings |
| First-obstacle avoidance | Bypasses the first obstacle and recovers the line |
| Reverse and 180-degree turn | Handles the second and later obstacles |
| Motor control | Controls the speed and direction of both DC motors |
| Recovery control | Stops manoeuvres when the IR sensors detect the line |

## File

```text
final_functional_code.ino
```

This file represents the final integrated version used during the complete vehicle testing.
