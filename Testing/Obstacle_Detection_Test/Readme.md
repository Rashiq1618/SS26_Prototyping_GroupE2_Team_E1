<img src="https://readme-typing-svg.demolab.com?font=Orbitron&size=36&duration=3000&pause=700&color=DC2626&center=true&vCenter=true&width=1000&lines=Obstacle+Detection+Test" alt="Animated Title" />

## Overview

This folder contains the standalone ultrasonic-sensor test used during the development of the autonomous vehicle.

The test verifies that both ultrasonic sensors operate correctly, compares their distance measurements, and displays the results through the Arduino Serial Monitor.

<div align="center">
  <img
    src="obstacle_detection_test.png"
    alt="Ultrasonic obstacle-detection test"
    width="800"
  />
</div>

## Test Purpose

The test was performed to:

- Verify that both ultrasonic sensors produced valid readings.
- Compare the left and right measured distances.
- determine the useful obstacle-detection range.
- Check whether an obstacle was inside the selected threshold.
- Support the final positioning of the sensors on the chassis.
- Identify hardware faults before modifying the main control software.

Testing the sensors at different distances and angles helped determine a suitable mounting position. The sensors had to provide sufficient coverage in front of the vehicle while avoiding unnecessary detection of nearby walls or parts of the prototype.

## Detection Threshold

The test used an obstacle-detection threshold of:

```cpp
const int obstacleDistance = 25;
```

An obstacle was reported when at least one valid ultrasonic reading was equal to or below `25 cm`.

| Measured distance | Serial Monitor result |
| ---: | --- |
| `25 cm` or less | `OBSTACLE DETECTED` |
| Greater than `25 cm` | `No Obstacle` |
| Invalid reading | Ignored for obstacle detection |

## Serial Monitor Output

The program continuously displays the left and right ultrasonic readings:

```text
Left: 21 cm   |   Right: 22 cm
>>> OBSTACLE DETECTED <<<
------------------------
```

When both valid measurements are above the threshold, the output becomes:

```text
Left: 36 cm   |   Right: 36 cm
No Obstacle
------------------------
```

## Component Rechecking

Individual component tests remained useful after the complete vehicle had been assembled.

During one stage of development, the obstacle-avoidance routine did not respond correctly. The ultrasonic sensors were therefore tested separately using this program. The recheck showed that one sensor was producing unreliable measurements because of a defective cable connection.

After the cable was corrected, both sensors returned stable readings and the obstacle-handling behaviour operated normally again.

This demonstrated the importance of checking the hardware independently before assuming that unexpected vehicle behaviour is caused by the main software.

## Files

```text
obstacle_detection_test.ino
obstacle_detection_test.png
```

The Arduino file contains the standalone test, while the image shows the physical setup and Serial Monitor results.
