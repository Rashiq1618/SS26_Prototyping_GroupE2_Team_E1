<img src="https://readme-typing-svg.demolab.com?font=Orbitron&size=36&duration=3000&pause=700&color=DC2626&center=true&vCenter=true&width=1000&lines=Obstacle+Avoidance" alt="Animated Title" />

<div align="center">
  <img
    src="obstacle_avoidance_gif.gif"
    alt="Autonomous vehicle avoiding an obstacle"
    width="380"
  />
</div>

## Overview

This folder contains the Arduino code for the first-obstacle avoidance logic used by the autonomous vehicle.

## Avoidance Logic

The vehicle always avoids the first obstacle toward the **left**, which corresponds to the inner side of the track. This direction was selected because the inner side provides more available space for the avoidance sequence.

### Avoidance Sequence

The obstacle-avoidance logic follows this sequence:

1. Detect a nearby obstacle using the ultrasonic sensors.
2. Stop and confirm that the obstacle is valid.
3. Reverse to create turning space.
4. Select an adaptive left-turn duration.
5. Pivot left and move beside the obstacle.
6. Pivot right toward the track.
7. Search for the black line using the IR sensors.
8. Stop the avoidance routine immediately when the line is detected.
9. Resume normal line following.

## Obstacle Detection

The two ultrasonic sensors measure the distance in front of the vehicle. The program uses the smaller valid measurement so that the robot reacts when either sensor detects a nearby obstacle.

```cpp
return min(leftDistance, rightDistance);
```

The obstacle-detection threshold is:

```cpp
OBSTACLE_TRIGGER_CM = 21;
```

When a valid distance of `21 cm` or less is detected, the robot stops before starting the confirmation process.

## Obstacle Confirmation

A single close reading is not immediately accepted as an obstacle.

After stopping, the robot takes three additional distance measurements. The obstacle is confirmed when at least two of the three readings are within the configured threshold.

| Parameter | Value |
| --- | ---: |
| Obstacle threshold | 21 cm |
| Confirmation readings | 3 |
| Required close readings | 2 |
| Delay between readings | 20 ms |

This confirmation method reduces false reactions caused by temporary echoes or unstable ultrasonic measurements.

## Adaptive Initial Turn

Before the left pivot, the robot compares the readings from the left and right ultrasonic sensors.

The configured difference is:

```cpp
TILT_DIFF_CM = 4;
```

A difference below `4 cm` is treated as approximately aligned.

Depending on the readings, the program selects one of three left-turn durations.

| Ultrasonic condition | Selected action | Turn duration |
| --- | --- | ---: |
| Difference below 4 cm | Normal left turn | 350 ms |
| Right side at least 4 cm closer | Smaller left turn | 320 ms |
| Left side at least 4 cm closer | Larger left turn | 380 ms |
| Invalid reading | Normal left turn | 350 ms |

This adaptive turn allows the robot to begin the avoidance sequence according to its alignment with the obstacle.

## Avoidance Sequence Details

### Step 1 — Reverse

The robot reverses to create sufficient turning space.

```cpp
speed = 80
duration = 250 ms
```

### Step 2 — Adaptive Left Pivot

The robot pivots left using the duration selected from the ultrasonic comparison.

```cpp
turn speed = 90
duration = 320, 350, or 380 ms
```

### Step 3 — Forward Beside the Obstacle

The robot moves forward beside the obstacle.

```cpp
speed = 85
duration = 750 ms
```

A small motor compensation is applied during this movement:

```cpp
left motor  = speed × 1.2
right motor = speed
```

### Step 4 — Right Pivot Toward the Track

The robot pivots right toward the expected track position.

```cpp
turn speed = 90
maximum duration = 550 ms
```

During this movement, the IR sensors are checked continuously. If the black line is detected, the pivot stops immediately.

### Step 5 — Forward Line Search

If the right pivot does not find the track, the robot moves forward while continuing to check the IR sensors.

```cpp
left speed = 80
right speed = 80
maximum duration = 450 ms
```

### Step 6 — Soft-Right Search

If the line is still not detected, the robot performs a soft-right curve.

```cpp
left speed = 90
right speed = 45
maximum duration = 350 ms
```

### Step 7 — Gentle Recovery

The robot performs a longer gentle-right search.

```cpp
left speed = 66
right speed = 50
maximum duration = 1000 ms
```

### Step 8 — Forced-Right Recovery

If the previous recovery stages fail, the robot performs a stronger right curve.

```cpp
left speed = 90
right speed = 40
maximum duration = 2500 ms
```

This is the final line-search stage.

## Movement Summary

| Step | Movement | Left motor | Right motor | Maximum duration | IR checked |
| ---: | --- | ---: | ---: | ---: | :---: |
| 1 | Reverse | -80 | -80 | 250 ms | No |
| 2 | Adaptive left pivot | -90 | 90 | 320–380 ms | No |
| 3 | Forward beside obstacle | Approximately 102 | 85 | 750 ms | No |
| 4 | Right pivot | 90 | -90 | 550 ms | Yes |
| 5 | Forward search | 80 | 80 | 450 ms | Yes |
| 6 | Soft-right search | 90 | 45 | 350 ms | Yes |
| 7 | Gentle recovery | 66 | 50 | 1000 ms | Yes |
| 8 | Forced-right recovery | 90 | 40 | 2500 ms | Yes |

Negative values indicate reverse motor rotation.

## Real-Time Line Detection During Recovery

The final five movements use `millis()`-based loops instead of a single blocking delay.

During each movement, the program repeatedly checks the IR sensors:

```cpp
while (millis() - startTime < maximumDuration) {
  if (lineDetectedNow()) {
    stopMotors();
    return true;
  }
}
```

This allows the movement to stop before reaching its maximum duration.

As soon as the black line is detected:

1. The motors stop.
2. The current recovery movement ends.
3. The remaining recovery steps are skipped.
4. The avoidance routine returns a successful result.
5. Normal line following resumes.

This prevents the robot from crossing the line while a timed recovery movement is still active.

## Post-Recovery Obstacle Ignore Period

After a successful avoidance, obstacle detection is temporarily ignored for:

```cpp
POST_RECOVERY_IGNORE_OBSTACLE_MS = 2500;
```

This prevents the same obstacle from being detected again immediately while the vehicle is still moving away from it.
