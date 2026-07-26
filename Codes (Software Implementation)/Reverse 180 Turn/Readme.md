<div align="center">

<img src="https://readme-typing-svg.demolab.com?font=Orbitron&size=36&duration=3000&pause=700&color=A855F7&center=true&vCenter=true&width=900&lines=Reverse+180%C2%B0+Turn;Autonomous+Turnaround+and+Line+Recovery" alt="Reverse 180 Turn Animated Title" />

</div>

## Overview

This Arduino implementation enables an autonomous vehicle to reverse, perform an in-place 180° turnaround, recover the black line, and resume normal line following.

The turnaround sequence is triggered after a confirmed obstacle is detected following the initial obstacle-avoidance operation.

## Turnaround Sequence

The vehicle performs the following steps:

1. Stops before starting the maneuver.
2. Reverses for a predefined duration.
3. Pivots left or right in place.
4. Ignores the original line during the initial turning period.
5. Detects and confirms the line after completing most of the turn.
6. Stops the pivot movement.
7. Resets the PID controller.
8. Resumes normal line following in the opposite direction.

## Configuration

```cpp
const int REVERSE_SPEED = 70;
const int REVERSE_DURATION_MS = 250;

const bool TURNAROUND_TO_LEFT = true;
const int TURNAROUND_SPEED = 80;

const int PRE_TURNAROUND_STOP_DURATION_MS = 250;

const unsigned long TURNAROUND_MINIMUM_DURATION_MS = 1000;
const unsigned long TURNAROUND_TIMEOUT_MS = 3200;

const int TURNAROUND_LINE_CONFIRMATION_COUNT = 2;
```

| Constant                             | Purpose                                            |
| ------------------------------------ | -------------------------------------------------- |
| `REVERSE_SPEED`                      | Motor speed used while reversing.                  |
| `REVERSE_DURATION_MS`                | Duration of the reverse movement.                  |
| `TURNAROUND_TO_LEFT`                 | Selects the turnaround direction.                  |
| `TURNAROUND_SPEED`                   | Motor speed used during the pivot.                 |
| `TURNAROUND_MINIMUM_DURATION_MS`     | Minimum time before line detection is accepted.    |
| `TURNAROUND_TIMEOUT_MS`              | Maximum allowed turnaround duration.               |
| `TURNAROUND_LINE_CONFIRMATION_COUNT` | Number of line readings required for confirmation. |

## Reverse Movement

The reverse function applies a negative speed to both motors.

```cpp
void moveBackwardTimed(int speed, int durationMs) {
  setMotorSpeeds(-speed, -speed);

  delay(durationMs);

  stopMotors();
  delay(60);
}
```

The function is called before the pivot begins:

```cpp
moveBackwardTimed(
  REVERSE_SPEED,
  REVERSE_DURATION_MS
);
```

Because both motor-speed values are negative, both motors rotate backward.

## 180° Turn and Line Recovery

```cpp
bool performTurnaroundAndRecoverLine() {
  resetPidController();

  stopMotors();
  delay(100);

  moveBackwardTimed(
    REVERSE_SPEED,
    REVERSE_DURATION_MS
  );

  unsigned long turnaroundStartTime = millis();
  bool lineRecovered = false;

  while (
    millis() - turnaroundStartTime <
    TURNAROUND_TIMEOUT_MS
  ) {
    if (TURNAROUND_TO_LEFT) {
      setMotorSpeeds(
        -TURNAROUND_SPEED,
        TURNAROUND_SPEED
      );
    } else {
      setMotorSpeeds(
        TURNAROUND_SPEED,
        -TURNAROUND_SPEED
      );
    }

    if (
      millis() - turnaroundStartTime >=
      TURNAROUND_MINIMUM_DURATION_MS
    ) {
      if (
        confirmLineDetection(
          TURNAROUND_LINE_CONFIRMATION_COUNT
        )
      ) {
        lineRecovered = true;
        break;
      }
    }

    delay(5);
  }

  stopMotors();

  if (lineRecovered) {
    updateLastKnownLinePosition();

    obstacleIgnoreUntil =
      millis() + POST_RECOVERY_OBSTACLE_IGNORE_MS;

    resetPidController();
    currentMotionState = MOVING_FORWARD;

    stabilizeLineFollower();

    resetPidController();
    currentMotionState = MOVING_FORWARD;

    return true;
  }

  currentMotionState = STOPPED;

  return false;
}
```

## Motor Direction Logic

### Reverse

```cpp
setMotorSpeeds(-speed, -speed);
```

Both motors rotate backward.

### Left Pivot

```cpp
setMotorSpeeds(
  -TURNAROUND_SPEED,
  TURNAROUND_SPEED
);
```

The left motor rotates backward while the right motor rotates forward.

### Right Pivot

```cpp
setMotorSpeeds(
  TURNAROUND_SPEED,
  -TURNAROUND_SPEED
);
```

The left motor rotates forward while the right motor rotates backward.

Using opposite motor directions allows the vehicle to rotate approximately around its own center.

## Line Confirmation

The vehicle does not stop turning after a single infrared-sensor reading. The line must be detected repeatedly before the turnaround is considered complete.

```cpp
bool confirmLineDetection(
  int requiredConfirmationCount
) {
  int confirmedReadingCount = 0;

  for (
    int sampleIndex = 0;
    sampleIndex < requiredConfirmationCount;
    sampleIndex++
  ) {
    int leftSensorValue =
      digitalRead(LEFT_IR_SENSOR_PIN);

    int rightSensorValue =
      digitalRead(RIGHT_IR_SENSOR_PIN);

    if (
      leftSensorValue == HIGH ||
      rightSensorValue == HIGH
    ) {
      confirmedReadingCount++;
    }

    delay(5);
  }

  return (
    confirmedReadingCount >=
    requiredConfirmationCount
  );
}
```

This confirmation process reduces the chance of stopping because of a temporary or incorrect sensor reading.

## Original-Line Rejection

Line detection is disabled during the minimum turnaround period:

```cpp
if (
  millis() - turnaroundStartTime >=
  TURNAROUND_MINIMUM_DURATION_MS
) {
  // Line confirmation is allowed here.
}
```

This prevents the infrared sensors from immediately detecting the original line before the vehicle has completed enough of the 180° rotation.

## Recovery Behaviour

When the line is successfully recovered, the system:

* Updates the last known line position.
* Temporarily disables obstacle detection.
* Resets accumulated PID values.
* Sets the motion state to `MOVING_FORWARD`.
* Stabilizes the line-following controller.
* Continues driving in the opposite direction.

If the line is not found before the timeout, the motors stop and the vehicle enters the `STOPPED` state.

## Key Features

* Controlled reverse movement
* Configurable left or right turnaround
* In-place 180° pivot
* Minimum turn-duration protection
* Turnaround timeout protection
* Multiple-reading line confirmation
* Automatic PID reset
* Automatic line-following recovery


