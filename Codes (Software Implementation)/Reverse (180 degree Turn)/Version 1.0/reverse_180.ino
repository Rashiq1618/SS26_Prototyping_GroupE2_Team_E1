const bool ENABLE_LINE_FOLLOWING = true;
const bool ENABLE_OBSTACLE_HANDLING = true;

const int LEFT_MOTOR_ENABLE_PIN = 5;
const int LEFT_MOTOR_INPUT_1_PIN = 3;
const int LEFT_MOTOR_INPUT_2_PIN = 2;

const int RIGHT_MOTOR_ENABLE_PIN = 10;
const int RIGHT_MOTOR_INPUT_1_PIN = 9;
const int RIGHT_MOTOR_INPUT_2_PIN = 8;

// IR sensors: HIGH = black line, LOW = white surface.
const int LEFT_IR_SENSOR_PIN = 4;
const int RIGHT_IR_SENSOR_PIN = A0;

const int LEFT_ULTRASONIC_TRIGGER_PIN = 12;
const int LEFT_ULTRASONIC_ECHO_PIN = 11;
const int RIGHT_ULTRASONIC_TRIGGER_PIN = 7;
const int RIGHT_ULTRASONIC_ECHO_PIN = 6;

const bool USE_LEFT_ULTRASONIC_SENSOR = true;
const bool USE_RIGHT_ULTRASONIC_SENSOR = true;
const unsigned long ULTRASONIC_TIMEOUT_US = 7000;

const float LINE_FOLLOWING_BASE_SPEED = 90;
const int PID_MAX_CORRECTION = 10;

const int LINE_SEARCH_INNER_SPEED = 22;
const int LINE_SEARCH_OUTER_SPEED = 82;
const int LINE_SEARCH_NO_HISTORY_SPEED = 50;

float kp = 10.0;
float ki = 0.0;
float kd = 10.0;

float previousError = 0;
float integralError = 0;

const int OBSTACLE_DETECTION_DISTANCE_CM = 21;
const int OBSTACLE_CONFIRMATION_SAMPLE_COUNT = 3;
const int OBSTACLE_CONFIRMATION_REQUIRED_COUNT = 2;
const int OBSTACLE_CONFIRMATION_DELAY_MS = 20;

const unsigned long OBSTACLE_COOLDOWN_MS = 3000;
const unsigned long POST_RECOVERY_OBSTACLE_IGNORE_MS = 2500;

unsigned long lastObstacleDetectionTime = 0;
unsigned long obstacleIgnoreUntil = 0;

int obstacleEncounterCount = 0;
bool initialObstacleAvoided = false;
bool travelingInOppositeDirection = false;

const int OBSTACLE_BYPASS_FORWARD_SPEED = 75;
const int OBSTACLE_BYPASS_TURN_SPEED = 80;
const int REVERSE_SPEED = 70;
const int REVERSE_DURATION_MS = 250;

const int LEFT_TURN_DURATION_ALIGNED_MS = 350;
const int LEFT_TURN_DURATION_RIGHT_CLOSER_MS = 320;
const int LEFT_TURN_DURATION_LEFT_CLOSER_MS = 380;
const int ULTRASONIC_ALIGNMENT_TOLERANCE_CM = 4;

const int OBSTACLE_SIDE_FORWARD_DURATION_MS = 850;
const int OBSTACLE_RIGHT_TURN_DURATION_MS = 550;

const int POST_TURN_FORWARD_SPEED = 70;
const int POST_TURN_FORWARD_DURATION_MS = 450;

const int SOFT_RIGHT_LEFT_MOTOR_SPEED = 80;
const int SOFT_RIGHT_RIGHT_MOTOR_SPEED = 50;
const int SOFT_RIGHT_DURATION_MS = 350;

const int DEBUG_STEP_DELAY_MS = 0;

const unsigned long LINE_RECOVERY_TIMEOUT_MS = 3000;
const int LINE_RECOVERY_FORWARD_SPEED = 55;
const int LINE_RECOVERY_CHECK_DELAY_MS = 5;
const bool REQUIRE_BOTH_IR_SENSORS_FOR_RECOVERY = false;

const unsigned long FORCED_RIGHT_SEARCH_TIMEOUT_MS = 2500;
const int FORCED_RIGHT_LEFT_MOTOR_SPEED = 80;
const int FORCED_RIGHT_RIGHT_MOTOR_SPEED = 35;

const bool TURNAROUND_TO_LEFT = true;
const int TURNAROUND_SPEED = 80;
const int PRE_TURNAROUND_STOP_DURATION_MS = 250;
const unsigned long TURNAROUND_MINIMUM_DURATION_MS = 1000;
const unsigned long TURNAROUND_TIMEOUT_MS = 3200;
const int TURNAROUND_LINE_CONFIRMATION_COUNT = 2;
const unsigned long LINE_FOLLOWING_STABILIZATION_MS = 1000;

enum MotionState {
  STOPPED,
  MOVING_FORWARD,
  SEARCHING_LEFT,
  SEARCHING_RIGHT
};

MotionState currentMotionState = STOPPED;

enum LastKnownLinePosition {
  LINE_POSITION_UNKNOWN,
  LINE_POSITION_LEFT,
  LINE_POSITION_RIGHT
};

LastKnownLinePosition lastKnownLinePosition = LINE_POSITION_UNKNOWN;

void setup() {
  pinMode(LEFT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_INPUT_1_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_INPUT_2_PIN, OUTPUT);

  pinMode(RIGHT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_INPUT_1_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_INPUT_2_PIN, OUTPUT);

  pinMode(LEFT_IR_SENSOR_PIN, INPUT);
  pinMode(RIGHT_IR_SENSOR_PIN, INPUT);

  pinMode(LEFT_ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(LEFT_ULTRASONIC_ECHO_PIN, INPUT);

  pinMode(RIGHT_ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(RIGHT_ULTRASONIC_ECHO_PIN, INPUT);

  stopMotors();
}

void loop() {
  if (ENABLE_OBSTACLE_HANDLING && millis() >= obstacleIgnoreUntil) {
    long detectedDistanceCm = readFrontDistanceCm();

    if (detectedDistanceCm > 0 && detectedDistanceCm <= OBSTACLE_DETECTION_DISTANCE_CM) {

      stopMotors();
      delay(50);

      if (confirmObstacleWhileStopped()) {
        unsigned long currentTime = millis();

        if (obstacleEncounterCount == 0 ||
            currentTime - lastObstacleDetectionTime > OBSTACLE_COOLDOWN_MS) {
          lastObstacleDetectionTime = currentTime;

          if (!initialObstacleAvoided) {
            obstacleEncounterCount = 1;

            bool lineRecovered = executeInitialObstacleBypass();

            if (lineRecovered) {
              initialObstacleAvoided = true;
            }
          } else {
            obstacleEncounterCount++;

            stopMotors();
            delay(PRE_TURNAROUND_STOP_DURATION_MS);

            bool lineRecovered = performTurnaroundAndRecoverLine();

            if (lineRecovered) {
              travelingInOppositeDirection = !travelingInOppositeDirection;
            }
          }

          return;
        }
      }
    }
  }

  if (ENABLE_LINE_FOLLOWING) {
    runLineFollower();
  } else {
    stopMotors();
  }
}

bool confirmObstacleWhileStopped() {
  int closeReadingCount = 0;

  for (int sampleIndex = 1;
       sampleIndex <= OBSTACLE_CONFIRMATION_SAMPLE_COUNT;
       sampleIndex++) {
    long measuredDistanceCm = readFrontDistanceCm();

    if (measuredDistanceCm > 0 &&
        measuredDistanceCm <= OBSTACLE_DETECTION_DISTANCE_CM) {
      closeReadingCount++;
    }

    delay(OBSTACLE_CONFIRMATION_DELAY_MS);
  }

  return closeReadingCount >= OBSTACLE_CONFIRMATION_REQUIRED_COUNT;
}

long readFrontDistanceCm() {
  long leftDistanceCm = -1;
  long rightDistanceCm = -1;

  if (USE_LEFT_ULTRASONIC_SENSOR) {
    leftDistanceCm = readUltrasonicDistanceCm(
      LEFT_ULTRASONIC_TRIGGER_PIN,
      LEFT_ULTRASONIC_ECHO_PIN
    );
  }

  delay(5);

  if (USE_RIGHT_ULTRASONIC_SENSOR) {
    rightDistanceCm = readUltrasonicDistanceCm(
      RIGHT_ULTRASONIC_TRIGGER_PIN,
      RIGHT_ULTRASONIC_ECHO_PIN
    );
  }

  if (leftDistanceCm <= 0 && rightDistanceCm <= 0) {
    return -1;
  }

  if (leftDistanceCm <= 0) {
    return rightDistanceCm;
  }

  if (rightDistanceCm <= 0) {
    return leftDistanceCm;
  }

  return min(leftDistanceCm, rightDistanceCm);
}

long readUltrasonicDistanceCm(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(triggerPin, LOW);

  long pulseDurationUs = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);

  if (pulseDurationUs == 0) {
    return -1;
  }

  return pulseDurationUs * 0.0343 / 2;
}

int selectInitialLeftTurnDurationMs() {
  long leftDistanceCm = readUltrasonicDistanceCm(
    LEFT_ULTRASONIC_TRIGGER_PIN,
    LEFT_ULTRASONIC_ECHO_PIN
  );

  delay(5);

  long rightDistanceCm = readUltrasonicDistanceCm(
    RIGHT_ULTRASONIC_TRIGGER_PIN,
    RIGHT_ULTRASONIC_ECHO_PIN
  );

  if (leftDistanceCm <= 0 || rightDistanceCm <= 0) {
    return LEFT_TURN_DURATION_ALIGNED_MS;
  }

  if (rightDistanceCm + ULTRASONIC_ALIGNMENT_TOLERANCE_CM <= leftDistanceCm) {
    return LEFT_TURN_DURATION_RIGHT_CLOSER_MS;
  }

  if (leftDistanceCm + ULTRASONIC_ALIGNMENT_TOLERANCE_CM <= rightDistanceCm) {
    return LEFT_TURN_DURATION_LEFT_CLOSER_MS;
  }

  return LEFT_TURN_DURATION_ALIGNED_MS;
}

bool executeInitialObstacleBypass() {
  resetPidController();
  stopMotors();
  delay(150);

  moveBackwardForBypass(REVERSE_SPEED, REVERSE_DURATION_MS);
  applyDebugStepDelay();

  int leftTurnDurationMs = selectInitialLeftTurnDurationMs();

  pivotLeftForBypass(OBSTACLE_BYPASS_TURN_SPEED, leftTurnDurationMs);
  applyDebugStepDelay();

  moveForwardForBypass(
    OBSTACLE_BYPASS_FORWARD_SPEED,
    OBSTACLE_SIDE_FORWARD_DURATION_MS
  );
  applyDebugStepDelay();

  pivotRightForBypass(
    OBSTACLE_BYPASS_TURN_SPEED,
    OBSTACLE_RIGHT_TURN_DURATION_MS
  );
  applyDebugStepDelay();

  bool lineRecovered = moveForwardUntilLineOrTimeout(
    POST_TURN_FORWARD_SPEED,
    POST_TURN_FORWARD_SPEED,
    POST_TURN_FORWARD_DURATION_MS
  );

  if (!lineRecovered) {
    applyDebugStepDelay();

    lineRecovered = moveForwardUntilLineOrTimeout(
      SOFT_RIGHT_LEFT_MOTOR_SPEED,
      SOFT_RIGHT_RIGHT_MOTOR_SPEED,
      SOFT_RIGHT_DURATION_MS
    );
  }

  if (!lineRecovered) {
    applyDebugStepDelay();

    lineRecovered = recoverLineStraightAhead();
  }

  if (!lineRecovered) {
    applyDebugStepDelay();

    lineRecovered = searchForLineToRight();
  }

  if (lineRecovered) {
    resetPidController();
    currentMotionState = MOVING_FORWARD;

    obstacleIgnoreUntil = millis() + POST_RECOVERY_OBSTACLE_IGNORE_MS;
    return true;
  }

  stopMotors();
  currentMotionState = STOPPED;
  return false;
}

void applyDebugStepDelay() {
  if (DEBUG_STEP_DELAY_MS > 0) {
    delay(DEBUG_STEP_DELAY_MS);
  }
}

bool isRecoveryLineDetected() {
  int leftSensorValue = digitalRead(LEFT_IR_SENSOR_PIN);
  int rightSensorValue = digitalRead(RIGHT_IR_SENSOR_PIN);

  return isRecoveryLineConditionMet(leftSensorValue, rightSensorValue);
}

bool recoverLineStraightAhead() {
  unsigned long recoveryStartTime = millis();

  setMotorSpeeds(LINE_RECOVERY_FORWARD_SPEED, LINE_RECOVERY_FORWARD_SPEED);

  while (millis() - recoveryStartTime < LINE_RECOVERY_TIMEOUT_MS) {
    int leftSensorValue = digitalRead(LEFT_IR_SENSOR_PIN);
    int rightSensorValue = digitalRead(RIGHT_IR_SENSOR_PIN);

    if (isRecoveryLineConditionMet(leftSensorValue, rightSensorValue)) {
      if (leftSensorValue == HIGH && rightSensorValue == LOW) {
        lastKnownLinePosition = LINE_POSITION_LEFT;
      } else if (rightSensorValue == HIGH && leftSensorValue == LOW) {
        lastKnownLinePosition = LINE_POSITION_RIGHT;
      }

      stopMotors();
      delay(100);
      return true;
    }

    setMotorSpeeds(
      LINE_RECOVERY_FORWARD_SPEED * 1.2,
      LINE_RECOVERY_FORWARD_SPEED
    );

    delay(LINE_RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();
  return false;
}

bool searchForLineToRight() {
  unsigned long searchStartTime = millis();

  setMotorSpeeds(
    FORCED_RIGHT_LEFT_MOTOR_SPEED,
    FORCED_RIGHT_RIGHT_MOTOR_SPEED
  );

  while (millis() - searchStartTime < FORCED_RIGHT_SEARCH_TIMEOUT_MS) {
    int leftSensorValue = digitalRead(LEFT_IR_SENSOR_PIN);
    int rightSensorValue = digitalRead(RIGHT_IR_SENSOR_PIN);

    if (isRecoveryLineConditionMet(leftSensorValue, rightSensorValue)) {
      if (leftSensorValue == HIGH && rightSensorValue == LOW) {
        lastKnownLinePosition = LINE_POSITION_LEFT;
      } else if (rightSensorValue == HIGH && leftSensorValue == LOW) {
        lastKnownLinePosition = LINE_POSITION_RIGHT;
      }

      stopMotors();
      delay(100);
      return true;
    }

    setMotorSpeeds(
      FORCED_RIGHT_LEFT_MOTOR_SPEED,
      FORCED_RIGHT_RIGHT_MOTOR_SPEED
    );

    delay(LINE_RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();
  return false;
}

bool isRecoveryLineConditionMet(int leftSensorValue, int rightSensorValue) {
  if (REQUIRE_BOTH_IR_SENSORS_FOR_RECOVERY) {
    return leftSensorValue == HIGH && rightSensorValue == HIGH;
  }

  return leftSensorValue == HIGH || rightSensorValue == HIGH;
}

bool performTurnaroundAndRecoverLine() {
  resetPidController();

  stopMotors();
  delay(100);

  moveBackwardTimed(REVERSE_SPEED, REVERSE_DURATION_MS);

  unsigned long turnaroundStartTime = millis();
  bool lineRecovered = false;

  while (millis() - turnaroundStartTime < TURNAROUND_TIMEOUT_MS) {
    if (TURNAROUND_TO_LEFT) {
      setMotorSpeeds(-TURNAROUND_SPEED, TURNAROUND_SPEED);
    } else {
      setMotorSpeeds(TURNAROUND_SPEED, -TURNAROUND_SPEED);
    }

    // Ignore the departure line until the minimum turnaround time has elapsed.
    if (millis() - turnaroundStartTime >= TURNAROUND_MINIMUM_DURATION_MS) {
      if (confirmLineDetection(TURNAROUND_LINE_CONFIRMATION_COUNT)) {
        lineRecovered = true;
        break;
      }
    }

    delay(5);
  }

  stopMotors();

  if (lineRecovered) {
    updateLastKnownLinePosition();

    obstacleIgnoreUntil = millis() + POST_RECOVERY_OBSTACLE_IGNORE_MS;

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

bool confirmLineDetection(int requiredConfirmationCount) {
  int confirmedReadingCount = 0;

  for (int sampleIndex = 0;
       sampleIndex < requiredConfirmationCount;
       sampleIndex++) {
    int leftSensorValue = digitalRead(LEFT_IR_SENSOR_PIN);
    int rightSensorValue = digitalRead(RIGHT_IR_SENSOR_PIN);

    if (leftSensorValue == HIGH || rightSensorValue == HIGH) {
      confirmedReadingCount++;
    }

    delay(5);
  }

  return confirmedReadingCount >= requiredConfirmationCount;
}

void updateLastKnownLinePosition() {
  int leftSensorValue = digitalRead(LEFT_IR_SENSOR_PIN);
  int rightSensorValue = digitalRead(RIGHT_IR_SENSOR_PIN);

  if (leftSensorValue == HIGH && rightSensorValue == LOW) {
    lastKnownLinePosition = LINE_POSITION_LEFT;
  } else if (rightSensorValue == HIGH && leftSensorValue == LOW) {
    lastKnownLinePosition = LINE_POSITION_RIGHT;
  } else {
    lastKnownLinePosition = LINE_POSITION_UNKNOWN;
  }
}

void runLineFollower() {
  int leftSensorValue = digitalRead(LEFT_IR_SENSOR_PIN);
  int rightSensorValue = digitalRead(RIGHT_IR_SENSOR_PIN);

  if (leftSensorValue == HIGH && rightSensorValue == LOW) {
    lastKnownLinePosition = LINE_POSITION_LEFT;
  } else if (rightSensorValue == HIGH && leftSensorValue == LOW) {
    lastKnownLinePosition = LINE_POSITION_RIGHT;
  }

  if (leftSensorValue == LOW && rightSensorValue == LOW) {
    if (lastKnownLinePosition == LINE_POSITION_LEFT) {
      currentMotionState = SEARCHING_LEFT;
    } else if (lastKnownLinePosition == LINE_POSITION_RIGHT) {
      currentMotionState = SEARCHING_RIGHT;
    } else {
      currentMotionState = STOPPED;
    }
  } else {
    currentMotionState = MOVING_FORWARD;
  }

  executeMotionState(currentMotionState, leftSensorValue, rightSensorValue);
  delay(5);
}

void executeMotionState(
  MotionState motionState,
  int leftSensorValue,
  int rightSensorValue
) {
  switch (motionState) {
    case STOPPED:
      setMotorSpeeds(
        LINE_SEARCH_NO_HISTORY_SPEED,
        LINE_SEARCH_NO_HISTORY_SPEED
      );
      break;

    case MOVING_FORWARD: {
      int currentError = leftSensorValue - rightSensorValue;

      float derivativeError = currentError - previousError;
      integralError += currentError;

      float motorCorrection =
        kp * currentError +
        ki * integralError +
        kd * derivativeError;

      motorCorrection = constrain(
        motorCorrection,
        -PID_MAX_CORRECTION,
        PID_MAX_CORRECTION
      );

      previousError = currentError;

      int leftMotorSpeed = constrain(
        LINE_FOLLOWING_BASE_SPEED - motorCorrection,
        0,
        255
      );
      int rightMotorSpeed = constrain(
        LINE_FOLLOWING_BASE_SPEED + motorCorrection,
        0,
        255
      );

      setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
      break;
    }

    case SEARCHING_LEFT:
      setMotorSpeeds(LINE_SEARCH_INNER_SPEED, LINE_SEARCH_OUTER_SPEED);
      break;

    case SEARCHING_RIGHT:
      setMotorSpeeds(LINE_SEARCH_OUTER_SPEED, LINE_SEARCH_INNER_SPEED);
      break;
  }
}

void stabilizeLineFollower() {
  unsigned long stabilizationStartTime = millis();

  while (millis() - stabilizationStartTime < LINE_FOLLOWING_STABILIZATION_MS) {
    runLineFollower();
  }
}

void moveForwardForBypass(int speed, int durationMs) {
  setMotorSpeeds(speed * 1.2, speed);
  delay(durationMs);
  stopMotors();
  delay(50);
}

void moveForwardTimedAsymmetric(
  int leftMotorSpeed,
  int rightMotorSpeed,
  int durationMs
) {
  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
  delay(durationMs);
  stopMotors();
  delay(50);
}

bool moveForwardUntilLineOrTimeout(
  int leftMotorSpeed,
  int rightMotorSpeed,
  int timeoutMs
) {
  unsigned long movementStartTime = millis();

  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);

  while (millis() - movementStartTime < timeoutMs) {
    if (isRecoveryLineDetected()) {
      stopMotors();
      delay(100);
      return true;
    }

    delay(LINE_RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();
  delay(50);
  return false;
}

void moveBackwardForBypass(int speed, int durationMs) {
  setMotorSpeeds(-speed, -speed);
  delay(durationMs);
  stopMotors();
  delay(50);
}

void pivotLeftForBypass(int speed, int durationMs) {
  setMotorSpeeds(-speed, speed);
  delay(durationMs);
  stopMotors();
  delay(50);
}

void pivotRightForBypass(int speed, int durationMs) {
  setMotorSpeeds(speed, -speed);
  delay(durationMs);
  stopMotors();
  delay(50);
}

void moveForwardTimed(int speed, int durationMs) {
  setMotorSpeeds(speed, speed);
  delay(durationMs);
  stopMotors();
  delay(60);
}

void moveBackwardTimed(int speed, int durationMs) {
  setMotorSpeeds(-speed, -speed);
  delay(durationMs);
  stopMotors();
  delay(60);
}

void pivotRightTimed(int speed, int durationMs) {
  setMotorSpeeds(speed, -speed);
  delay(durationMs);
  stopMotors();
  delay(60);
}

void pivotLeftTimed(int speed, int durationMs) {
  setMotorSpeeds(-speed, speed);
  delay(durationMs);
  stopMotors();
  delay(60);
}

void setMotorSpeeds(int leftMotorSpeed, int rightMotorSpeed) {
  if (leftMotorSpeed > 0) {
    digitalWrite(LEFT_MOTOR_INPUT_1_PIN, LOW);
    digitalWrite(LEFT_MOTOR_INPUT_2_PIN, HIGH);
  } else if (leftMotorSpeed < 0) {
    digitalWrite(LEFT_MOTOR_INPUT_1_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_INPUT_2_PIN, LOW);
  } else {
    digitalWrite(LEFT_MOTOR_INPUT_1_PIN, LOW);
    digitalWrite(LEFT_MOTOR_INPUT_2_PIN, LOW);
  }

  if (rightMotorSpeed > 0) {
    digitalWrite(RIGHT_MOTOR_INPUT_1_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_INPUT_2_PIN, HIGH);
  } else if (rightMotorSpeed < 0) {
    digitalWrite(RIGHT_MOTOR_INPUT_1_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_INPUT_2_PIN, LOW);
  } else {
    digitalWrite(RIGHT_MOTOR_INPUT_1_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_INPUT_2_PIN, LOW);
  }

  analogWrite(LEFT_MOTOR_ENABLE_PIN, abs(leftMotorSpeed));
  analogWrite(RIGHT_MOTOR_ENABLE_PIN, abs(rightMotorSpeed));
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_ENABLE_PIN, 0);
  analogWrite(RIGHT_MOTOR_ENABLE_PIN, 0);

  digitalWrite(LEFT_MOTOR_INPUT_1_PIN, LOW);
  digitalWrite(LEFT_MOTOR_INPUT_2_PIN, LOW);

  digitalWrite(RIGHT_MOTOR_INPUT_1_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_INPUT_2_PIN, LOW);
}

void resetPidController() {
  previousError = 0;
  integralError = 0;
}
