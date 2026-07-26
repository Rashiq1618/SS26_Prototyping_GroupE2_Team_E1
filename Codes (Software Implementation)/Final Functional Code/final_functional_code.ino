// ===============================
// Module switches
// ===============================

const bool ENABLE_LINE_FOLLOWING = true;
const bool ENABLE_OBSTACLE_HANDLING = true;


// ===============================
// Motor pins
// ===============================

const int enA = 5;
const int in1 = 3;
const int in2 = 2;

const int enB = 10;
const int in3 = 9;
const int in4 = 8;


// ===============================
// IR sensor pins
// HIGH = BLACK, LOW = WHITE
// ===============================

const int irLeft  = 4;
const int irRight = A0;


// ===============================
// Ultrasonic sensor pins
// ===============================

const int trigLeft  = 12;
const int echoLeft  = 11;

const int trigRight = 7;
const int echoRight = 6;

const bool USE_LEFT_ULTRASONIC  = true;
const bool USE_RIGHT_ULTRASONIC = true;

const unsigned long ULTRASONIC_TIMEOUT_US = 7000;


// ===============================
// Line follower values
// Balanced for speed 135
// ===============================

const float MOTOR_SPEED = 135;
const float TURN_SPEED  = 66;

const int MAX_CORRECTION = 12;

const int searchInner = 27;
const int searchOuter = 101;
const int noMemorySpeed = 61;

float Kp = 10.0;
float Ki = 0.0;
float Kd = 10.0;

float previousError = 0;
float integral = 0;


// ===============================
// Obstacle detection values
// ===============================

const int OBSTACLE_TRIGGER_CM = 21;

const int OBSTACLE_CONFIRM_COUNT = 3;
const int OBSTACLE_REQUIRED_CLOSE_READINGS = 2;
const int OBSTACLE_CONFIRM_DELAY_MS = 20;

const unsigned long OBSTACLE_COOLDOWN_MS = 3000;

unsigned long lastObstacleTime = 0;
unsigned long ignoreObstacleUntil = 0;

const unsigned long POST_RECOVERY_IGNORE_OBSTACLE_MS = 2500;

int obstacleCount = 0;
bool firstObstacleAvoided = false;
bool returningOppositeDirection = false;


// ===============================
// First obstacle avoidance values
// ===============================

const int AVOID_SPEED = 85;
const int AVOID_TURN_SPEED = 90;
const int BACKUP_SPEED = 80;

const int BACKUP_MS = 250;

const int AVOID_LEFT_TURN_MS_ALIGNED = 350;
const int AVOID_LEFT_TURN_MS_RIGHT_SIDE_CLOSER = 320;
const int AVOID_LEFT_TURN_MS_LEFT_SIDE_CLOSER = 380;

const int TILT_DIFF_CM = 4;

const int AVOID_SIDE_FORWARD_MS = 750;
const int AVOID_RIGHT_TURN_MS = 550;

const int AFTER_RIGHT_FORWARD_SPEED = 80;
const int AFTER_RIGHT_FORWARD_MS = 450;

const int SOFT_RIGHT_LEFT_SPEED = 90;
const int SOFT_RIGHT_RIGHT_SPEED = 45;
const int SOFT_RIGHT_MS = 350;

const int DEBUG_STEP_DELAY = 0;

const unsigned long LINE_RECOVERY_TIMEOUT_MS = 1000;

const int RECOVERY_LEFT_SPEED = 66;
const int RECOVERY_RIGHT_SPEED = 50;

const int RECOVERY_CHECK_DELAY_MS = 5;

const bool RECOVER_REQUIRE_BOTH_IR_BLACK = false;

const unsigned long FORCE_RIGHT_TIMEOUT_MS = 2500;

const int FORCE_RIGHT_LEFT_SPEED = 90;
const int FORCE_RIGHT_RIGHT_SPEED = 40;


// ===============================
// Repeated 180-degree turn values
// Used for obstacle 2 and later
// ===============================

const bool TURN_180_TO_LEFT = true;

const int TURN_180_SPEED = 70;

const int BEFORE_180_STOP_MS = 250;

const unsigned long TURN_180_MIN_MS = 1000;
const unsigned long TURN_180_TIMEOUT_MS = 3200;

const int TURN_180_LINE_CONFIRM_COUNT = 2;

const unsigned long LINE_FOLLOWER_STABILIZE_MS = 1000;


// ===============================
// Movement states
// ===============================

enum MovementState {
  STOP,
  FORWARD,
  SEARCH_LEFT,
  SEARCH_RIGHT
};

MovementState currentState = STOP;


enum LastSeen {
  NONE,
  LEFT,
  RIGHT
};

LastSeen lastSeenLine = NONE;


// ===============================
// Setup
// ===============================

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  stopMotors();

  Serial.println(
    "Line follower started: first obstacle avoidance, later obstacles use 180 turn"
  );
}


// ===============================
// Main loop
// ===============================

void loop() {
  if (
    ENABLE_OBSTACLE_HANDLING &&
    millis() >= ignoreObstacleUntil
  ) {
    long firstCm = readFrontDistanceCM();

    if (
      firstCm > 0 &&
      firstCm <= OBSTACLE_TRIGGER_CM
    ) {
      Serial.print("FIRST CLOSE OBSTACLE READING: ");
      Serial.println(firstCm);

      stopMotors();
      delay(50);

      if (obstacleConfirmedWhileStopped()) {
        unsigned long now = millis();

        if (
          obstacleCount == 0 ||
          now - lastObstacleTime > OBSTACLE_COOLDOWN_MS
        ) {
          lastObstacleTime = now;

          if (!firstObstacleAvoided) {
            obstacleCount = 1;

            Serial.println("First obstacle detected");
            Serial.println("Starting obstacle avoidance");

            bool recovered = avoidObstacleBehindLogic();

            if (recovered) {
              firstObstacleAvoided = true;

              Serial.println(
                "First obstacle avoided and saved in memory"
              );
            }
          }
          else {
            obstacleCount++;

            Serial.print("Obstacle number ");
            Serial.print(obstacleCount);
            Serial.println(" detected after first avoidance");

            Serial.println(
              "Stopping, then starting 180-degree turn"
            );

            stopMotors();
            delay(BEFORE_180_STOP_MS);

            bool recovered = turnAround180AndRecoverLine();

            if (recovered) {
              returningOppositeDirection =
                !returningOppositeDirection;

              Serial.println(
                "180-degree turn complete"
              );
            }
          }

          return;
        }
      }
      else {
        Serial.println(
          "False obstacle reading. Continue line following."
        );
      }
    }
  }

  if (ENABLE_LINE_FOLLOWING) {
    runLineFollower();
  }
  else {
    stopMotors();
  }
}


// ===============================
// Confirm obstacle while stopped
// ===============================

bool obstacleConfirmedWhileStopped() {
  int closeReadings = 0;

  for (
    int i = 1;
    i <= OBSTACLE_CONFIRM_COUNT;
    i++
  ) {
    long cm = readFrontDistanceCM();

    Serial.print("Stopped confirm reading ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(cm);

    if (
      cm > 0 &&
      cm <= OBSTACLE_TRIGGER_CM
    ) {
      closeReadings++;
    }

    delay(OBSTACLE_CONFIRM_DELAY_MS);
  }

  Serial.print("Close readings while stopped: ");
  Serial.println(closeReadings);

  return (
    closeReadings >=
    OBSTACLE_REQUIRED_CLOSE_READINGS
  );
}


// ===============================
// Ultrasonic functions
// ===============================

long readFrontDistanceCM() {
  long leftDistance = -1;
  long rightDistance = -1;

  if (USE_LEFT_ULTRASONIC) {
    leftDistance =
      readDistanceCM(trigLeft, echoLeft);
  }

  delay(5);

  if (USE_RIGHT_ULTRASONIC) {
    rightDistance =
      readDistanceCM(trigRight, echoRight);
  }

  if (
    leftDistance <= 0 &&
    rightDistance <= 0
  ) {
    return -1;
  }

  if (leftDistance <= 0) {
    return rightDistance;
  }

  if (rightDistance <= 0) {
    return leftDistance;
  }

  return min(leftDistance, rightDistance);
}


long readDistanceCM(
  int trigPin,
  int echoPin
) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(
    echoPin,
    HIGH,
    ULTRASONIC_TIMEOUT_US
  );

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2;
}


// ===============================
// Choose adaptive left-turn time
// ===============================

int chooseLeftTurnTime() {
  long leftD =
    readDistanceCM(trigLeft, echoLeft);

  delay(5);

  long rightD =
    readDistanceCM(trigRight, echoRight);

  Serial.print("Tilt check - Left US: ");
  Serial.print(leftD);
  Serial.print(" cm, Right US: ");
  Serial.print(rightD);
  Serial.println(" cm");

  if (
    leftD <= 0 ||
    rightD <= 0
  ) {
    Serial.println(
      "Tilt unknown -> normal left turn"
    );

    return AVOID_LEFT_TURN_MS_ALIGNED;
  }

  if (
    rightD + TILT_DIFF_CM <= leftD
  ) {
    Serial.println(
      "Right side closer -> less left turn"
    );

    return
      AVOID_LEFT_TURN_MS_RIGHT_SIDE_CLOSER;
  }

  if (
    leftD + TILT_DIFF_CM <= rightD
  ) {
    Serial.println(
      "Left side closer -> more left turn"
    );

    return
      AVOID_LEFT_TURN_MS_LEFT_SIDE_CLOSER;
  }

  Serial.println(
    "Aligned -> normal left turn"
  );

  return AVOID_LEFT_TURN_MS_ALIGNED;
}


// ===============================
// First obstacle avoidance
// ===============================

bool avoidObstacleBehindLogic() {
  Serial.println(
    "Obstacle avoidance routine started"
  );

  resetPID();

  stopMotors();
  delay(150);

  // Step 1: Back up
  Serial.println("Step 1: Back up");

  avoidMoveBackwardTimed(
    BACKUP_SPEED,
    BACKUP_MS
  );

  debugStepPause();

  // Step 2: Adaptive left pivot
  Serial.println(
    "Step 2: Adaptive left turn"
  );

  int chosenLeftTurnTime =
    chooseLeftTurnTime();

  Serial.print("Chosen left turn time: ");
  Serial.print(chosenLeftTurnTime);
  Serial.println(" ms");

  avoidPivotLeftTimed(
    AVOID_TURN_SPEED,
    chosenLeftTurnTime
  );

  debugStepPause();

  // Step 3: Forward beside obstacle
  Serial.println(
    "Step 3: Forward beside obstacle"
  );

  avoidMoveForwardTimed(
    AVOID_SPEED,
    AVOID_SIDE_FORWARD_MS
  );

  debugStepPause();

  // Step 4: Right pivot with IR checking
  Serial.println(
    "Step 4: Right pivot with IR check"
  );

  bool foundLine =
    pivotRightTimedStopOnLine(
      AVOID_TURN_SPEED,
      AVOID_RIGHT_TURN_MS
    );

  // Step 5: Forward with IR checking
  if (!foundLine) {
    debugStepPause();

    Serial.println(
      "Step 5: Forward clearance"
    );

    foundLine =
      moveForwardCustomTimedStopOnLine(
        AFTER_RIGHT_FORWARD_SPEED,
        AFTER_RIGHT_FORWARD_SPEED,
        AFTER_RIGHT_FORWARD_MS
      );
  }

  // Step 6: Stronger soft-right curve
  if (!foundLine) {
    debugStepPause();

    Serial.println(
      "Step 6: Soft right curve"
    );

    foundLine =
      moveForwardCustomTimedStopOnLine(
        SOFT_RIGHT_LEFT_SPEED,
        SOFT_RIGHT_RIGHT_SPEED,
        SOFT_RIGHT_MS
      );
  }

  // Step 7: Gentle recovery
  if (!foundLine) {
    debugStepPause();

    Serial.println(
      "Step 7: Gentle recovery"
    );

    foundLine = borrowedRecoverLine();
  }

  // Step 8: Forced right recovery
  if (!foundLine) {
    debugStepPause();

    Serial.println(
      "Step 8: Forced right recovery"
    );

    foundLine = forceRightFindLine();
  }

  if (foundLine) {
    Serial.println(
      "Line found after obstacle"
    );

    resetPID();

    currentState = FORWARD;

    ignoreObstacleUntil =
      millis() +
      POST_RECOVERY_IGNORE_OBSTACLE_MS;

    return true;
  }

  Serial.println(
    "Line not found after obstacle"
  );

  stopMotors();

  currentState = STOP;

  return false;
}


// ===============================
// Optional debugging pause
// ===============================

void debugStepPause() {
  if (DEBUG_STEP_DELAY > 0) {
    delay(DEBUG_STEP_DELAY);
  }
}


// ===============================
// Read line during recovery
// ===============================

bool lineDetectedNow() {
  int leftValue =
    digitalRead(irLeft);

  int rightValue =
    digitalRead(irRight);

  return lineFoundForRecovery(
    leftValue,
    rightValue
  );
}


// ===============================
// Gentle recovery
// Maximum 1000 ms
// ===============================

bool borrowedRecoverLine() {
  unsigned long startTime = millis();

  while (
    millis() - startTime <
    LINE_RECOVERY_TIMEOUT_MS
  ) {
    int valueLeft =
      digitalRead(irLeft);

    int valueRight =
      digitalRead(irRight);

    if (
      lineFoundForRecovery(
        valueLeft,
        valueRight
      )
    ) {
      if (
        valueLeft == HIGH &&
        valueRight == LOW
      ) {
        lastSeenLine = LEFT;
      }
      else if (
        valueRight == HIGH &&
        valueLeft == LOW
      ) {
        lastSeenLine = RIGHT;
      }

      stopMotors();
      delay(100);

      return true;
    }

    setMotorSpeed(
      RECOVERY_LEFT_SPEED,
      RECOVERY_RIGHT_SPEED
    );

    delay(RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();

  return false;
}


// ===============================
// Forced right recovery
// ===============================

bool forceRightFindLine() {
  unsigned long startTime = millis();

  Serial.println(
    "Forced right search started"
  );

  setMotorSpeed(
    FORCE_RIGHT_LEFT_SPEED,
    FORCE_RIGHT_RIGHT_SPEED
  );

  while (
    millis() - startTime <
    FORCE_RIGHT_TIMEOUT_MS
  ) {
    int valueLeft =
      digitalRead(irLeft);

    int valueRight =
      digitalRead(irRight);

    if (
      lineFoundForRecovery(
        valueLeft,
        valueRight
      )
    ) {
      if (
        valueLeft == HIGH &&
        valueRight == LOW
      ) {
        lastSeenLine = LEFT;
      }
      else if (
        valueRight == HIGH &&
        valueLeft == LOW
      ) {
        lastSeenLine = RIGHT;
      }

      stopMotors();
      delay(100);

      Serial.println(
        "Line found during forced right search"
      );

      return true;
    }

    setMotorSpeed(
      FORCE_RIGHT_LEFT_SPEED,
      FORCE_RIGHT_RIGHT_SPEED
    );

    delay(RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();

  Serial.println(
    "Forced right search failed"
  );

  return false;
}


// ===============================
// Recovery line rule
// ===============================

bool lineFoundForRecovery(
  int valueLeft,
  int valueRight
) {
  if (RECOVER_REQUIRE_BOTH_IR_BLACK) {
    return (
      valueLeft == HIGH &&
      valueRight == HIGH
    );
  }

  return (
    valueLeft == HIGH ||
    valueRight == HIGH
  );
}


// ===============================
// 180-degree turn after obstacle 1
// ===============================

bool turnAround180AndRecoverLine() {
  resetPID();

  stopMotors();
  delay(100);

  moveBackwardTimed(
    BACKUP_SPEED,
    BACKUP_MS
  );

  unsigned long startTime = millis();

  bool foundLine = false;

  while (
    millis() - startTime <
    TURN_180_TIMEOUT_MS
  ) {
    if (TURN_180_TO_LEFT) {
      setMotorSpeed(
        -TURN_180_SPEED,
        TURN_180_SPEED
      );
    }
    else {
      setMotorSpeed(
        TURN_180_SPEED,
        -TURN_180_SPEED
      );
    }

    if (
      millis() - startTime >=
      TURN_180_MIN_MS
    ) {
      if (
        confirmBlackLine(
          TURN_180_LINE_CONFIRM_COUNT
        )
      ) {
        foundLine = true;
        break;
      }
    }

    delay(5);
  }

  stopMotors();

  if (foundLine) {
    rememberCurrentLinePosition();

    ignoreObstacleUntil =
      millis() +
      POST_RECOVERY_IGNORE_OBSTACLE_MS;

    resetPID();

    currentState = FORWARD;

    stabilizeLineFollower();

    resetPID();

    currentState = FORWARD;

    return true;
  }

  Serial.println(
    "Black line not found after 180-degree turn"
  );

  currentState = STOP;

  return false;
}


// ===============================
// Confirm black line
// ===============================

bool confirmBlackLine(
  int requiredCount
) {
  int confirmedCount = 0;

  for (
    int i = 0;
    i < requiredCount;
    i++
  ) {
    int valueLeft =
      digitalRead(irLeft);

    int valueRight =
      digitalRead(irRight);

    if (
      valueLeft == HIGH ||
      valueRight == HIGH
    ) {
      confirmedCount++;
    }

    delay(5);
  }

  return (
    confirmedCount >= requiredCount
  );
}


// ===============================
// Remember line position
// ===============================

void rememberCurrentLinePosition() {
  int valueLeft =
    digitalRead(irLeft);

  int valueRight =
    digitalRead(irRight);

  if (
    valueLeft == HIGH &&
    valueRight == LOW
  ) {
    lastSeenLine = LEFT;
  }
  else if (
    valueRight == HIGH &&
    valueLeft == LOW
  ) {
    lastSeenLine = RIGHT;
  }
  else {
    lastSeenLine = NONE;
  }
}


// ===============================
// Main line follower
// ===============================

void runLineFollower() {
  int leftIR =
    digitalRead(irLeft);

  int rightIR =
    digitalRead(irRight);

  // Remember where line was last seen
  if (
    leftIR == HIGH &&
    rightIR == LOW
  ) {
    lastSeenLine = LEFT;
  }
  else if (
    rightIR == HIGH &&
    leftIR == LOW
  ) {
    lastSeenLine = RIGHT;
  }

  // Both sensors white: search using memory
  if (
    leftIR == LOW &&
    rightIR == LOW
  ) {
    if (lastSeenLine == LEFT) {
      currentState = SEARCH_LEFT;
    }
    else if (lastSeenLine == RIGHT) {
      currentState = SEARCH_RIGHT;
    }
    else {
      currentState = STOP;
    }
  }
  else {
    currentState = FORWARD;
  }

  actOnState(
    currentState,
    leftIR,
    rightIR
  );

  delay(5);
}


// ===============================
// Apply line-following movement
// ===============================

void actOnState(
  MovementState state,
  int leftIR,
  int rightIR
) {
  switch (state) {
    case STOP:
      setMotorSpeed(
        noMemorySpeed,
        noMemorySpeed
      );
      break;

    case FORWARD: {
      int error =
        leftIR - rightIR;

      float derivative =
        error - previousError;

      integral += error;

      float correction =
        Kp * error +
        Ki * integral +
        Kd * derivative;

      correction = constrain(
        correction,
        -MAX_CORRECTION,
        MAX_CORRECTION
      );

      previousError = error;

      int leftSpeed = constrain(
        MOTOR_SPEED - correction,
        0,
        255
      );

      int rightSpeed = constrain(
        MOTOR_SPEED + correction,
        0,
        255
      );

      setMotorSpeed(
        leftSpeed,
        rightSpeed
      );

      break;
    }

    case SEARCH_LEFT:
      setMotorSpeed(
        searchInner,
        searchOuter
      );
      break;

    case SEARCH_RIGHT:
      setMotorSpeed(
        searchOuter,
        searchInner
      );
      break;
  }
}


// ===============================
// Stabilize after 180-degree turn
// ===============================

void stabilizeLineFollower() {
  unsigned long startTime = millis();

  while (
    millis() - startTime <
    LINE_FOLLOWER_STABILIZE_MS
  ) {
    runLineFollower();
  }
}


// ===============================
// Obstacle movement helpers
// ===============================

void avoidMoveForwardTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    speedValue * 1.2,
    speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(50);
}


void moveForwardCustomTimed(
  int leftSpeed,
  int rightSpeed,
  int timeMs
) {
  setMotorSpeed(
    leftSpeed,
    rightSpeed
  );

  delay(timeMs);

  stopMotors();
  delay(50);
}


// ===============================
// Forward movement with IR checking
// ===============================

bool moveForwardCustomTimedStopOnLine(
  int leftSpeed,
  int rightSpeed,
  int timeMs
) {
  unsigned long startTime = millis();

  setMotorSpeed(
    leftSpeed,
    rightSpeed
  );

  while (
    millis() - startTime <
    timeMs
  ) {
    if (lineDetectedNow()) {
      stopMotors();
      delay(100);

      return true;
    }

    delay(RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();
  delay(50);

  return false;
}


// ===============================
// Right pivot with IR checking
// ===============================

bool pivotRightTimedStopOnLine(
  int speedValue,
  int timeMs
) {
  unsigned long startTime = millis();

  setMotorSpeed(
    speedValue,
    -speedValue
  );

  while (
    millis() - startTime <
    timeMs
  ) {
    if (lineDetectedNow()) {
      stopMotors();
      delay(100);

      return true;
    }

    delay(RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();
  delay(50);

  return false;
}


void avoidMoveBackwardTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    -speedValue,
    -speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(50);
}


void avoidPivotLeftTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    -speedValue,
    speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(50);
}


void avoidPivotRightTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    speedValue,
    -speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(50);
}


// ===============================
// Original movement helpers
// ===============================

void moveForwardTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    speedValue,
    speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(60);
}


void moveBackwardTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    -speedValue,
    -speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(60);
}


void pivotRightTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    speedValue,
    -speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(60);
}


void pivotLeftTimed(
  int speedValue,
  int timeMs
) {
  setMotorSpeed(
    -speedValue,
    speedValue
  );

  delay(timeMs);

  stopMotors();
  delay(60);
}


// ===============================
// Motor control
// ===============================

void setMotorSpeed(
  int leftSpeed,
  int rightSpeed
) {
  leftSpeed = constrain(
    leftSpeed,
    -255,
    255
  );

  rightSpeed = constrain(
    rightSpeed,
    -255,
    255
  );

  if (leftSpeed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (leftSpeed < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (rightSpeed > 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (rightSpeed < 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  analogWrite(
    enA,
    abs(leftSpeed)
  );

  analogWrite(
    enB,
    abs(rightSpeed)
  );
}


// ===============================
// Stop motors
// ===============================

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


// ===============================
// Reset PID memory
// ===============================

void resetPID() {
  previousError = 0;
  integral = 0;
}
