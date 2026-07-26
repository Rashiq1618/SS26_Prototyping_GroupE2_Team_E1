// ==================================================
// Standalone First-Obstacle Avoidance
// ==================================================
//
// This program isolates the first-obstacle avoidance
// logic from the final integrated robot program.
//
// HIGH = black detected by IR sensor
// LOW  = white detected by IR sensor
//
// The robot normally moves forward for demonstration.
// After detecting and confirming an obstacle, it:
// 1. Reverses
// 2. Selects an adaptive left pivot
// 3. Moves beside the obstacle
// 4. Turns back toward the track
// 5. Uses the IR sensors to recover the black line
// ==================================================


// ==================================================
// Motor pins
// ==================================================

const int enA = 5;
const int in1 = 3;
const int in2 = 2;

const int enB = 10;
const int in3 = 9;
const int in4 = 8;


// ==================================================
// IR sensor pins
// HIGH = BLACK, LOW = WHITE
// ==================================================

const int irLeft  = 4;
const int irRight = A0;


// ==================================================
// Ultrasonic sensor pins
// ==================================================

const int trigLeft  = 12;
const int echoLeft  = 11;

const int trigRight = 7;
const int echoRight = 6;

const unsigned long ULTRASONIC_TIMEOUT_US = 7000;


// ==================================================
// Obstacle detection values
// ==================================================

const int OBSTACLE_TRIGGER_CM = 21;

const int OBSTACLE_CONFIRM_COUNT = 3;
const int OBSTACLE_REQUIRED_CLOSE_READINGS = 2;
const int OBSTACLE_CONFIRM_DELAY_MS = 20;

const unsigned long POST_RECOVERY_IGNORE_OBSTACLE_MS = 2500;

unsigned long ignoreObstacleUntil = 0;


// ==================================================
// Demonstration movement
// ==================================================

// Used only while testing this standalone module.
// The integrated program uses line following instead.
const int TEST_FORWARD_SPEED = 80;


// ==================================================
// First-obstacle avoidance values
// ==================================================

const int AVOID_SPEED = 85;
const int AVOID_TURN_SPEED = 90;
const int BACKUP_SPEED = 80;

const int BACKUP_MS = 250;

// Three possible adaptive left turns
const int AVOID_LEFT_TURN_MS_ALIGNED = 350;
const int AVOID_LEFT_TURN_MS_RIGHT_SIDE_CLOSER = 320;
const int AVOID_LEFT_TURN_MS_LEFT_SIDE_CLOSER = 380;

// Minimum ultrasonic difference used to identify tilt
const int TILT_DIFF_CM = 4;

const int AVOID_SIDE_FORWARD_MS = 750;
const int AVOID_RIGHT_TURN_MS = 550;

const int AFTER_RIGHT_FORWARD_SPEED = 80;
const int AFTER_RIGHT_FORWARD_MS = 450;

const int SOFT_RIGHT_LEFT_SPEED = 90;
const int SOFT_RIGHT_RIGHT_SPEED = 45;
const int SOFT_RIGHT_MS = 350;


// ==================================================
// Line-recovery values
// ==================================================

const unsigned long LINE_RECOVERY_TIMEOUT_MS = 1000;

const int RECOVERY_LEFT_SPEED = 66;
const int RECOVERY_RIGHT_SPEED = 50;

const int RECOVERY_CHECK_DELAY_MS = 5;

// false: either sensor may confirm the line
// true: both sensors must detect black
const bool RECOVER_REQUIRE_BOTH_IR_BLACK = false;

const unsigned long FORCE_RIGHT_TIMEOUT_MS = 2500;

const int FORCE_RIGHT_LEFT_SPEED = 90;
const int FORCE_RIGHT_RIGHT_SPEED = 40;


// ==================================================
// Setup
// ==================================================

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

  Serial.println("Standalone obstacle-avoidance program started");
}


// ==================================================
// Main loop
// ==================================================

void loop() {
  // Ignore ultrasonic detection briefly after recovery
  // so the same obstacle is not detected again.
  if (millis() < ignoreObstacleUntil) {
    setMotorSpeed(TEST_FORWARD_SPEED, TEST_FORWARD_SPEED);
    return;
  }

  long frontDistance = readFrontDistanceCM();

  if (
    frontDistance > 0 &&
    frontDistance <= OBSTACLE_TRIGGER_CM
  ) {
    Serial.print("Initial close reading: ");
    Serial.print(frontDistance);
    Serial.println(" cm");

    stopMotors();
    delay(50);

    if (obstacleConfirmedWhileStopped()) {
      Serial.println("Obstacle confirmed");
      Serial.println("Starting avoidance sequence");

      bool recovered = avoidObstacle();

      if (recovered) {
        Serial.println("Obstacle avoided and line recovered");

        ignoreObstacleUntil =
          millis() + POST_RECOVERY_IGNORE_OBSTACLE_MS;
      }
      else {
        Serial.println("Avoidance stopped: line not recovered");
        stopMotors();

        // Keep the robot stopped after failed recovery.
        while (true) {
          stopMotors();
        }
      }

      return;
    }

    Serial.println("False obstacle reading");
  }

  // Standalone demonstration movement.
  // In the final integrated code, runLineFollower()
  // replaces this command.
  setMotorSpeed(TEST_FORWARD_SPEED, TEST_FORWARD_SPEED);
}


// ==================================================
// Confirm obstacle while stopped
// ==================================================

bool obstacleConfirmedWhileStopped() {
  int closeReadings = 0;

  for (
    int i = 1;
    i <= OBSTACLE_CONFIRM_COUNT;
    i++
  ) {
    long distanceCm = readFrontDistanceCM();

    Serial.print("Confirmation reading ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(distanceCm);
    Serial.println(" cm");

    if (
      distanceCm > 0 &&
      distanceCm <= OBSTACLE_TRIGGER_CM
    ) {
      closeReadings++;
    }

    delay(OBSTACLE_CONFIRM_DELAY_MS);
  }

  Serial.print("Confirmed close readings: ");
  Serial.println(closeReadings);

  return (
    closeReadings >=
    OBSTACLE_REQUIRED_CLOSE_READINGS
  );
}


// ==================================================
// Ultrasonic functions
// ==================================================

long readFrontDistanceCM() {
  long leftDistance =
    readDistanceCM(trigLeft, echoLeft);

  delay(5);

  long rightDistance =
    readDistanceCM(trigRight, echoRight);

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

  // React to whichever sensor sees the closer object.
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


// ==================================================
// Select adaptive left-turn duration
// ==================================================

int chooseLeftTurnTime() {
  long leftDistance =
    readDistanceCM(trigLeft, echoLeft);

  delay(5);

  long rightDistance =
    readDistanceCM(trigRight, echoRight);

  Serial.print("Tilt check — left: ");
  Serial.print(leftDistance);
  Serial.print(" cm, right: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Use the normal turn when either reading is invalid.
  if (
    leftDistance <= 0 ||
    rightDistance <= 0
  ) {
    Serial.println("Tilt unknown: normal left turn");

    return AVOID_LEFT_TURN_MS_ALIGNED;
  }

  // Right ultrasonic sensor is at least 4 cm closer.
  // The robot requires a smaller left pivot.
  if (
    rightDistance + TILT_DIFF_CM <=
    leftDistance
  ) {
    Serial.println("Right side closer: smaller left turn");

    return
      AVOID_LEFT_TURN_MS_RIGHT_SIDE_CLOSER;
  }

  // Left ultrasonic sensor is at least 4 cm closer.
  // The robot requires a larger left pivot.
  if (
    leftDistance + TILT_DIFF_CM <=
    rightDistance
  ) {
    Serial.println("Left side closer: larger left turn");

    return
      AVOID_LEFT_TURN_MS_LEFT_SIDE_CLOSER;
  }

  // Difference is smaller than 4 cm.
  Serial.println("Obstacle approximately aligned");

  return AVOID_LEFT_TURN_MS_ALIGNED;
}


// ==================================================
// Complete first-obstacle avoidance sequence
// ==================================================

bool avoidObstacle() {
  stopMotors();
  delay(150);

  // ------------------------------------------------
  // Step 1: Reverse to create turning space
  // ------------------------------------------------

  Serial.println("Step 1: Reverse");

  moveBackwardTimed(
    BACKUP_SPEED,
    BACKUP_MS
  );

  // ------------------------------------------------
  // Step 2: Adaptive left pivot
  // ------------------------------------------------

  Serial.println("Step 2: Adaptive left pivot");

  int chosenLeftTurnTime =
    chooseLeftTurnTime();

  Serial.print("Selected pivot duration: ");
  Serial.print(chosenLeftTurnTime);
  Serial.println(" ms");

  pivotLeftTimed(
    AVOID_TURN_SPEED,
    chosenLeftTurnTime
  );

  // ------------------------------------------------
  // Step 3: Move forward beside the obstacle
  // ------------------------------------------------

  Serial.println("Step 3: Move beside obstacle");

  moveForwardTimed(
    AVOID_SPEED,
    AVOID_SIDE_FORWARD_MS
  );

  // ------------------------------------------------
  // Step 4: Pivot right toward the track
  // The IR sensors are checked continuously.
  // ------------------------------------------------

  Serial.println("Step 4: Right pivot with IR checking");

  bool foundLine =
    pivotRightTimedStopOnLine(
      AVOID_TURN_SPEED,
      AVOID_RIGHT_TURN_MS
    );

  // ------------------------------------------------
  // Step 5: Forward clearance with IR checking
  // ------------------------------------------------

  if (!foundLine) {
    Serial.println("Step 5: Forward line search");

    foundLine =
      moveForwardTimedStopOnLine(
        AFTER_RIGHT_FORWARD_SPEED,
        AFTER_RIGHT_FORWARD_SPEED,
        AFTER_RIGHT_FORWARD_MS
      );
  }

  // ------------------------------------------------
  // Step 6: Soft-right curve with IR checking
  // ------------------------------------------------

  if (!foundLine) {
    Serial.println("Step 6: Soft-right line search");

    foundLine =
      moveForwardTimedStopOnLine(
        SOFT_RIGHT_LEFT_SPEED,
        SOFT_RIGHT_RIGHT_SPEED,
        SOFT_RIGHT_MS
      );
  }

  // ------------------------------------------------
  // Step 7: Gentle right recovery
  // ------------------------------------------------

  if (!foundLine) {
    Serial.println("Step 7: Gentle recovery");

    foundLine = gentleRecoverLine();
  }

  // ------------------------------------------------
  // Step 8: Forced right recovery
  // ------------------------------------------------

  if (!foundLine) {
    Serial.println("Step 8: Forced-right recovery");

    foundLine = forceRightFindLine();
  }

  stopMotors();

  return foundLine;
}


// ==================================================
// IR line-detection functions
// ==================================================

bool lineDetectedNow() {
  int leftValue = digitalRead(irLeft);
  int rightValue = digitalRead(irRight);

  return lineFoundForRecovery(
    leftValue,
    rightValue
  );
}


bool lineFoundForRecovery(
  int leftValue,
  int rightValue
) {
  if (RECOVER_REQUIRE_BOTH_IR_BLACK) {
    return (
      leftValue == HIGH &&
      rightValue == HIGH
    );
  }

  return (
    leftValue == HIGH ||
    rightValue == HIGH
  );
}


// ==================================================
// Gentle line recovery
// ==================================================

bool gentleRecoverLine() {
  unsigned long startTime = millis();

  while (
    millis() - startTime <
    LINE_RECOVERY_TIMEOUT_MS
  ) {
    if (lineDetectedNow()) {
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


// ==================================================
// Forced-right line recovery
// ==================================================

bool forceRightFindLine() {
  unsigned long startTime = millis();

  setMotorSpeed(
    FORCE_RIGHT_LEFT_SPEED,
    FORCE_RIGHT_RIGHT_SPEED
  );

  while (
    millis() - startTime <
    FORCE_RIGHT_TIMEOUT_MS
  ) {
    if (lineDetectedNow()) {
      stopMotors();
      delay(100);

      return true;
    }

    delay(RECOVERY_CHECK_DELAY_MS);
  }

  stopMotors();

  return false;
}


// ==================================================
// Movement with continuous IR checking
// ==================================================

bool moveForwardTimedStopOnLine(
  int leftSpeed,
  int rightSpeed,
  int durationMs
) {
  unsigned long startTime = millis();

  setMotorSpeed(
    leftSpeed,
    rightSpeed
  );

  while (
    millis() - startTime <
    durationMs
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


bool pivotRightTimedStopOnLine(
  int speedValue,
  int durationMs
) {
  unsigned long startTime = millis();

  setMotorSpeed(
    speedValue,
    -speedValue
  );

  while (
    millis() - startTime <
    durationMs
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


// ==================================================
// Timed movement helpers
// ==================================================

void moveForwardTimed(
  int speedValue,
  int durationMs
) {
  // Small compensation used in the final code.
  setMotorSpeed(
    speedValue * 1.2,
    speedValue
  );

  delay(durationMs);

  stopMotors();
  delay(50);
}


void moveBackwardTimed(
  int speedValue,
  int durationMs
) {
  setMotorSpeed(
    -speedValue,
    -speedValue
  );

  delay(durationMs);

  stopMotors();
  delay(50);
}


void pivotLeftTimed(
  int speedValue,
  int durationMs
) {
  setMotorSpeed(
    -speedValue,
    speedValue
  );

  delay(durationMs);

  stopMotors();
  delay(50);
}


// ==================================================
// Motor control
// ==================================================

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

  // Left motor direction
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

  // Right motor direction
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

  analogWrite(enA, abs(leftSpeed));
  analogWrite(enB, abs(rightSpeed));
}


// ==================================================
// Stop motors
// ==================================================

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
