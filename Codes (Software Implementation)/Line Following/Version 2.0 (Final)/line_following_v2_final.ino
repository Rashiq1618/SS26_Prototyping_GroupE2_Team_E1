// Sensors output HIGH on black and LOW on white.
// Motor forward direction: input 1 LOW, input 2 HIGH.

const int LEFT_MOTOR_ENABLE_PIN = 5;
const int LEFT_MOTOR_INPUT_1_PIN = 3;
const int LEFT_MOTOR_INPUT_2_PIN = 2;

const int RIGHT_MOTOR_ENABLE_PIN = 10;
const int RIGHT_MOTOR_INPUT_1_PIN = 9;
const int RIGHT_MOTOR_INPUT_2_PIN = 8;

const int LEFT_LINE_SENSOR_PIN = 4;
const int RIGHT_LINE_SENSOR_PIN = A0;

const float BASE_MOTOR_SPEED = 110;
const int MAXIMUM_PID_CORRECTION = 10;

const int SEARCH_INNER_MOTOR_SPEED = 22;
const int SEARCH_OUTER_MOTOR_SPEED = 82;
const int NO_LINE_HISTORY_SPEED = 50;

float kp = 10.0;
float ki = 0.0;
float kd = 10.0;

float previousError = 0;
float accumulatedError = 0;

enum MovementState {
  NoLineHistory,
  FollowingLine,
  SearchingLeft,
  SearchingRight
};

MovementState currentMovementState = NoLineHistory;

enum LastDetectedLineSide {
  NoDetectedLineSide,
  DetectedOnLeft,
  DetectedOnRight
};

LastDetectedLineSide lastDetectedLineSide = NoDetectedLineSide;

void setup() {
  pinMode(LEFT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_INPUT_1_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_INPUT_2_PIN, OUTPUT);

  pinMode(RIGHT_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_INPUT_1_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_INPUT_2_PIN, OUTPUT);

  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);

  stopMotors();
}

void loop() {
  int leftSensorState = digitalRead(LEFT_LINE_SENSOR_PIN);
  int rightSensorState = digitalRead(RIGHT_LINE_SENSOR_PIN);

  if (leftSensorState == HIGH && rightSensorState == LOW) {
    lastDetectedLineSide = DetectedOnLeft;
  }
  else if (rightSensorState == HIGH && leftSensorState == LOW) {
    lastDetectedLineSide = DetectedOnRight;
  }

  if (leftSensorState == LOW && rightSensorState == LOW) {
    if (lastDetectedLineSide == DetectedOnLeft) {
      currentMovementState = SearchingLeft;
    }
    else if (lastDetectedLineSide == DetectedOnRight) {
      currentMovementState = SearchingRight;
    }
    else {
      currentMovementState = NoLineHistory;
    }
  }
  else {
    currentMovementState = FollowingLine;
  }

  executeMovementState(
    currentMovementState,
    leftSensorState,
    rightSensorState
  );

  delay(5);
}

void executeMovementState(
  MovementState movementState,
  int leftSensorState,
  int rightSensorState
) {
  switch (movementState) {
    case NoLineHistory:
      setMotorSpeeds(
        NO_LINE_HISTORY_SPEED,
        NO_LINE_HISTORY_SPEED
      );
      break;

    case FollowingLine: {
      int lineError = leftSensorState - rightSensorState;
      float errorDerivative = lineError - previousError;

      accumulatedError += lineError;

      float pidCorrection =
        kp * lineError +
        ki * accumulatedError +
        kd * errorDerivative;

      pidCorrection = constrain(
        pidCorrection,
        -MAXIMUM_PID_CORRECTION,
        MAXIMUM_PID_CORRECTION
      );

      previousError = lineError;

      int leftMotorSpeed = constrain(
        BASE_MOTOR_SPEED - pidCorrection,
        0,
        255
      );

      int rightMotorSpeed = constrain(
        BASE_MOTOR_SPEED + pidCorrection,
        0,
        255
      );

      setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
      break;
    }

    case SearchingLeft:
      setMotorSpeeds(
        SEARCH_INNER_MOTOR_SPEED,
        SEARCH_OUTER_MOTOR_SPEED
      );
      break;

    case SearchingRight:
      setMotorSpeeds(
        SEARCH_OUTER_MOTOR_SPEED,
        SEARCH_INNER_MOTOR_SPEED
      );
      break;
  }
}

void setMotorSpeeds(int leftMotorSpeed, int rightMotorSpeed) {
  if (leftMotorSpeed > 0) {
    digitalWrite(LEFT_MOTOR_INPUT_1_PIN, LOW);
    digitalWrite(LEFT_MOTOR_INPUT_2_PIN, HIGH);
  }
  else if (leftMotorSpeed < 0) {
    digitalWrite(LEFT_MOTOR_INPUT_1_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_INPUT_2_PIN, LOW);
  }
  else {
    digitalWrite(LEFT_MOTOR_INPUT_1_PIN, LOW);
    digitalWrite(LEFT_MOTOR_INPUT_2_PIN, LOW);
  }

  if (rightMotorSpeed > 0) {
    digitalWrite(RIGHT_MOTOR_INPUT_1_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_INPUT_2_PIN, HIGH);
  }
  else if (rightMotorSpeed < 0) {
    digitalWrite(RIGHT_MOTOR_INPUT_1_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_INPUT_2_PIN, LOW);
  }
  else {
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
