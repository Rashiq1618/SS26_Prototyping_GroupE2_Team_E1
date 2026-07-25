// IR sensor pins
const int leftIrSensorPin = 4;
const int rightIrSensorPin = A0;

// Left motor driver pins
const int leftMotorEnablePin = 5;
const int leftMotorInput1Pin = 3;
const int leftMotorInput2Pin = 2;

// Right motor driver pins
const int rightMotorEnablePin = 10;
const int rightMotorInput1Pin = 9;
const int rightMotorInput2Pin = 8;

// Speed settings
const int straightSpeed = 55;
const int gentleTurnInnerSpeed = 35;
const int gentleTurnOuterSpeed = 85;
const int sharpTurnInnerSpeed = 25;
const int sharpTurnOuterSpeed = 100;
const int lineSearchSpeed = 30;

// Sensor and hardware settings
const bool blackIsLow = false;
const bool invertSteering = true;
const bool swapSensors = false;
const bool swapMotors = false;

// Timing settings
const unsigned long loopDelayMs = 5;
const int sensorSampleCount = 3;
const unsigned int sensorSampleDelayUs = 200;

enum Direction {
  directionNone,
  directionForward,
  directionLeft,
  directionRight
};

Direction lastDetectedDirection = directionForward;

void setup() {
  Serial.begin(9600);

  pinMode(leftIrSensorPin, INPUT);
  pinMode(rightIrSensorPin, INPUT);

  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorInput1Pin, OUTPUT);
  pinMode(leftMotorInput2Pin, OUTPUT);

  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorInput1Pin, OUTPUT);
  pinMode(rightMotorInput2Pin, OUTPUT);

  stopMotors();
  delay(1000);

  Serial.println("Line follower started");
}

void loop() {
  followLine();
  delay(loopDelayMs);
}

void followLine() {
  bool leftSensorDetectsBlack = readBlackSurface(leftIrSensorPin);
  bool rightSensorDetectsBlack = readBlackSurface(rightIrSensorPin);

  if (swapSensors) {
    const bool temporarySensorValue = leftSensorDetectsBlack;
    leftSensorDetectsBlack = rightSensorDetectsBlack;
    rightSensorDetectsBlack = temporarySensorValue;
  }

  if (leftSensorDetectsBlack && rightSensorDetectsBlack) {
    lastDetectedDirection = directionForward;
    driveForward(straightSpeed, straightSpeed);
  } else if (leftSensorDetectsBlack && !rightSensorDetectsBlack) {
    lastDetectedDirection = directionLeft;

    if (invertSteering) {
      turnRightGently();
    } else {
      turnLeftGently();
    }
  } else if (!leftSensorDetectsBlack && rightSensorDetectsBlack) {
    lastDetectedDirection = directionRight;

    if (invertSteering) {
      turnLeftGently();
    } else {
      turnRightGently();
    }
  } else {
    recoverLostLine();
  }
}

void recoverLostLine() {
  if (lastDetectedDirection == directionLeft) {
    if (invertSteering) {
      turnRightSharply();
    } else {
      turnLeftSharply();
    }
  } else if (lastDetectedDirection == directionRight) {
    if (invertSteering) {
      turnLeftSharply();
    } else {
      turnRightSharply();
    }
  } else {
    driveForward(lineSearchSpeed, lineSearchSpeed);
  }
}

bool readBlackSurface(int sensorPin) {
  int blackDetectionCount = 0;

  for (int sampleIndex = 0;
       sampleIndex < sensorSampleCount;
       sampleIndex++) {
    const int sensorValue = digitalRead(sensorPin);

    if (sensorValueRepresentsBlack(sensorValue)) {
      blackDetectionCount++;
    }

    delayMicroseconds(sensorSampleDelayUs);
  }

  return blackDetectionCount > sensorSampleCount / 2;
}

bool sensorValueRepresentsBlack(int sensorValue) {
  if (blackIsLow) {
    return sensorValue == LOW;
  }

  return sensorValue == HIGH;
}

void turnLeftGently() {
  driveForward(
    gentleTurnInnerSpeed,
    gentleTurnOuterSpeed
  );
}

void turnRightGently() {
  driveForward(
    gentleTurnOuterSpeed,
    gentleTurnInnerSpeed
  );
}

void turnLeftSharply() {
  driveForward(
    sharpTurnInnerSpeed,
    sharpTurnOuterSpeed
  );
}

void turnRightSharply() {
  driveForward(
    sharpTurnOuterSpeed,
    sharpTurnInnerSpeed
  );
}

void driveForward(int leftMotorSpeed, int rightMotorSpeed) {
  setLeftMotor(leftMotorSpeed, true);
  setRightMotor(rightMotorSpeed, true);
}

void setLeftMotor(int motorSpeed, bool moveForward) {
  if (swapMotors) {
    setMotor(
      rightMotorEnablePin,
      rightMotorInput1Pin,
      rightMotorInput2Pin,
      motorSpeed,
      moveForward
    );
  } else {
    setMotor(
      leftMotorEnablePin,
      leftMotorInput1Pin,
      leftMotorInput2Pin,
      motorSpeed,
      moveForward
    );
  }
}

void setRightMotor(int motorSpeed, bool moveForward) {
  if (swapMotors) {
    setMotor(
      leftMotorEnablePin,
      leftMotorInput1Pin,
      leftMotorInput2Pin,
      motorSpeed,
      moveForward
    );
  } else {
    setMotor(
      rightMotorEnablePin,
      rightMotorInput1Pin,
      rightMotorInput2Pin,
      motorSpeed,
      moveForward
    );
  }
}

void setMotor(
  int enablePin,
  int input1Pin,
  int input2Pin,
  int requestedSpeed,
  bool moveForward
) {
  const int constrainedSpeed = constrain(requestedSpeed, 0, 255);

  analogWrite(enablePin, constrainedSpeed);

  if (moveForward) {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, HIGH);
  } else {
    digitalWrite(input1Pin, HIGH);
    digitalWrite(input2Pin, LOW);
  }
}

void stopMotors() {
  analogWrite(leftMotorEnablePin, 0);
  analogWrite(rightMotorEnablePin, 0);

  digitalWrite(leftMotorInput1Pin, LOW);
  digitalWrite(leftMotorInput2Pin, LOW);

  digitalWrite(rightMotorInput1Pin, LOW);
  digitalWrite(rightMotorInput2Pin, LOW);
}
