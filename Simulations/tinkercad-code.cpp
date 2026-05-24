// ===============================
// TASK 2 - Line Following + Obstacle Avoidance
// Arduino + 2 Switches as IR Sensors + 2 Ultrasonic Sensors + 2 DC Motors
// ===============================

// ---------- Simulated IR sensors using Switches ----------
const int leftIRButton  = 4;
const int rightIRButton = 13;

// ---------- Ultrasonic sensors ----------
const int trigLeft  = 7;
const int echoLeft  = 6;

const int trigRight = 12;
const int echoRight = 11;

// ---------- Motor driver pins ----------
const int ENA_LeftMotor = 5;    // PWM speed control for left motor
const int IN1 = 3;
const int IN2 = 2;

const int ENB_RightMotor = 10;  // PWM speed control for right motor
const int IN3 = 9;
const int IN4 = 8;

// ---------- Settings ----------
const int motorSpeed = 180;     // motorSpeed = 180 => duty cycle ≈ 70%
const int turnSpeed  = 150;     // turnSpeed  = 150 => duty cycle ≈ 59%
const int obstacleLimit = 15;   // cm

void setup() {
  Serial.begin(9600);

  // Buttons simulate IR sensors
  pinMode(leftIRButton, INPUT_PULLUP);
  pinMode(rightIRButton, INPUT_PULLUP);

  // Ultrasonic sensors
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // Motor driver
  pinMode(ENA_LeftMotor, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB_RightMotor, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("System started...");
}

void loop() {
  int distanceLeft = readUltrasonic(trigLeft, echoLeft);
  int distanceRight = readUltrasonic(trigRight, echoRight);

  int leftIR = digitalRead(leftIRButton);
  int rightIR = digitalRead(rightIRButton);

  printSensorValues(distanceLeft, distanceRight, leftIR, rightIR);

  // First priority: obstacle detection
  if (distanceLeft < obstacleLimit || distanceRight < obstacleLimit) {
    avoidObstacle(distanceLeft, distanceRight);
  } 
  else {
    followLine(leftIR, rightIR);
  }

  delay(2000);
}

// ===============================
// ULTRASONIC SENSOR FUNCTION
// ===============================
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long travelTime = pulseIn(echoPin, HIGH, 30000);

  int distance = (travelTime / 2) * 0.0343;

  if (distance == 0) {
    distance = 999; // means no object detected
  }

  return distance;
}

// ===============================
// LINE FOLLOWING LOGIC
// ===============================
void followLine(int leftIR, int rightIR) {
  // INPUT_PULLUP logic:
  // Not pressed = HIGH
  // Pressed = LOW
  //
  // We use:
  // HIGH = black line 
  // LOW  = white surface

  if (leftIR == HIGH && rightIR == HIGH) {
    Serial.println("Action: Move forward");
    moveForward();
  }
  else if (leftIR == HIGH && rightIR == LOW) {
    Serial.println("Action: Turn left");
    turnLeft();
  }
  else if (leftIR == LOW && rightIR == HIGH) {
    Serial.println("Action: Turn right");
    turnRight();
  }
  else {
    Serial.println("Action: Stop / line lost");
    stopMotors();
  }
}

// ===============================
// OBSTACLE AVOIDANCE LOGIC
// ===============================
void avoidObstacle(int leftDistance, int rightDistance) {
  Serial.println("Obstacle detected!");

  stopMotors();
  delay(300);

  if (leftDistance < obstacleLimit && rightDistance >= obstacleLimit) {
    Serial.println("Obstacle on left -> Turn right");
    turnRight();
    delay(500);
  }
  else if (rightDistance < obstacleLimit && leftDistance >= obstacleLimit) {
    Serial.println("Obstacle on right -> Turn left");
    turnLeft();
    delay(500);
  }
  else {
    Serial.println("Obstacle in front -> Move back then turn right");
    moveBackward();
    delay(400);

    turnRight();
    delay(600);
  }

  stopMotors();
  delay(200);
}

// ===============================
// MOTOR CONTROL FUNCTIONS
// ===============================
void moveForward() {
  analogWrite(ENA_LeftMotor, motorSpeed);
  analogWrite(ENB_RightMotor, motorSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA_LeftMotor, motorSpeed);
  analogWrite(ENB_RightMotor, motorSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA_LeftMotor, turnSpeed);
  analogWrite(ENB_RightMotor, turnSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA_LeftMotor, turnSpeed);
  analogWrite(ENB_RightMotor, turnSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA_LeftMotor, 0);
  analogWrite(ENB_RightMotor, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ===============================
// SERIAL MONITOR OUTPUT
// ===============================
void printSensorValues(int distanceLeft, int distanceRight, int leftIR, int rightIR) {
  Serial.print("Left US: ");
  Serial.print(distanceLeft);
  Serial.print(" cm | Right US: ");
  Serial.print(distanceRight);
  Serial.print(" cm | Left IR/Button: ");
  Serial.print(leftIR == LOW ? "WHITE" : "BLACK");
  Serial.print(" | Right IR/Button: ");
  Serial.println(rightIR == LOW ? "WHITE" : "BLACK");
}
