// ========================================
// Ultrasonic Obstacle Detection Test
// ========================================

// Left ultrasonic sensor
const int trigLeft = 12;
const int echoLeft = 11;

// Right ultrasonic sensor
const int trigRight = 7;
const int echoRight = 6;

// Test threshold used during component testing
const int obstacleDistance = 25;  // cm

const unsigned long ultrasonicTimeout = 7000;

void setup() {
  Serial.begin(9600);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  Serial.println("Obstacle Detection Test Started");
  Serial.println("--------------------------------");
}

void loop() {
  long leftDistance = readDistance(trigLeft, echoLeft);

  // Small delay prevents interference between sensors
  delay(5);

  long rightDistance = readDistance(trigRight, echoRight);

  Serial.print("Left: ");

  if (leftDistance > 0) {
    Serial.print(leftDistance);
    Serial.print(" cm");
  } else {
    Serial.print("No reading");
  }

  Serial.print("   |   Right: ");

  if (rightDistance > 0) {
    Serial.print(rightDistance);
    Serial.println(" cm");
  } else {
    Serial.println("No reading");
  }

  bool leftObstacle =
    leftDistance > 0 &&
    leftDistance <= obstacleDistance;

  bool rightObstacle =
    rightDistance > 0 &&
    rightDistance <= obstacleDistance;

  if (leftObstacle || rightObstacle) {
    Serial.println(">>> OBSTACLE DETECTED <<<");
  } else {
    Serial.println("No Obstacle");
  }

  Serial.println("------------------------");

  delay(300);
}

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(
    echoPin,
    HIGH,
    ultrasonicTimeout
  );

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2;
}
