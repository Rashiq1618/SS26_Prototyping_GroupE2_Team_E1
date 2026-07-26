// ========================================
// IR Line Sensor Test
// ========================================

// IR sensor pins
const int irLeft = 4;
const int irRight = A0;

// Final project configuration:
// HIGH = black
// LOW  = white
const bool BLACK_IS_LOW = false;

void setup() {
  Serial.begin(9600);

  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  Serial.println("IR Sensor Test Started");
  Serial.println("--------------------------------");
}

void loop() {
  bool leftBlack = readBlack(irLeft);
  bool rightBlack = readBlack(irRight);

  if (leftBlack && rightBlack) {
    Serial.println(
      "BLACK BLACK  --> Line Found (Centered)"
    );
  }
  else if (leftBlack && !rightBlack) {
    Serial.println(
      "BLACK WHITE  --> Line on LEFT"
    );
  }
  else if (!leftBlack && rightBlack) {
    Serial.println(
      "WHITE BLACK  --> Line on RIGHT"
    );
  }
  else {
    Serial.println(
      "WHITE WHITE  --> No Line"
    );
  }

  delay(200);
}

bool readBlack(int pin) {
  int value = digitalRead(pin);

  if (BLACK_IS_LOW) {
    return value == LOW;
  }

  return value == HIGH;
}
