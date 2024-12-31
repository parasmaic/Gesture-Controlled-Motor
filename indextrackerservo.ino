#include <Servo.h>

const int servoPin = 9;
Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);
}

void loop() {
  if (Serial.available()) {
    String angleStr = Serial.readStringUntil('\n');
    angleStr.trim();

    float angle = angleStr.toFloat();

    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
    } else {
      Serial.println("Angle out of range");
    }
  }
}
