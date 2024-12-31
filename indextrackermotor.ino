#include <SimpleEncoder.h>

const int motorPin1 = 7;    //in1
const int motorPin2 = 8;    //in2
const int enablePin = 9;    //ena (pwm pin)

const int BUTTON_PIN = 10;  //just for the encoder library (not actually used)
const int PINA = 2;         //encoder pin a
const int PINB = 3;         //encoder pin b
SimpleEncoder encoder(BUTTON_PIN, PINA, PINB);

float targetSpeed = 0;
float targetRPM = 0;
float currentRPM = 0;
int motorPWM = 0;
long previousPosition = 0;
unsigned long previousTime = 0;
float Kp = 1.5;  // proportional gain for speed control

const float maxRPM = 150.0;  //max motor rpm
const int MIN_PWM = 0;       //minimum pwm value

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  Serial.begin(115200);

  // set motor to initial direction
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  encoder.setValue(0);
  previousTime = millis();
}

void loop() {
  if (Serial.available() >= sizeof(float)) {
    // read target speed from serial
    Serial.readBytes((char*)&targetSpeed, sizeof(targetSpeed));
    targetRPM = (targetSpeed / 100.0) * maxRPM; // convert to rpm
  }

  unsigned long currentTime = millis();
  long currentPosition = encoder.getValue();

  float timeDiff = (currentTime - previousTime) / 60000.0;

  //calculate rpm based on encoder pulses
  currentRPM = ((currentPosition - previousPosition) / 960.0) / timeDiff; // 960 pulses/rev

  previousPosition = currentPosition;
  previousTime = currentTime;

  float speedError = targetRPM - currentRPM;

  //basic proportional control
  motorPWM = Kp * speedError;
  motorPWM = constrain(motorPWM, MIN_PWM, 255); // limit pwm to valid range

  analogWrite(enablePin, motorPWM);

  Serial.print("Target RPM: ");
  Serial.print(targetRPM);
  Serial.print(" | Current RPM: ");
  Serial.print(currentRPM);
  Serial.print(" | Motor PWM: ");
  Serial.println(motorPWM);
  Serial.print("Speed Error: ");
  Serial.println(speedError);
}
