// Define motor control pins
#define IN1 14
#define IN2 27
#define ENA 12  // PWM1

#define IN3 26
#define IN4 25
#define ENB 33  // PWM2

#define SERIAL_TIMEOUT 1000  // 1 second

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Motor controller ready.");

  // Set direction pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set PWM pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  static String inputBuffer;
  static unsigned long lastCharTime = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        float values[4];
        int parsed = sscanf(inputBuffer.c_str(), "%f,%f,%f,%f",
                            &values[0], &values[1], &values[2], &values[3]);

        if (parsed == 4) {
          int dir1 = (int)values[0];
          int dir2 = (int)values[1];
          int pwm1 = constrain((int)values[2], 0, 255);
          int pwm2 = constrain((int)values[3], 0, 255);

          // Print to Serial Monitor
          Serial.print("dir1: "); Serial.print(dir1);
          Serial.print(", dir2: "); Serial.print(dir2);
          Serial.print(", pwm1: "); Serial.print(pwm1);
          Serial.print(", pwm2: "); Serial.println(pwm2);

          // Motor A direction
          digitalWrite(IN1, dir1);
          digitalWrite(IN2, !dir1);

          // Motor B direction
          digitalWrite(IN3, dir2);
          digitalWrite(IN4, !dir2);

          // PWM output
          analogWrite(ENA, pwm1);
          analogWrite(ENB, pwm2);
        } else {
          Serial.print("Invalid input: '"); Serial.print(inputBuffer); Serial.println("'");
        }
      }
      inputBuffer = "";
      break;
    } else if (c != '\r') {
      inputBuffer += c;
      lastCharTime = millis();
    }
  }

  if (inputBuffer.length() > 0 && (millis() - lastCharTime) > SERIAL_TIMEOUT) {
    Serial.print("Timeout: discarding partial input: '");
    Serial.print(inputBuffer); Serial.println("'");
    inputBuffer = "";
  }

  delay(1); // Prevent watchdog reset
}
