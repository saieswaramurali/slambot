#include <Arduino.h>

int dir1v = A1; // orange
int dir2v = A0; //black
int pwm1v = 3; // right green
int pwm2v = 5; //left yellow

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Ready to receive motor speeds");

    pinMode(dir1v, OUTPUT);
    pinMode(dir2v, OUTPUT);
    pinMode(pwm1v, OUTPUT);
    pinMode(pwm2v, OUTPUT);
}

void loop() {
       if (Serial.available()) {
        String data = Serial.readStringUntil('\n');  
        data.trim();  

        Serial.print("Received String: '");
        Serial.print(data);
        Serial.println("'");

        // Split manually using ',' as delimiter
        float values[4] = {0.0, 0.0, 0.0, 0.0};
        String tempValues[4];  // Temporary storage for extracted strings
        int index = 0;

        for (int i = 0; i < data.length(); i++) {
            if (data[i] == ',') {
                index++;
                if (index >= 4) break;  // Prevent overflow
            } else {
                tempValues[index] += data[i];
            }
        }

        // Convert to float
        for (int i = 0; i < 4; i++) {
            values[i] = tempValues[i].toFloat();
        }

        Serial.println("✅ Parsed Values:");
        for (int i = 0; i < 4; i++) {
            Serial.print("Value[");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(values[i], 2);
        }

       values[0] = values[0] == 1 ? 0 : 1 ; 

     digitalWrite(dir1v, int(values[0])) ; 
     analogWrite(pwm1v, int(values[2])) ; 

     Serial.print("value 1 ") ; 
     Serial.println(int(values[1])) ; 
     Serial.print("value 3 ") ; 
     Serial.println(int(values[3])) ; 
     digitalWrite(dir2v, int(values[1])) ; 
     analogWrite(pwm2v,  int(values[3])) ; 
     delay(10) ; 

        
    }

  
}