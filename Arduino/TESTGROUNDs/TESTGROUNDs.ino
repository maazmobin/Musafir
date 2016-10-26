#include <Encoder.h>
Encoder encL(2,4);
Encoder encR(3,5);
long encCurr1, encCurr2;

#include "MusafirMotor.h"
MusafirMotor motorL(7, 6, 9);
MusafirMotor motorR(13, 12, 10);

void setup() {
  Serial.begin(115200);
  
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);
}

void loop() {
    encCurr1 = encL.read();
    encCurr2 = -encR.read();
    encL.write(0); encR.write(0);
    Serial.print(encCurr1);
    Serial.print(", ");
    Serial.println(encCurr2);
    delay(10);
    if(Serial.available()>0){
      while(Serial.read()){
        motorL.setPWM(50);
        motorR.setPWM(50);  
      }
    }
}
