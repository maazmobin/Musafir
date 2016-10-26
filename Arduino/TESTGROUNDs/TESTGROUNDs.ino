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

int speeds[] = {0, 20, 30, 40, 50, 80, 100, 125, 150, 175, 200, 225, 250, 254, 255,
                254, 250, 225, 200, 175, 150, 125, 100, 80, 50, 40, 30, 20, 0};

void loop() {
    encCurr1 = encL.read();
    encCurr2 = -encR.read();
    encL.write(0); encR.write(0);
    delay(10);
    int currSecond = millis()/3000;
    if(currSecond>0 && currSecond<30){
      motorL.setPWM(speeds[currSecond-1]);
      motorR.setPWM(speeds[currSecond-1]);
      Serial.print(speeds[currSecond-1]);
      Serial.print(", ");
      Serial.print(encCurr1);
      Serial.print(", ");
      Serial.println(encCurr2);
    }else{
      motorL.setPWM(0);
      motorR.setPWM(0);
    }
}
