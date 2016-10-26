#include <math.h>
#include <Encoder.h>
Encoder encL(2,4);
Encoder encR(3,5);
long encCurrL, encCurrR;

#include "MusafirMotor.h"
MusafirMotor motorL(7, 6, 9);
MusafirMotor motorR(13, 12, 10);

double spdL, spdR;
#define WHEEL_DIAMETER_CM    9.15  // centimeter
#define TICKS_PER_REV     1500
#define DISTANCE_PER_TICK (float)((M_PI*WHEEL_DIAMETER_CM)/TICKS_PER_REV)

int interval=100;

#include "Navigator.h"
Navigator  navigator;
//from https://github.com/solderspot/NavBot/blob/master/NavBot_v1/BlankBot.h
// Navigator defines
#define WHEELBASE               nvMM(190)      // millimeters
#define WHEEL_DIAMETER          nvMM(91.5)     // millimeters

// correct for systematic errors
#define WHEEL_RL_SCALER         1.0f  // Ed
#define WHEELBASE_SCALER        1.0f  // Eb
// correct distance 
#define DISTANCE_SCALER         1.0f  // Es

void setup() {
  Serial.begin(115200);
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);
  
  navigator.InitEncoder( WHEEL_DIAMETER, WHEELBASE, TICKS_PER_REV );
  navigator.SetDistanceScaler( DISTANCE_SCALER );
  navigator.SetWheelbaseScaler( WHEELBASE_SCALER );
  navigator.SetWheelRLScaler( WHEEL_RL_SCALER );
  navigator.Reset(millis());  
}

int speeds[] = {0, 20, 30, 40, 50, 80, 100, 125, 150, 175, 200, 225, 250, 254, 255,
                254, 250, 225, 200, 175, 150, 125, 100, 80, 50, 40, 30, 20, 0};

void loop() {
    encCurrL = encL.read();
    encCurrR =-encR.read();
    navigator.UpdateTicks(encCurrL, encCurrR, millis());
    encL.write(0); encR.write(0);
    int currSecond = millis()/1000;
    if(currSecond>0 && currSecond<30){
      motorL.setPWM(50);
      motorR.setPWM(50);
      Serial.print(millis());
      Serial.print(", ");
      Serial.print(navigator.Position().x/10);
      Serial.print(", ");
      Serial.print(navigator.Position().y/10);
      Serial.print(", ");
      Serial.print(navigator.Heading());
      Serial.print(", ");
      Serial.print(navigator.TurnRate());
      Serial.print(", ");
      Serial.println(navigator.Speed()/10);
    }else{
      motorL.setPWM(0);
      motorR.setPWM(0);
    }
    delay(interval);
}
