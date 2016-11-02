#include <math.h>

#include <Encoder.h>
Encoder encL(2,4);
Encoder encR(3,5);
long encCurrL, encCurrR;

#include "MusafirMotor.h"
MusafirMotor motorL(7, 6, 9);
MusafirMotor motorR(13, 12, 10);

#include "Navigator.h"
Navigator  navigator;
//from https://github.com/solderspot/NavBot/blob/master/NavBot_v1/BlankBot.h
// Navigator defines
#define WHEELBASE               nvMM(190)      // millimeters
#define WHEEL_DIAMETER          nvMM(89)     // millimeters
#define TICKS_PER_REV           1500
#define WHEEL_DIAMETER_CM       8.9           // centi-meters
#define DISTANCE_PER_TICK       (M_PI*WHEEL_DIAMETER_CM)/1500.0

// correct for systematic errors
#define WHEEL_RL_SCALER         0.97f  // Ed
#define WHEELBASE_SCALER        0.98f  // Eb
// correct distance 
#define DISTANCE_SCALER         (164.0f/150.0f)  // Es

#include <PID_v1.h>
double measuredVelL=0, measuredVelR=0;
double pwmL=0, pwmR=0;
double velL=0, velR=0;
// PID (&input, &output, &setpoint, kp, ki, kd, DIRECT/REVERSE)
PID pidL(&measuredVelL, &pwmL, &velL, 2.05,1,0, DIRECT);
PID pidR(&measuredVelR, &pwmR, &velR, 2,1,0, DIRECT);

boolean pidActive= false;

unsigned long previousMillis = 0;
int interval = 10; // in ms
int debugInterval = 100; // in ms
unsigned long debugPreviousMillis = 0;

int tempRunningTime=5000; // ms



void setup() {
  Serial.begin(115200);
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);
  
  navigator.InitEncoder( WHEEL_DIAMETER, WHEELBASE, TICKS_PER_REV );
  navigator.SetDistanceScaler( DISTANCE_SCALER );
  navigator.SetWheelbaseScaler( WHEELBASE_SCALER );
  navigator.SetWheelRLScaler( WHEEL_RL_SCALER );
  navigator.SetMinInterval(interval);
  navigator.Reset(millis());

  initPID();
  velL = 40; //cm/s for TESTING of nav Speed calculations.
  velR = 40;
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
}

void loop() {   
  pidL.Compute();
  pidR.Compute();

  if(velL>0) motorL.setPWM(pwmL);
  else motorL.setPWM(0);
  if(velR>0) motorR.setPWM(pwmR);
  else motorR.setPWM(0);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    encCurrL = encL.read(); encL.write(0); 
    encCurrR =-encR.read(); encR.write(0);
    navigator.UpdateTicks(encCurrL, encCurrR, millis());
    float distanceL = (float)encCurrL*DISTANCE_PER_TICK;
    measuredVelL = (float)distanceL*(1000.0/interval);
    float distanceR = (float)encCurrR*DISTANCE_PER_TICK;
    measuredVelR = (float)distanceR*(1000.0/interval);
  }

  if (currentMillis - debugPreviousMillis >= debugInterval && millis()<=tempRunningTime) {
    debugPreviousMillis = currentMillis;
   /* Serial.print(millis());
    Serial.print(", ");
    Serial.print(navigator.Position().x/10);
    Serial.print(", ");
    Serial.print(navigator.Position().y/10);
    Serial.print(", ");
    Serial.print(navigator.Heading());
    Serial.print(", ");
    Serial.print(navigator.TurnRate());
    Serial.print(", ");
    Serial.print(navigator.Speed()/10);
    Serial.print(", ");
    Serial.print(measuredVelL);
    Serial.print(", ");
    Serial.println(measuredVelR);*/

    String dataTX=String(int(navigator.Position().x/10))+","+String(int(navigator.Position().y/10))+","+String(int(navigator.Heading()))+","+String(navigator.TurnRate())+","+String(navigator.Speed()/10);
    Serial.println(dataTX);
  }
 else if(millis()>=tempRunningTime)
  {
      velL = 0; //cm/s for TESTING of nav Speed calculations.
  velR = 0;
  motorL.setDir(BRAKE);
  motorR.setDir(BRAKE);
    }
}

void initPID(void){
  pidL.SetMode(MANUAL); // PID CONTROL OFF
  pidR.SetMode(MANUAL);
  pidL.SetSampleTime(interval); // sample time for PID
  pidR.SetSampleTime(interval);
  pidL.SetOutputLimits(0,250);  // min/max PWM
  pidR.SetOutputLimits(0,250);  
}
