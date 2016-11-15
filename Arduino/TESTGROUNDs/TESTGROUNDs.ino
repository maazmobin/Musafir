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
#define WHEEL_RL_SCALER         0.98f  // Ed
#define WHEELBASE_SCALER        1.01f  // Eb
// correct distance 
#define DISTANCE_SCALER         (118.0f/120.0f)  // Es

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

float myAngle=0;

float angleThreshold = 0.02; //0.1=10% of 3.142
float distanceThreshold = 3 ;  //cm

float errorAngle = 0 , angleFollow = 1.57 , recvAngle=0 , distanceFollow = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;

float xTraj=0,yTraj=0,xCurr=0,yCurr=0;
int following=0;

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
  velL = 00; //cm/s for TESTING of nav Speed calculations.
  velR = 00;
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  
  inputString.reserve(200);
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
    distanceL=abs(distanceL);
    measuredVelL = (float)distanceL*(1000.0/interval);
    float distanceR = (float)encCurrR*DISTANCE_PER_TICK;
    distanceR=abs(distanceR);
    measuredVelR = (float)distanceR*(1000.0/interval);
  }

  if (currentMillis - debugPreviousMillis >= debugInterval) {
    debugPreviousMillis = currentMillis;
    String dataTX=String(int(navigator.Position().x/10))+","+String(int(navigator.Position().y/10))+","+String(navigator.Heading())+","+String(navigator.TurnRate())+","+String(navigator.Speed()/10);
    Serial.println(dataTX);
  }
  pathFollowing();
}

void pathFollowing(void)
{
    xCurr=int(navigator.Position().x/10);
    yCurr=int(navigator.Position().y/10);
    int c1,c2;
  if (stringComplete) 
    {
    if(inputString.indexOf(',')>=1){
        c1 = inputString.indexOf(',')+1;
        xTraj = inputString.substring(0,c1).toInt();
        yTraj = inputString.substring(c1).toInt();
        following=1;
      }
      else
      {
        recvAngle=inputString.toFloat();
        following=2;
        }
    inputString = "";
    stringComplete = false;
  }
  
  if(following==1)    {angleFollow=atan2(yTraj-yCurr,xTraj-xCurr);distanceFollow=sqrt  (sq(yTraj-yCurr)  +   sq(xTraj-xCurr) );}
  else if(following == 2 && recvAngle <= 3.2 && recvAngle >= -3.2)    { angleFollow=recvAngle ; distanceFollow=0 ; }
  
    errorAngle=angleFollow-navigator.Heading();
    errorAngle=atan2(sin(errorAngle),cos(errorAngle));
    
    if(errorAngle>=(3.142*angleThreshold))
    {
      VelR(20);      
      VelL(-20);
      }
    else if(errorAngle<=(-3.142*angleThreshold))
    {
      VelR(-20);      
      VelL(20);
      }
    else if(distanceFollow>=distanceThreshold ||distanceFollow<=(-1*distanceThreshold))
    {
      VelR(30);      
      VelL(30);
      }
      else
    {
      VelR(0);      
      VelL(0);
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
void VelL(int velocity)
{
  if(velocity>=0){velL=velocity;motorL.setDir(FORWARD);}
  else if(velocity<=0){velL=(-1*velocity);motorL.setDir(BACKWARD);}
  }
void VelR(int velocity)
{
  if(velocity>=0){velR=velocity;motorR.setDir(FORWARD);}
  else if(velocity<=0){velR=(-1*velocity);motorR.setDir(BACKWARD);}
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
