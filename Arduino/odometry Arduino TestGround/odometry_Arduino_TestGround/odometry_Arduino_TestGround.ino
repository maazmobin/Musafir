const int bufferSize = 100;
int circularBuffer1[bufferSize];
int circularBuffer2[bufferSize];
int smoothing1=0;
int smoothing2=0;

/*#define trajArraySize 6
int trajBuffer[trajArraySize][2] = {{0 , 40},{-40 , 40},{-80 , 80},{-80 , 120},{-40 , 160},{-80 , 200}};*/
#define trajArraySize 4
int trajBuffer[trajArraySize][2] = {{0 , 40},{-80 , 80},{-40 , 160},{-80 , 200}};
int currentIndex = 0;

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
#define WHEELBASE               nvMM(189)      // millimeters
#define WHEEL_DIAMETER          nvMM(89)      // millimeters
#define TICKS_PER_REV           1520          //ROBOT 1
#define WHEEL_DIAMETER_CM       8.9           // centi-meters
#define WHEELBASE_CM            18.9
#define DISTANCE_PER_TICK       (M_PI*WHEEL_DIAMETER_CM)/((float)TICKS_PER_REV)

// correct for systematic errors
#define WHEEL_RL_SCALER         1.0f  // Ed
#define WHEELBASE_SCALER        1.0f  // Eb
// correct distance 
#define DISTANCE_SCALER         1.0f  // Es

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

float errorAngle = 0 , angleFollow = 1.57 , recvAngle=0 , distanceFollow = 0 , distanceFollow_2 = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;

float xTraj=0,yTraj=0,xCurr=0,yCurr=0;
float xTraj_2=0,yTraj_2=0;
int following=0;

float kpW = 15 , kpV = 5 , w = 0 , v = 0 , vl = 0 , vr = 0 ;
float velDiff=0;

#define maxVelocity 60

void setup() {
  Serial.begin(115200);
  initSerialBufferArray();
  initCircularBufferArray();
  
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
  if (distanceFollow == 0 && currentIndex<trajArraySize) 
    {
    xTraj=trajBuffer[currentIndex][0];
    yTraj=trajBuffer[currentIndex][1];
    currentIndex ++ ;
  }
  if (currentIndex<trajArraySize)
  {
 distanceFollow_2=40;}
 else {distanceFollow_2=0;}
  
    angleFollow=atan2(yTraj-yCurr,xTraj-xCurr);
    distanceFollow=sqrt  (sq(yTraj-yCurr)  +   sq(xTraj-xCurr) );
    
    errorAngle=angleFollow-navigator.Heading();
    errorAngle=atan2(sin(errorAngle),cos(errorAngle));
    
    if(errorAngle<=0.04 && errorAngle>=(-0.04) )
    errorAngle=0;
    if(distanceFollow<=4)
    {distanceFollow=0; errorAngle=0;}
  //  w=constrain(w,-0.5*v,0.5*v);
    v=kpV*(distanceFollow+distanceFollow_2);
    w=kpW*errorAngle;
    vl=(2*v)-(w*WHEELBASE_CM);
    vl/=(2*WHEEL_DIAMETER_CM);
    vr=(2*v)+(w*WHEELBASE_CM);
    vr/=(2*WHEEL_DIAMETER_CM);
                              //CAR DRIVE

   if( vl >= vr && vl >= maxVelocity )
   {
    velDiff=(maxVelocity/vl); //in %
    vl=maxVelocity;
    vr=vr*velDiff;
    }
   else if( vr >= vl && vr >= maxVelocity )
   {
    velDiff=(maxVelocity/vr); //in %
    vr=maxVelocity;
    vl=vl*velDiff;
    }
    
    

    
   /* smoothing1 -= circularBuffer1[bufferSize-1];
    smoothing2 -= circularBuffer2[bufferSize-1];;
    for(int i=bufferSize-1 ; i>=1 ; i--)
    { 
    circularBuffer1[i] = circularBuffer1[i-1] ;
    circularBuffer2[i] = circularBuffer2[i-1] ;
       }
   circularBuffer1[0] = vr;
   smoothing1 += circularBuffer1[0];
   circularBuffer2[0] = vl;
   smoothing2 += circularBuffer2[0];
   vr=smoothing1/bufferSize;
   vl=smoothing2/bufferSize;
   
   */
   if(vl>0 && vl<=10)
   vl=10;
   else if(vl>=-10 && vl<0)
   vl=-10;
   
   if(vr>0 && vr<=10)
   vr=10;
   else if(vr>=-10 && vr<0)
   vr=-10;
   
   VelL(vl);
   VelR(vr);
  // Serial.println(String(vr)+","+String(vl));
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

void initSerialBufferArray (void)   //Initializing Zero in the Serial Buffer Array
{
/*  for(int i =0 ; i < SerialBufferSize ; i++)
  {
    for(int j =0 ; j < 2 ; j++)
    {  
      SerialFifoBuffer[i][j]= 0 ;
    }  
  }*/
}
void initCircularBufferArray (void)   //Initializing Zero in the Serial Buffer Array
{
    for(int i =0 ; i < bufferSize ; i++)
  {
    circularBuffer1[i]=0;
    circularBuffer2[i]=0;
  }
}
