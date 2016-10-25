#define DEBUG 1
#define MAGICADDRESS 7

//NOT// SET VELOCITY:  D,speed_motor_left,speed_motor_right\n
//NOT// SET PIDs:      H,P,I,D,1/2\n
// READ ENCODER:  R\n
// SET PWM:       L,speed_motor_left,speed_motor_right\n
// RESET ENCODER: I\n
//NOT// READ PID Value:S,1/2\n
//NOT// SET DebugRate: Z,rate\n   -- bigger/slower

#include <math.h>
#include <EEPROM.h>
#include <Encoder.h>
Encoder encL(2,4);
Encoder encR(3,5);

#include "MusafirMotor.h"
MusafirMotor motorL(7, 6, 9);
MusafirMotor motorR(13, 12, 10);

#include <PID_v1.h>
struct motorParams {
  double kp;
  double ki;
  double kd;
};
motorParams motorPIDL;
motorParams motorPIDR;
double vel1 = 0, vel2 = 0;
double spd1, spd2;
double pwm1, pwm2;
// PID (&input, &output, &setpoint, kp, ki, kd, DIRECT/REVERSE)
PID pidL(&spd1, &pwm1, &vel1, 1,0,0, DIRECT);
PID pidR(&spd2, &pwm2, &vel2, 1,0,0, DIRECT);
boolean pidActive= false;


unsigned long previousMillis = 0;
const long interval = 10; // in ms

String inputString = "";
boolean stringComplete = false;


#include "Navigator.h"
Navigator  navigator;
//from https://github.com/solderspot/NavBot/blob/master/NavBot_v1/BlankBot.h
// Navigator defines
#define WHEELBASE               nvMM(190)      // millimeters
#define WHEEL_DIAMETER          nvMM(91.5)     // millimeters
#define TICKS_PER_REV           1500

// correct for systematic errors
#define WHEEL_RL_SCALER         1.0f  // Ed
#define WHEELBASE_SCALER        1.0f  // Eb
// correct distance 
#define DISTANCE_SCALER         1.0f  // Es


void setup() {
  Serial.begin(115200);
  inputString.reserve(100);
  
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);
  
  navigator.InitEncoder( WHEEL_DIAMETER, WHEELBASE, TICKS_PER_REV );
  navigator.SetDistanceScaler( DISTANCE_SCALER );
  navigator.SetWheelbaseScaler( WHEELBASE_SCALER );
  navigator.SetWheelRLScaler( WHEEL_RL_SCALER );
  navigator.Reset(millis());

  initEEPROM();

  pidL.SetMode(MANUAL); // PID CONTROL OFF
  pidR.SetMode(MANUAL);
  pidL.SetTunings(motorPIDL.kp, motorPIDL.ki, motorPIDL.kd);
  pidR.SetTunings(motorPIDR.kp, motorPIDR.ki, motorPIDR.kd);
  pidL.SetSampleTime(interval); // sample time for PID
  pidR.SetSampleTime(interval);
  pidL.SetOutputLimits(0,255);  // min/max PWM
  pidR.SetOutputLimits(0,255);
}

void(* resetFunc) (void) = 0;

unsigned int debugCount=0;
bool debugPrinted=false;
long encCurr1, encCurr2;

void loop() {
  if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    encCurr1 = encL.read();
    encCurr2 = -encR.read(); //encoder is reversed
    navigator.UpdateTicks(encCurr1, encCurr2, currentMillis);
    encL.write(0); encR.write(0);

    float distance1 = (float)encCurr1*TICKS_PER_REV;
    spd1 = (float)distance1*(1000.0/interval);
    float distance2 = (float)encCurr2*TICKS_PER_REV;
    spd2 = (float)distance2*(1000.0/interval);
    debugCount++;
  }
  
  pidL.Compute();
  pidR.Compute();
  if(pidActive){
    if(vel1>0) motorL.setPWM(pwm1);
    else motorL.setPWM(0);
    if(vel2>0) motorR.setPWM(pwm2);
    else motorR.setPWM(0);
  }
    
  if(debugCount%100==0){
    Serial.print("x=");
    Serial.print(navigator.Position().x);
    Serial.print(",y=");
    Serial.print(navigator.Position().y);
    Serial.print(",T=");
    Serial.print(navigator.Heading());
    Serial.print(",Speed=");
    Serial.print(navigator.Speed());
    Serial.print(",TurnRate=");
    Serial.println(navigator.TurnRate());

    Serial.print("Spd1=");
    Serial.print(spd1);
    Serial.print("Spd2=");
    Serial.println(spd2);
  }

}

void interpretSerialData(void){
    int c1=1, c2=1;
    int val1=0, val2=0;
    switch(inputString[0]){
      case 'D':
        // COMMAND:  D,speed_motor_left,speed_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        val2 = inputString.substring(c1).toInt();
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        vel1 = val1;
        vel2 = val2;
        if(DEBUG){
          Serial.print("Velocity 1 ");
          Serial.println(vel1);
          Serial.print("Velocity 2 ");
          Serial.println(vel2);
        }         
        Serial.println('d');
        if(vel1>0)
          pidL.SetMode(AUTOMATIC);
        else
          pidL.SetMode(MANUAL);
        if(vel2>0)
          pidR.SetMode(AUTOMATIC);
        else
          pidR.SetMode(MANUAL);
        pidActive= true;
        break;
      case 'H':
        // COMMAND:  H,P,I,D,1/2\n
        float p,i,d;
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        p = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        i = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        d = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        val1 = inputString.substring(c1).toInt();
        if(val1==1) {
          motorPIDL.kp = p;
          motorPIDL.ki = i;
          motorPIDL.kd = d;
          pidL.SetTunings(motorPIDL.kp, motorPIDL.ki, motorPIDL.kd);
          EEPROM.put((const int)MAGICADDRESS, motorPIDL);
          if(DEBUG) Serial.println("motorPIDL ");
        }
        else if(val1==2){
          motorPIDR.kp = p;
          motorPIDR.ki = i;
          motorPIDR.kd = d;
          pidR.SetTunings(motorPIDR.kp, motorPIDR.ki, motorPIDR.kd);
          EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
          if(DEBUG) Serial.println("motorPIDR ");
        }
        Serial.print("h,");
        Serial.print(val1);
        Serial.print(',');
        Serial.println(p);
        Serial.print(',');
        Serial.println(i);
        Serial.print(',');
        Serial.println(d);
        break;
      case 'L':
        // COMMAND:  L,speed_motor_left,speed_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = inputString.indexOf(',',c2)+1;
        val2 = inputString.substring(c1).toInt();
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        pidActive= false;
        pidL.SetMode(MANUAL);
        pidR.SetMode(MANUAL);
        pwm1 = val1;
        pwm2 = val2;
        motorL.setPWM(pwm1);
        motorR.setPWM(pwm2);
        if(DEBUG){
          Serial.print("PWM1: "); Serial.println(val1);        
          Serial.print("PWM2: "); Serial.println(val2);
        }
        Serial.println('l');
        break;
      case 'R':
        // COMMAND:  R\n
        Serial.print("r,");
        Serial.print(encL.read());
        Serial.print(',');
        Serial.print(encR.read());
        Serial.println();
        break;
      case 'I':
        // COMMAND: I\n
        // ZP=add navigator.reset here
        navigator.Reset(millis());
        Serial.println('i');
        break;
      case 'S':
        // COMMAND:  S,1/2\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1).toInt();
        if(val1==1) {
          Serial.print("s,");
          Serial.print(motorPIDL.kp);
          Serial.print(',');
          Serial.print(motorPIDL.ki);
          Serial.print(',');
          Serial.println(motorPIDL.kd);
        }else if(val1==2) {
          Serial.print("s,");
          Serial.print(motorPIDR.kp);
          Serial.print(',');
          Serial.print(motorPIDR.ki);
          Serial.print(',');
          Serial.println(motorPIDR.kd);
        }
        break;
      default:
        Serial.print("UNKNOWN COMMAND: ");
        Serial.println(inputString);
        break;
    }
}

void serialEvent() 
{
  while(Serial.available()){
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void initEEPROM(void){
  int checkEEPROM=0;
  EEPROM.get(0, checkEEPROM);
  if(checkEEPROM==MAGICADDRESS){
    if(DEBUG) Serial.println("Reading from EEPROM.");
    EEPROM.get((const int)MAGICADDRESS, motorPIDL);   //(address, data)
    EEPROM.get((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }
  else{
    // Set default values
    if(DEBUG) Serial.println("Setting Default Values.");
    EEPROM.put(0, MAGICADDRESS);
    motorPIDL.kp = 1.0;
    motorPIDL.ki = 0.0;
    motorPIDL.kd = 0.0;
    EEPROM.put((const int)MAGICADDRESS, motorPIDL);

    motorPIDR.kp = 1.0;
    motorPIDR.ki = 0.0;
    motorPIDR.kd = 0.0;
    EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }  
}
