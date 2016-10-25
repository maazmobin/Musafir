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
#include <PID_v1.h>

Encoder encL(2,4);
Encoder encR(3,5);

#include "MusafirMotor.h"
MusafirMotor motorL(7, 6, 9);
MusafirMotor motorR(13, 12, 10);

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
}

void(* resetFunc) (void) = 0;

unsigned int debugCount=0;
void loop() {
  if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
  }

  
  if(navigator.UpdateTicks(encL.read(), encR.read(), millis()))
    debugCount++;

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
    Serial.print(navigator.TurnRate());
  }

}

void interpretSerialData(void){
    int c1=1, c2=1;
    int val1=0, val2=0;
    switch(inputString[0]){
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
        motorL.setPWM(val1);
        motorR.setPWM(val2);
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
        Serial.println('i');
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
