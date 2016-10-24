#ifndef Pose_h
#define Pose_h

#if defined (ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
	#include <pins_arduino.h>
#endif

#if defined (__AVR__)
	#include <avr/io.h>
#endif

class Pose
{
  public:
    Pose(int x, int y, int heading){
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
    int x;
	int y;
	int heading;
};

#endif