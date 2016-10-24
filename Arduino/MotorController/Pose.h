/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

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
    Pose(float x, float y, float heading){
		this.x = x;
		this.y = y;
		this.heading = heading;
	}
    float x;
	float y;
	float heading;
};

#endif