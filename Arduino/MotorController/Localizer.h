#ifndef Localizer_h
#define Localizer_h

#if defined (ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
	#include <pins_arduino.h>
#endif

#if defined (__AVR__)
	#include <avr/io.h>
#endif

class Localizer {
	public:
		Localizer(long* encCount1, long* encCount2, const float wheelDia, const float wheelBase, const int countsPerRevolution);
		Pose getPose();
		void setPose(Pose pose);
		void setPose(int x, int y, int heading);
		void setPosition(int x, int y);
		void setHeading(int heading);
		void compute(void);
	private:
		
};

#endif