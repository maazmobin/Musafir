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
		Localizer(long* encCount1, long* encCount2, float wheelDia, float wheelBase, int countsPerRevolution);
		Pose getPose();
		void setPose(Pose pose);
		void setPose(int x, int y, int heading);
		void setPosition(int x, int y);
		void setHeading(int heading);
		void compute(void);
	private:
		Pose  _pose(0,0,0);	//x=0, y=0, theta=0;
		long* _encCount1;
		long* _encCount2;
		float _wheelDia;
		float _wheelBase;
		int _countsPerRevolution;
};

#endif