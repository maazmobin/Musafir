// ---------------------------------------------------------------------------
// Created by Zaid Pirwani - zaid@ejaadtech.com
// Copyright 2015 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html
//
// See "Localizer.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Localizer constructor
// ---------------------------------------------------------------------------

Localizer::Localizer(	long* encCount1, long* encCount2,
						float wheelDia, float wheelBase,
						int countsPerRevolution){
					_encCount1;
					_encCount2;
					wheelDia;
					wheelBase;
					countsPerRevolution;
}


// ---------------------------------------------------------------------------
// Standard  methods
// ---------------------------------------------------------------------------

void Localizer::SOMETHIN(uint8_t pwm) {

}



class Localizer {
	public:
		Pose getPose();
		void setPose(Pose pose);
		void setPose(int x, int y, int heading);
		void setPosition(int x, int y);
		void setHeading(int heading);
		void compute(void);
	private:
		
};

#endif