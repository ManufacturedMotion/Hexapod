



#include <stdbool.h>
#include <Arduino.h>
#include "three_by_matrices.hpp"

#ifndef HEXA_POSITION
#define HEXA_POSITION

	#define ROTATION_MAGNITUDE_SCALE 100.0 // Scale applied to rotations for determining scaledMagnitude() of move

	class Position {
		public:
			double x;
			double y;
			double z;
			double roll;
			double pitch;
			double yaw;
			void setPos(const Position& pos);
			void set(double new_x, double new_y, double new_z, double new_roll, double new_pitch, double new_yaw);
			void scalarMult(double factor);
			void independentScalarMult(double factors[6]);
			_Bool equals(const Position& pos);
			Position operator*(const double& multiplier);
			void operator*=(const double& multiplier);
			Position operator+(const Position& pos);
			void  operator+=(const Position& pos);
			Position operator-(const Position& pos);
			void operator-=(const Position& pos);
			_Bool operator<(const Position& pos);
			_Bool operator>(const Position& pos);
			_Bool operator<=(const Position& pos);
			_Bool operator>=(const Position& pos);
			_Bool operator==(const Position& pos);
			_Bool operator!=(const Position& pos);
			void clear();
			ThreeByOne coord();
			Position unitVector();
			Position operator=(const Position& pos);
			double magnitude();
			double scaledMagnitude();
			void usbSerialize();
		private:
	};
  	Position getPosFromCommand(String command);

#endif // HEXA_POSITION
