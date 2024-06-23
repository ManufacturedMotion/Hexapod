#include <stdbool.h>
#include <Arduino.h>
#include "three_by_matrices.hpp"

#ifndef HEXA_POSITION
#define HEXA_POSITION

	class Position {
		public:
			double X;
			double Y;
			double Z;
			double roll;
			double pitch;
			double yaw;
			void setPos(const Position& pos);
			void set(double new_X, double new_Y, double new_Z, double new_roll, double new_pitch, double new_yaw);
			void addPos(const Position& pos);
			void scalarMult(double factor);
			void independentScalarMult(double factors[6]);
			void subtractPos(const Position& pos);
			double distanceFromOrigin();
			_Bool equals(const Position& pos);
			Position operator*(const double& multiplier);
			void operator*=(const double& multiplier);
			Position operator+(const Position& pos);
			void  operator+=(const Position& pos);
			Position operator-(const Position& pos);
			void operator-=(const Position& pos);
			ThreeByOne coord();
			double magnitude();
		private:
	};
  Position getPosFromCommand(String command);

#endif
