#include "axis.hpp"
#include "config.hpp"
#include "three_by_matrices.hpp"
#include <stdbool.h>
#include <stdint.h>

#ifndef HEXA_LEG
#define HEXA_LEG

	#define NUM_AXES_PER_LEG 3
	#define MOVEMENT_INTERVAL_US 5000
	class Leg {
		public:
			Leg();
			void initializeAxes(uint8_t leg_number);
			double current_angles[NUM_AXES_PER_LEG];
			_Bool rapidMove(double x,  double y, double z);
			_Bool linearMove(double x,  double y, double z, double speed);
			Axis axes[NUM_AXES_PER_LEG];
			ThreeByOne forwardKinematics(double axis0_angle, double axis1_angle, double axis2_angle);
			uint8_t runLegSpeed(void* (*)(double));
			_Bool linearMoveSetup(double x,  double y, double z, double target_speed, _Bool relative = false);
			uint8_t linearMovePerform();
			_Bool isMoving();
			void wait(uint32_t time_ms);

		private:
			uint8_t _leg_number;
			double _length0 = 63.00;
			double _length1 = 92.00;
			double _length2 = 157.5;
			void moveAxes();
			_Bool checkSafeCoords(double x, double y, double z);
			_Bool inverseKinematics(double x, double y, double z);
			double _next_angles[NUM_AXES_PER_LEG];
			double _current_cartesian[NUM_AXES_PER_LEG];
			double _next_cartesian[NUM_AXES_PER_LEG];
			double _start_cartesian[NUM_AXES_PER_LEG];
			double _end_cartesian[NUM_AXES_PER_LEG];
			uint32_t _move_start_time; 
			double _max_speed = 1000000.0;
			uint32_t _move_time;
			_Bool _moving_flag = false;
			void * _movement_function;
			double linearMovement(double move_progress);
	};

#endif
