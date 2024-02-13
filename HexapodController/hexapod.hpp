#include "axis.hpp"
#include "leg.hpp"
#include "position.hpp"
#include <stdbool.h>
#include <stdint.h>

#ifndef HEXA_H
#define HEXA_H

	#define NUM_LEGS 6
	class Hexapod {
		public:
			Hexapod();
			_Bool linearMove(Position next_pos);
			Leg legs[NUM_LEGS];
		private:
			uint8_t _leg_group_1[NUM_LEGS / 2] = {1,4,6}; // Divide legs into two self-stable groups
			uint8_t _leg_group_2[NUM_LEGS / 2] = {2,3,5};
	};

#endif
