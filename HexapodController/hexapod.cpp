#include "leg.hpp"
#include "axis.hpp"
#include "hexapod.hpp"
#include "position.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

Hexapod::Hexapod() { 
    for (uint8_t i = 1; i <= NUM_LEGS; i++) {
        legs[i - 1].initializeAxes(i);
    }
}

_Bool linearMove(Position next_pos) {
    return true;
} 
