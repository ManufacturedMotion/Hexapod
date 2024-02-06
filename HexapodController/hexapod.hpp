#include <axis.hpp>
#include <leg.hpp>
#include <position.hpp>
#include <stdbool.h>
#include <stdint.h>

#ifndef HEXAPOD_H
#define HEXAPOD_H

#define NUM_LEGS 6
class Hexapod {
    public:
        Hexapod();
        _Bool linear_move(Position next_pos);
        
    private:
        uint8_t _leg_group1[NUM_LEGS / 2] = {1,4,6}; // Divide legs into two self-stable groups
        uint8_t _leg_group2[NUM_LEGS / 2] = {2,3,5};
};

#endif