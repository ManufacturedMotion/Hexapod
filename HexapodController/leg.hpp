#include <axis.hpp>
#include <stdbool.h>
#include <stdint.h>

#ifndef LEG_H
#define LEG_H

#define NUM_AXES_PER_LEG 3
class Leg {
    public:
        Leg(uint8_t leg_number);
        double current_angles[NUM_AXES_PER_LEG];
        double current_cartesian[NUM_AXES_PER_LEG];
        _Bool rapid_move(double x,  double y, double z);
        _Bool linear_move(double x,  double y, double z, double speed);
    private:
        uint8_t _leg_number;
        double _length0;
        double _length1;
        double _length2;
        void move_axes();
        _Bool check_safe_coords(double x, double y, double z);
        _Bool inverse_kinematics(double x, double y, double z);
        double _next_angles[NUM_AXES_PER_LEG];
        double _next_cartesian[NUM_AXES_PER_LEG];
        Axis axes[3];
};

#endif