#include <axis.hpp>
#include <stdbool.h>
#include <stdint.h>

#ifndef LEG_H
#define LEG_H


#define NUM_AXES 3
class Leg {
    public:
        double current_angles[NUM_AXES];
        double current_cartesian[NUM_AXES];
        double get_current_x();
        double get_current_y();
        double get_current_z();
        _Bool rapid_move(double x,  double y, double z);
        _Bool linear_move(double x,  double y, double z, double speed);
    private:

        //Inverse kinematics setup variables
        double _length0;
        double _length1;
        double _length2;

        //Linear movement variables
        uint32_t _move_time;
        double _start_x;
        double _start_y;
        double _start_z;
        double _end_x;
        double _end_y;
        double _end_z;
        double _move_progress;
        uint32_t _move_start_time;

        _Bool move_axes();
        _Bool inverse_kinematics(double x, double y, double z);
        double _next_angles[NUM_AXES];
        double _next_cartesian[NUM_AXES];
        Axis axes[3];
};

#endif