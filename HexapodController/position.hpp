#include <stdbool.h>

#ifndef POSITION_H
#define POSITION_H

class Position {
    public:
        double X;
        double Y;
        double Z;
        double roll;
        double pitch;
        double yaw;
        void set_pos(const Position& pos);
        void set(double new_X, double new_Y, double new_Z, double new_roll, double new_pitch, double new_yaw);
        void add_pos(const Position& pos);
        void scalar_mult(double factor);
        void independent_scalar_mult(double factors[6]);
        void subtract_pos(const Position& pos);
        double distance_from_origin();
        _Bool equals(const Position& pos);
    private:
};

#endif