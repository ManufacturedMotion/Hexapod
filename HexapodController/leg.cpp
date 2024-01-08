#include "leg.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

double * Leg::inverse_kinematics(double x, double y, double z) {
    static double results[3];
    if (fabs(y < 0.0005)) {
        if (x > 0.0005) {
            results[0] = PI / 2.0;
        }
        else if (x < -0.0005) {
            results[0] = -PI / 2.0;
        }
        else {
            results[0] = 0;
        }
    }  
    else {
        results[0] = atan2(x,y);
    }

    double yv = sqrt(y * y + x * x);
    double dt = sqrt(yv * yv + z * z);
    
    double theta1_tool0 = atan2(yv,z);;
    double theta1_tool1 = acos((dt * dt + _length1 * _length1 - _length2 * _length2) / (2 * dt * _length1));
    
    double theta2_tool = acos((_length1 * _length1 + _length2 * _length2 - dt * dt) / (2 * _length1 * _length2));

    results[1] = theta1_tool0 - theta1_tool1;
    results[2] = PI - theta2_tool;

    // results[1] = theta1_tool0 + theta1_tool1;
    // results[2] = -(PI - theta2_tool);

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (results[i] != results[i]); //Check for NaN
        return false;
    }
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        _next_angles[i] = results[i];
    }
    return results;
}

_Bool Leg::rapid_move(double x,  double y, double z) {
    double * positions = inverse_kinematics(x, y, z) 
    move_axes(positions);
    return true;
}

_Bool Leg::linear_move_setup(double x,  double y, double z, double speed) {
    _move_time = (fabs(get_current_pos() - pos) / speed) * 1000;
    _start_x = get_current_x();
    _start_y = get_current_y();
    _start_z = get_current_z();
    _end_x = x;
    _end_y = y;
    _end_z = z;
    _move_progress = 0;
    _move_start_time = _millis;
}


uint8_t Leg::run_linear_move() {
    _move_progress = (float)(_millis - _move_start_time) / ((float) _move_time);
    if (_move_progress <= 1.0) {
        double next_x = _move_progress * (_end_x - _start_x) + _start_x;
        double next_y = _move_progress * (_end_x - _start_x) + _start_y;
        double next_z = _move_progress * (_end_x - _start_x) + _start_z;
        double * positions = inverse_kinematics(next_x, next_y, next_z); 
        move_axes(inverse_kinematics(next_x, next_y, next_z));
        return 0;
    }
    return 255;
}

void Leg::move_axes(double * positions) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axes[i].move_to_pos(_next_angles[i]);
    }
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        current_angles[i] = _next_angles[i];
    }
}

double Leg::get_current_x() {
    return current_cartesian[0];
}

double Leg::get_current_y() {
    return current_cartesian[1];
}

double Leg::get_current_z() {
    return current_cartesian[2];
}