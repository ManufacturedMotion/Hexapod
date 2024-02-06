#include "leg.hpp"
#include "axis.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

Leg::Leg(uint8_t leg_number) {
    _leg_number = leg_number;
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        axes[i] = Axis(pwm_pins[_leg_number-1][i], min_pos[_leg_number-1][i], max_pos[_leg_number-1][i]);
        axes[i].set_mapping(offsets[_leg_number-1][i], map_mults[_leg_number-1][i]);
    }
}

_Bool Leg::inverse_kinematics(double x, double y, double z) {

    if (!check_safe_coords(x,y,z))
        return false; 
    double potential_results[3];
    if (fabs(y < 0.0005)) {
        if (x > 0.0005) {
            potential_results[0] = PI / 2.0;
        }
        else if (x < -0.0005) {
            potential_results[0] = -PI / 2.0;
        }
        else {
            potential_results[0] = 0;
        }
    }  
    else {
        potential_results[0] = atan2(x,y);
    }

    double yv = sqrt(y * y + x * x);
    double dt = sqrt(yv * yv + z * z);
    
    double theta1_tool0 = atan2(yv,z);;
    double theta1_tool1 = acos((dt * dt + _length1 * _length1 - _length2 * _length2) / (2 * dt * _length1));
    
    double theta2_tool = acos((_length1 * _length1 + _length2 * _length2 - dt * dt) / (2 * _length1 * _length2));

    potential_results[1] = theta1_tool0 - theta1_tool1;
    potential_results[2] = PI - theta2_tool;

    // potential_results[1] = theta1_tool0 + theta1_tool1;
    // potential_results[2] = -(PI - theta2_tool);

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (potential_results[i] != potential_results[i])  //Check for NaN
            return false;
    }
    
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        _next_angles[i] = potential_results[i];
    }
    return true;
}

_Bool Leg::check_safe_coords(double x, double y, double z) {
    //If coords within robot body
    //  return false;
    
    return true;
}

_Bool Leg::rapid_move(double x,  double y, double z) {
    if (inverse_kinematics(x, y, z)) {
        move_axes();
        return true;
    }
    return false;
}

_Bool Leg::linear_move(double x,  double y, double z, double speed) {
    return false;
}

void Leg::move_axes() {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axes[i].move_to_pos(_next_angles[i]);
    }
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        current_angles[i] = _next_angles[i];
    }
}