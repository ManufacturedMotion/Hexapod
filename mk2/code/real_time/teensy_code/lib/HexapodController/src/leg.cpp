#include "leg.hpp"
#include "hexapod.hpp"
#include "axis.hpp"
#include "config.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "three_by_matrices.hpp"
#include <Arduino.h>

double zero_points[NUM_LEGS][NUM_AXES_PER_LEG] = ZERO_POINTS;
int pwm_pins[NUM_LEGS][NUM_AXES_PER_LEG] = PWM_PINS;
double min_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MIN_POS;
double max_pos[NUM_LEGS][NUM_AXES_PER_LEG] = MAX_POS;
double scale_fact[NUM_LEGS][NUM_AXES_PER_LEG] = SCALE_FACT;
_Bool reverse_axis[NUM_LEGS][NUM_AXES_PER_LEG] = REVERSE_AXIS;

enum Dimension { X = 0, Y = 1, Z = 2};

Leg::Leg() {
    _leg_number = 0;
}

void Leg::initializeAxes(uint8_t leg_number) {
    _leg_number = leg_number;
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        axes[i].initializePositionLimits(pwm_pins[_leg_number][i], min_pos[_leg_number][i], max_pos[_leg_number][i]);
        axes[i].setMapping(zero_points[_leg_number][i], scale_fact[_leg_number][i], reverse_axis[_leg_number][i]);
    }
}

_Bool Leg::_inverseKinematics(double x, double y, double z) {

    if (!_checkSafeCoords(x,y,z))
        return false; 
    double potential_results[3];
    if (fabs(y < 0.1)) {
        if (x > 0.1) {
            potential_results[0] = M_PI / 2.0;
        }
        else if (x < -0.1) {
            potential_results[0] = -M_PI / 2.0;
        }
        else {
            potential_results[0] = 0;
        }
    }  
    else {
        potential_results[0] = atan2(x,y);
    }

    double y_virtual_planar = sqrt(y * y + x * x) - _length0;
    double planar_distance = sqrt(y_virtual_planar * y_virtual_planar + z * z);
    
    double theta2_tool  =   (planar_distance*planar_distance - _length1*_length1 - _length2*_length2)
                        /   (2 * _length1 * _length2);

    potential_results[2] = atan2(sqrt(1 - (theta2_tool*theta2_tool)), theta2_tool);
    
    double theta1_tool0 = atan2(z, y_virtual_planar);
    double theta1_tool1 = atan2(_length2 * sin(potential_results[2]), _length1 + _length2 * cos(potential_results[2]));
    potential_results[1] = theta1_tool0 - theta1_tool1;

    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        if (potential_results[i] != potential_results[i]) { //Check for NaN
            // Serial.println(i);
            // Serial.println(potential_results[i]);
            return false;
        }
    }

    // ThreeByOne resulting_pos = forwardKinematics(potential_results[0], potential_results[1], potential_results[2]);
    // Serial.printf("Result\n  x: %f; y: %f; z: %f\n", resulting_pos.values[0], resulting_pos.values[1], resulting_pos.values[2]);

    // Serial.printf("Result\n  angle0: %f; angle1: %f; angle2: %f\n", potential_results[0], potential_results[1], potential_results[2]);
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        _next_angles[i] = potential_results[i];
    }
    return true;
}

_Bool Leg::_checkSafeCoords(double x, double y, double z) {
    //If coords within robot body
    //  return false;
    
    return true;
}

_Bool Leg::rapidMove(double x,  double y, double z) {
    if (_inverseKinematics(x, y, z)) {
        _moveAxes();
        _current_cartesian[0] = x;
        _current_cartesian[1] = y;
        _current_cartesian[2] = z;
        return true;
    }
    return false;
}

uint8_t Leg::linearMovePerform() {
    double move_progress = (float)(millis() - _move_start_time) / ((float) _move_time);
    if (move_progress <= 1.0) {
        // Serial.printf("Move start time: %d, _move_time %d\n", millis() - _move_start_time, _move_time);
        double next_x = move_progress * (_end_cartesian[0] - _start_cartesian[0]) + _start_cartesian[0];
        double next_y = move_progress * (_end_cartesian[1] - _start_cartesian[1]) + _start_cartesian[1];
        double next_z = move_progress * (_end_cartesian[2] - _start_cartesian[2]) + _start_cartesian[2];
        _moving_flag = true;
        return rapidMove(next_x, next_y, next_z);
    }
    else {
        _moving_flag = false;
    }
    return 0;
}

void Leg::wait(uint32_t time_ms) {
   
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        _start_cartesian[i] = _current_cartesian[i];
        _end_cartesian[i] = _current_cartesian[i];
    }
    _move_start_time = millis();
    _moving_flag = true;
    _move_time = time_ms;
    // Serial.printf("Wait for %d ms\n", _move_time);
}


// double Leg::linearMovement(double move_progress, Dimension axis) {
//     return move_progress * (_end_cartesian[axis] - _start_cartesian[axis]) + _start_cartesian[axis];
// }

// double Leg::radialMovement(double move_progress, Dimension axis) {
//     return move_progress * 
// }

// void Leg::ISRLinearMove() {
//     // Attach this function to an IntervalTimer on a 10-50ms interval
//     uint8_t move_status = runLegSpeed(&linearMovement);
//     if (!move_status)
//         _moving_flag = false;
// }

_Bool Leg::isMoving() {
    return _moving_flag;
}

_Bool Leg::linearMoveSetup(double x,  double y, double z, double target_speed, _Bool relative) {
    uint8_t retval = 0;
    double speed = target_speed;
    if (target_speed > _max_speed) {
        speed = _max_speed;
        retval = 1; // move speed capped
    }
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        _start_cartesian[i] = _current_cartesian[i];
    }
    _end_cartesian[0] = x;
    _end_cartesian[1] = y;
    _end_cartesian[2] = z;
    // if (relative) {
    //     _end_cartesian[0] += _current_cartesian[0];
    //     _end_cartesian[1] += _current_cartesian[1];
    //     _end_cartesian[2] += _current_cartesian[2];
    // }
    _move_start_time = millis();
    _moving_flag = true;
    double x_dist = _start_cartesian[0] - _end_cartesian[0];
    double y_dist = _start_cartesian[1] - _end_cartesian[1];
    double z_dist = _start_cartesian[2] - _end_cartesian[2];
    _move_time = (sqrt(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist) / speed) * 1000; 
    // Serial.printf("_end_cartesian: x:%f, y:%f, z:%f\n", _end_cartesian[0], _end_cartesian[1], _end_cartesian[2]);
    return retval;
}

void Leg::_moveAxes() {
    for (uint8_t i = 0; i < NUM_AXES_PER_LEG; i++) {
        axes[i].moveToPos(_next_angles[i]);
        current_angles[i] = _next_angles[i];
    }
}

ThreeByOne Leg::forwardKinematics(double axis0_angle, double axis1_angle, double axis2_angle) {
    ThreeByOne length0(0.0, _length0, 0.0);
    ThreeByOne length1(0.0, _length1, 0.0);
    ThreeByOne length2(0.0, _length2, 0.0);

    length0.rotateYaw(axis0_angle);

    length1.rotateYaw(axis0_angle);
    length1.rotateRoll(axis1_angle);

    length2.rotateYaw(axis0_angle);
    length2.rotateRoll(axis1_angle);
    length2.rotateRoll(axis2_angle);

    // Serial.printf("length0\n  x: %f; y: %f; z: %f\n",length0.values[0], length0.values[1], length0.values[2]);
    // Serial.printf("length1\n  x: %f; y: %f; z: %f\n",length1.values[0], length1.values[1], length1.values[2]);
    // Serial.printf("length2\n  x: %f; y: %f; z: %f\n",length2.values[0], length2.values[1], length2.values[2]);
    return length0 + length1 + length2;
}

