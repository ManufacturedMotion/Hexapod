#include <stdbool.h>
#include <stdint.h>
#include <PWMServo.h>
#include <elapsedMillis.h>

#ifndef AXIS_H
#define AXIS_H

/*
TO DO:
1. Figure out method of determining if the servo has reached the target position
    Plan A: Just wait a certain number of microseconds based on the _max_speed attribute
            - Unsure how well this will work due to the servos going at different speeds with different loading
                - Worried the movements may look choppy with low loads as the motor will stop and start
                - I think we can get around that by making the time steps really small
    Plan B: https://www.instructables.com/Servo-Feedback-Hack-free/
        - This is a lot of work to do 18 times
        - We will run out of pins on the Teensy so will need to put in multiplexers for IO expansion
        - New PCB required for sure
*/

class Axis {
    public:
	Axis(); //default dummy constructor
        Axis(uint8_t pwm_pin, double min_pos, double max_pos);
        uint8_t move_to_pos(double pos);
        uint8_t move_to_pos_at_speed(double pos, double target_speed);
        _Bool set_max_pos(double max_pos);
        _Bool set_min_pos(double min_pos);
        double get_current_pos();
        _Bool set_mapping(double offset, double map_mult);
        _Bool set_max_speed(double max_speed);
        double get_max_speed();
        uint8_t run_speed();

    private:
        double _max_speed;      //rad/s
        double _max_pos;        //rad
        double _min_pos;        //rad
        double _map_mult;       //unitless
        double _zero_pos;       //rad
        uint64_t _next_go_time; //milliseconds
        double _current_pos = 0.0; //rad
        uint16_t _move_time = 0; //milliseconds
        double _start_rads;
        double _end_rads;
        double _move_progress;
        uint32_t _move_start_time;


        PWMServo servo;
        elapsedMillis _millis;
        
        uint8_t _motor_map(double x);
        double _rads_to_degrees(double rads);
        double _degrees_to_rads(double degrees);
};

#endif

