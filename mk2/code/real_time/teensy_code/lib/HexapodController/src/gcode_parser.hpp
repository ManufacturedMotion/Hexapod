#include "hexapod.hpp"
#include "position.hpp"
#include <stdint.h>
#include <stdbool.h>

#ifndef HEXA_GCODE_PARSE
#define HEXA_GCODE_PARSE

class GcodeParser {
    public:
        GcodeParser();
        GcodeParser(Hexapod &hexapod, double &x, double &y, double &z, double &roll, double &pitch, double &yaw, double &speed, _Bool &wait);
        void parseCommand(String command);
        void performPreset(uint8_t preset);
        void performMovement(uint8_t movement);
        void updateVariables(const String &command);
        _Bool optimizableCommand(const String &command);
        uint8_t movement_sel = 255;

    private:
        Hexapod _Hexapod;
        double _x;
        double _y;
        double _z;
        double _roll;
        double _pitch;
        double _yaw;
        double _speed;
        _Bool _wait;
        Position _position;
        uint16_t _movement_time = 0;
        double _leg_positions[NUM_LEGS][NUM_AXES_PER_LEG] = {{0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}};
};

#endif