#include "hexapod.hpp"
#include "position.hpp"
#include <stdint.h>
#include <stdbool.h>

#ifndef HEXA_GCODE_PARSE
#define HEXA_GCODE_PARSE

    class GcodeParser {
        public:
            GcodeParser(Hexapod &hexapod, double &x, double &y, double &z, double &roll, double &pitch, double &yaw, double &speed);
            void parseCommand(String command);
            void performPreset(uint8_t preset);
            void performMovement(uint8_t movement);
            void updateVariables(const String &command);
            _Bool optimizableCommand(const String &command);
            std::tuple<uint8_t, uint8_t, double> parseGcodeJoint(std::string joint);
            _Bool checkPositionString(std::string pos_str);
            _Bool isNumeric(const std::string &str);
            uint8_t movement_sel = 255;
            _Bool valid_MTPS = false;

        private:
            Hexapod &_Hexapod;
            double &_x;
            double &_y;
            double &_z;
            double &_roll;
            double &_pitch;
            double &_yaw;
            double &_speed;
            Position _position;
            uint32_t _movement_time = 0;
            double _leg_positions[NUM_LEGS][NUM_AXES_PER_LEG] = {{0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}};
            uint8_t _tune_leg = 25;
            uint8_t _tune_axis = 25;
            double _tune_pos;
            String _msg = "";
    };

#endif