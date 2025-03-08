#include "hexapod.hpp"
#include <stdbool.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include "position.hpp"

#ifndef HEXA_JSON_PARSE
#define HEXA_JSON_PARSE

    class JsonParser {
        public:
            JsonParser(Hexapod &hexapod, double &x, double &y, double &z, double &roll, double &pitch, double &yaw, double &speed);
            void parseCommand(String command_str);
            String getCommandType(const String &command);
            void performPreset(String preset);
            void performMovement(String movement);
            void updateVariables(const String &command);
            String movement_sel = "None";
            JsonDocument json_joints;

        private:
            Hexapod &_Hexapod;
            double _x;
            double _y;
            double _z;
            double _roll;
            double _pitch;
            double _yaw;
            double _speed;
            Position _position;
            uint32_t _movement_time = 0;
            double _leg_positions[NUM_LEGS][NUM_AXES_PER_LEG] = {{0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}};
    };

#endif