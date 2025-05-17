#include "user_config.hpp"
#include "hexapod.hpp"
#include <stdbool.h>
#include "position.hpp"

#ifndef HEXA_SERIAL_PARSE
#define HEXA_SERIAL_PARSE

    class SerialParser {
        public:
            SerialParser(Hexapod &hexapod);
            double x = 0;
            double y = 0;
            double z = 200;
            double roll = 0;
            double pitch = 0;
            double yaw = 0;
            double speed = 100;
            void parseCommand(String command_str);
            String getCommandType(const String &command);
            void performPreset(String preset);
            void performMovement(String movement);
            void updateVariables(const String &command);
            std::tuple<uint8_t, uint8_t, double> parseJsonJoint(String joint, JsonPair json_pos);
            _Bool checkPositionString(std::string pos_str);
            _Bool isNumeric(const String &str);
            String movement_sel = "None";
            JsonDocument json_joints;
            _Bool valid_MTPS = false;

        private:
            Hexapod &_Hexapod;
            Position _position;
            uint32_t _movement_time = 0;
            double _leg_positions[NUM_LEGS][NUM_AXES_PER_LEG] = {{0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.00}};
            uint8_t _tune_leg = 25;
            uint8_t _tune_axis = 25;
            double _tune_pos;
            String _msg = "";
    };


#endif
