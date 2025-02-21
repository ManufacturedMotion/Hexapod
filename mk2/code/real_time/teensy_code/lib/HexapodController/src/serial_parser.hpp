#include "user_config.hpp"
#include "json_parser.hpp"
#include "hexapod.hpp"
#include <stdbool.h>
#include "position.hpp"

#if DEBUG
    #include "gcode_parser.hpp"
#endif

#ifndef HEXA_SERIAL_PARSE
#define HEXA_SERIAL_PARSE

    class SerialParser {
        public:
            SerialParser(Hexapod &hexapod);
            void parseCommand(String command);
            _Bool optimizableCommand(const String &command);
            double x = 0;
            double y = 0;
            double z = 200;
            double roll = 0;
            double pitch = 0;
            double yaw = 0;
            double speed = 100;
            void test(Hexapod hexapod);

        private:
            Hexapod &_Hexapod; 
            JsonParser _JsonParser;
            #if DEBUG
                GcodeParser _GcodeParser;
            #endif

    };


#endif
