#include "serial_parser.hpp"
#include "json_parser.hpp"
#include "user_config.hpp"
#include "hexapod.hpp"
#include <stdbool.h>
#include "position.hpp"

#include "json_parser.hpp"    // 👈 full definition required
#include "gcode_parser.hpp"
//class JsonParser;
//class GcodeParser;

#if DEBUG
    #include "gcode_parser.hpp"
#endif

SerialParser::SerialParser(Hexapod &hexapod) 
    : _Hexapod(hexapod),  
    _JsonParser(*hexapod, *x, *y, *z, *roll, *pitch, *yaw, *speed)  
    #if DEBUG
        , _GcodeParser(*hexapod, *x, *y, *z, *roll, *pitch, *yaw, *speed)  
    #endif
{
    return;
}

void SerialParser::parseCommand(String command) {

    #if !DEBUG
        if (!command.startsWith("{")) {
            SERIAL_OUTPUT.printf("ERROR! Serial command not in json format. This is not supported unles DEBUG set to true in user_config.hpp\n");
            return;
        }
    #endif

    if (command.startsWith("{")) {
        _JsonParser.parseCommand(command);
    }
    else {
        #if DEBUG
            _GcodeParser.parseCommand(command);
        #endif
    }
}
