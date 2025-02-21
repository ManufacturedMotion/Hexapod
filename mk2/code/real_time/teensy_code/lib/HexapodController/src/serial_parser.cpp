#include "serial_parser.hpp"
#include "json_parser.hpp"
#include "user_config.hpp"
#include "hexapod.hpp"
#include <stdbool.h>
#include "position.hpp"

#if DEBUG
    #include "gcode_parser.hpp"
#endif

SerialParser::SerialParser(Hexapod &hexapod) {
    _Hexapod = hexapod;
    _JsonParser = JsonParser(hexapod, x, y, z, roll, pitch, yaw, speed);
    #if DEBUG
        _GcodeParser = GcodeParser(hexapod, x, y, z, roll, pitch, yaw, speed);
    #endif
}

void SerialParser::parseCommand(String command) {

    #if !DEBUG
        if (!command.startsWith("{")) {
            Serial.printf("ERROR! Serial command not in json format. This is not supported unles DEBUG set to true in user_config.hpp");
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

_Bool SerialParser::optimizableCommand(const String &command) {

    _Bool ret_val = false;
    if (command.startsWith("{")) {
        ret_val = _JsonParser.optimizableCommand(command);
    }
    else {
        #if DEBUG
            ret_val = _GcodeParser.optimizableCommand(command);
        #endif
    }

    return ret_val;

}
