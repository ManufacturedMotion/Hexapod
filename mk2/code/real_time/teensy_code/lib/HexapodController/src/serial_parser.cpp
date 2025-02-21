#include "serial_parser.hpp"
#include "json_parser.hpp"
#include "user_config.hpp"
#include "hexapod.hpp"
#include <stdbool.h>
#include "position.hpp"

#if DEBUG
    #include "gcode_parser.hpp"
#endif

SerialParser::SerialParser(Hexapod &hexapod) 
    : _Hexapod(hexapod),  // Initialize reference _Hexapod with the passed object
      _JsonParser(hexapod, x, y, z, roll, pitch, yaw, speed),  // Initialize JsonParser
      #if DEBUG
      _GcodeParser(hexapod, x, y, z, roll, pitch, yaw, speed)  // Initialize GcodeParser
      #endif
{
    
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

    _Bool retval = false;
    if (command.startsWith("{")) {
        retval = _JsonParser.optimizableCommand(command);
    }
    else {
        #if DEBUG
            retval = _GcodeParser.optimizableCommand(command);
        #endif
    }

    return retval;

}

void SerialParser::test(Hexapod hexapod) {
    //_GcodeParser.performPreset(1);
    //hexapod.sit();
    //_Hexapod.moveToZeros();
    //_Hexapod.sit();
}