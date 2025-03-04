#include "gcode_parser.hpp"
#include "hexapod.hpp"
#include <stdint.h>
#include <string>
#include <sstream>
#include <stdbool.h>

GcodeParser::GcodeParser(Hexapod &hexapod, double &x, double &y, double &z, double &roll, double &pitch, double &yaw, double &speed) : _Hexapod(hexapod){
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    _speed = speed;
}

void GcodeParser::parseCommand(String command) {
    
    if (command.length() == 0) {
        return;
    }
    
    String caps_command = command.toUpperCase();
    if (caps_command.startsWith("P")) {
        String preset_sel_str = caps_command.substring(1);
        uint8_t preset_sel = 255;
        preset_sel = uint8_t(std::stoi(preset_sel_str.c_str())); 
        performPreset(preset_sel);
    }
    else if (caps_command.startsWith("G")) {
        std::string command_str = caps_command.c_str();
        updateVariables(caps_command);
        _position.set(_x, _y, _z, _roll, _pitch, _yaw);
        performMovement(movement_sel);
        movement_sel = 255;
    }
    else {
        SERIAL_OUTPUT.printf("ERROR! Unsupported Gcode command received via Serial\n");
        SERIAL_OUTPUT.printf("STRING IS: %s", command);
    }

    return;
}

void GcodeParser::performPreset(uint8_t preset) {

    switch(preset) {
        case 0:
            SERIAL_OUTPUT.printf("Gcode parsing success; starfish preset selected (move all motors to zero).\n");
            _Hexapod.moveToZeros();
            break;
        case 1:
            SERIAL_OUTPUT.printf("Gcode parsing success; sit preset selected.\n");
            _Hexapod.sit();
            break;
        case 2:
            SERIAL_OUTPUT.printf("Gcode parsing success; stand preset selected.\n");
            _Hexapod.stand();
            break;
        default:
            SERIAL_OUTPUT.printf("ERROR! Gcode parser detected input for a preset that is not yet supported: %hu.\n", preset);
            break;
    }
    return;
}

void GcodeParser::performMovement(uint8_t movement) {

    switch(movement) {
        case 0:
            SERIAL_OUTPUT.printf("Gcode rapid move parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", _x, _y, _z, _roll, _pitch, _yaw, _speed);
            _Hexapod.rapidMove(_position);
            break;
        case 1:
            SERIAL_OUTPUT.printf("Gcode walk setup parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", _x, _y, _z, _roll, _pitch, _yaw, _speed);
            _Hexapod.walkSetup(_position, _speed);
            break;
        case 2: 
        //NOTE: if not all 18 positions specified, motors who were not explicitly provided will move to where the hexapod has memory of them being. (What is in the parsers _leg_positions array)
            for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
                for (uint8_t axis = 0; axis < NUM_AXES_PER_LEG; axis++) {
                    _Hexapod.legs[leg].axes[axis].moveToPosAtSpeed(_leg_positions[leg][axis], _speed);
                }
            }
            SERIAL_OUTPUT.printf("Gcode move to pos at speed x18 parsing success.\n");
            break;
        case 8:
            if (_movement_time != 0) {
                for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
                    _Hexapod.legEnqueue(leg, ThreeByOne(_leg_positions[leg]), _movement_time, true);
                  }
            }
            SERIAL_OUTPUT.printf("Gcode leg enqueue parsing success.\n");
            break;
        case 9:
            SERIAL_OUTPUT.printf("Gcode linear move setup parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", _x, _y, _z, _roll, _pitch, _yaw, _speed);
            _Hexapod.linearMoveSetup(_position, _speed);
            break;    
        default:
            SERIAL_OUTPUT.printf("ERROR! Gcode parser detected input for a movement that is not yet supported: %hu. \n", movement);
            break;   
    }
    return;
}

void GcodeParser::updateVariables(const String &command) {

    std::string cmd = command.c_str();
    std::stringstream ss(cmd);
    std::string token;
    
    while (ss >> token) {
        if (token[0] == 'G') {
            movement_sel = std::stoi(token.substr(1));  
        }
        else if (token[0] == 'X') {
            _x = std::stod(token.substr(1));  
        }
        else if (token[0] == 'Y') {
            _y = std::stod(token.substr(1)); 
        }
        else if (token[0] == 'Z') {
            _z = std::stod(token.substr(1)); 
        }
        else if (token[0] == 'R') {
            _roll = std::stod(token.substr(1)); 
        }
        else if (token[0] == 'P') {
            _pitch = std::stod(token.substr(1)); 
        }
        else if (token[0] == 'W') {
            _yaw = std::stod(token.substr(1)); 
        }
        else if (token[0] == 'V') {
            _speed = std::stod(token.substr(1)); 
        }
        else if (token[0] == 'T') {
            _movement_time = (std::stoi(token.substr(1)) != 0); 
        }
        else if (token[0] == 'L') {
            uint8_t L_end = token.find('S');  
            uint8_t leg = std::stoi(token.substr(1, L_end - 1)); 
            uint8_t axis = std::stoi(token.substr(L_end + 1, 1)) - 1;  
            double pos = std::stod(token.substr(L_end + 2));  
            if ((leg >= 0 && leg < NUM_LEGS) && (axis >= 0 && axis < NUM_AXES_PER_LEG)) {
                _leg_positions[leg][axis] = pos;
            }
        }
    }
    return;
}

_Bool GcodeParser::optimizableCommand(const String &command) {

    _Bool retval = false;
    String command_copy = command;
    String caps_command = command_copy.toUpperCase();
    std::string cmd = caps_command.c_str();
    std::stringstream ss(cmd);
    std::string token;
    while (ss >> token) {
        if (token[0] == 'G' && token[1] == '1') {
            retval = true;
        }
    }
    return retval;
}
