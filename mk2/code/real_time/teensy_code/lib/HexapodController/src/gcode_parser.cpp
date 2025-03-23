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
        SERIAL_OUTPUT.printf("ERROR: Unsupported Gcode command received via Serial\n");
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
            SERIAL_OUTPUT.printf("ERROR: Gcode parser detected input for a preset that is not yet supported: %hu.\n", preset);
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
            if (valid_MTPS){
                for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
                    for (uint8_t axis = 0; axis < NUM_AXES_PER_LEG; axis++) {
                        _Hexapod.legs[leg].axes[axis].moveToPosAtSpeed(_leg_positions[leg][axis], _speed);
                    }
                }
                SERIAL_OUTPUT.printf("Gcode move to pos at speed x18 parsing success.\n");
                valid_MTPS = false;
            }   
            break;
        case 3:
            if ((_tune_leg >= 0 && _tune_leg < NUM_LEGS) && (_tune_axis >= 0 && _tune_axis < NUM_AXES_PER_LEG)) {
                SERIAL_OUTPUT.printf("TUNE move parsing success; moving Leg %d, Axis %d to position %f\n", _tune_leg, _tune_axis, _tune_pos);
                _Hexapod.moveLegAxisToPos(_tune_leg, _tune_axis, _tune_pos);
                _tune_leg = 25;
                _tune_axis = 25;
            }
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
            SERIAL_OUTPUT.printf("ERROR: Gcode parser detected input for a movement that is not yet supported: %hu. \n", movement);
            break;   
    }
    return;
}

void GcodeParser::updateVariables(const String &command) {

    std::string cmd = command.c_str();
    std::stringstream ss(cmd);
    std::string token;

    uint8_t gcode_words = 0;
    while (ss >> token) {
        gcode_words++;
    }

    ss.clear();                
    ss.seekg(0, std::ios::beg);

    while (ss >> token) {

        if (token[0] == 'G') {
            movement_sel = std::stoi(token.substr(1));
            continue;
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
            if (movement_sel == 3) {
                if (gcode_words != 2) {
                    SERIAL_OUTPUT.println("ERROR: G3 command only allows one position specification.");
                    return;
                }
                auto [tune_leg, tune_axis, tune_pos] = parseGcodeJoint(token);
                _tune_leg = tune_leg;
                _tune_axis = tune_axis;
                _tune_pos = tune_pos;
                return;
            }
            
            else if (movement_sel == 2) {
                valid_MTPS = true;
                if (gcode_words != 19) {
                    SERIAL_OUTPUT.println("ERROR: G2 command requires all 18 positions to be specified.");
                    valid_MTPS = false;
                    return;
                }
                auto [leg, axis, pos] = parseGcodeJoint(token);
                if ((leg >= 0 && leg < NUM_LEGS) && (axis >= 0 && axis < NUM_AXES_PER_LEG)) {
                    _leg_positions[leg][axis] = pos;
                }
                else {
                    valid_MTPS = false;
                }
            }
            else {
                SERIAL_OUTPUT.println("Warning: Provided Joint key to command that does not consume it.");
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

std::tuple<uint8_t, uint8_t, double> GcodeParser::parseGcodeJoint(std::string joint) {

    uint8_t leg = 0;
    uint8_t axis = 0;
    double pos = 0.0;
    std::tuple<uint8_t, uint8_t, double> error_return = std::make_tuple(25, 25, 0);

    uint8_t L_pos = 0;  
    uint8_t S_pos = joint.find('S');
    if (S_pos == 255) {
        SERIAL_OUTPUT.printf("ERROR: Invalid format for command. Missing 'S' after 'L' for leg %f. Expected format: 'L<leg>S<axis><position>'.\n", leg);
        return error_return;
    }

    std::string leg_str = joint.substr(L_pos + 1, S_pos - L_pos - 1); 
    leg = std::stoi(leg_str);
    std::string axis_str = joint.substr(S_pos + 1, 1);  
    if (axis_str.empty() || axis_str.find_first_not_of(" \t\n\r") == std::string::npos) {
        SERIAL_OUTPUT.printf("ERROR: No axis given for leg %d. Expected format: 'L<leg>S<axis><position>'.\n", leg);
        return error_return;
    } 

    if (!isNumeric(leg_str) || !isNumeric(axis_str)) {
        SERIAL_OUTPUT.println("ERROR: Both the leg and axis values must be integers.");
        return error_return;
    }

    axis = std::stoi(axis_str); 
    if (leg < 0 || leg >= NUM_LEGS || axis <= 0 || axis > NUM_AXES_PER_LEG) {
        SERIAL_OUTPUT.println("ERROR: Leg or axis value is invalid.");
        return error_return;
    }
    axis -= 1; //do this after check because if axis = 0, we would roll back to 255 and bypass the return

    std::string pos_str = joint.substr(S_pos + 2); 
    if (pos_str.empty() || pos_str.find_first_not_of(" \t\n\r") == std::string::npos) {
        SERIAL_OUTPUT.printf("ERROR: No position given for leg %d, axis %d. Expected format: 'L<leg>S<axis><position>'.\n", leg, axis);
        return error_return;
    }

    bool is_valid_number = checkPositionString(pos_str);
    if (!is_valid_number) {
        SERIAL_OUTPUT.printf("ERROR: Invalid position format for leg %d, axis %d. Position contains invalid characters: '%s'\n", leg, axis, pos_str.c_str());
        return error_return;
    }
    
    pos = std::stod(pos_str);
    return std::make_tuple(leg, axis, pos);
}

_Bool GcodeParser::checkPositionString(std::string pos_str) {
    bool is_valid_number = true;
    bool decimal_point_found = false;
    for (char c : pos_str) {
        if (!std::isdigit(c) && c != '.' && c != '-' && c != '+') {
            is_valid_number = false;
            break;
        }
        if (c == '.') {
            if (decimal_point_found) {
                is_valid_number = false; 
                break;
            }
            decimal_point_found = true;
        }
    }
    return is_valid_number;
}

_Bool GcodeParser::isNumeric(const std::string &str) {
    for (unsigned int i = 0; i < str.length(); i++) {
        if (!std::isdigit(static_cast<unsigned char>(str[i]))) {
            return false;
        }
    }
    return true;
}