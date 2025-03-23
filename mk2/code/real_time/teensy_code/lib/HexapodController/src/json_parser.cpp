#include "json_parser.hpp"
#include "hexapod.hpp"
#include <stdbool.h>
#include <ArduinoJson.h>
#include "position.hpp"
#include "json_joints.hpp"

JsonParser::JsonParser(Hexapod &hexapod, double &x, double &y, double &z, double &roll, double &pitch, double &yaw, double &speed) : _Hexapod(hexapod) {
    _x = x;
    _y = y;
    _z = z;
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    _speed = speed;
}

void JsonParser::parseCommand(String command_str) {

    String caps_command = command_str.toUpperCase();
    JsonDocument command;
    DeserializationError json_error = deserializeJson(command, caps_command);

    if (json_error) {
      SERIAL_OUTPUT.printf("Error converting command to JSON object! Could not parse\n");
      SERIAL_OUTPUT.println(json_error.f_str());
      return;
    }

    String command_type = getCommandType(caps_command);
    if (command_type == "PRE"){
        String preset_sel = "None";
        preset_sel = String(command["PRE"].as<const char*>()); 
        performPreset(preset_sel);  
    }
    else if (command_type == "MV") {
        updateVariables(caps_command); 
        _position.set(_x, _y, _z, _roll, _pitch, _yaw);
        performMovement(movement_sel);
        movement_sel = "None";
    }
    else {
        SERIAL_OUTPUT.printf("ERROR: Unsupported JSON command received via Serial\n");
    }
    return;
}

String JsonParser::getCommandType(const String &command_str) {
    
    JsonDocument command;
    DeserializationError json_error = deserializeJson(command, command_str);

    if (json_error) {
        SERIAL_OUTPUT.printf("Error converting command to JSON object! Could not parse\n");
        SERIAL_OUTPUT.println(json_error.f_str());
        return "None";
    }

    JsonObject obj = command.as<JsonObject>();
    String key = "";
    for (JsonPair kv : obj) {
        return String(kv.key().c_str());
    }
    return key;
}

void JsonParser::performPreset(String preset) {

    if (preset == "Z") {
        SERIAL_OUTPUT.printf("JSON parsing success; starfish preset selected (move all motors to zero).\n");
        _Hexapod.moveToZeros();
    }
    else if (preset == "SIT"){
        SERIAL_OUTPUT.printf("JSON parsing success; sit preset selected.\n");
        _Hexapod.sit();
    }
    else if (preset == "STND"){
        SERIAL_OUTPUT.printf("JSON parsing success; stand preset selected.\n");
        _Hexapod.stand();
    }
    else {
        SERIAL_OUTPUT.printf("ERROR: JSON parser detected input for a preset that is not yet supported: %s.\n", preset);
    }
    return;
}

void JsonParser::performMovement(String movement) {
    
    if (movement == "RPD") {
        SERIAL_OUTPUT.printf("JSON rapid move parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", _x, _y, _z, _roll, _pitch, _yaw, _speed);
        _Hexapod.rapidMove(_position);
    }
    else if (movement == "WLK") {
        SERIAL_OUTPUT.printf("JSON walk setup parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", _x, _y, _z, _roll, _pitch, _yaw, _speed);
        _Hexapod.walkSetup(_position, _speed);
    }
    //NOTE: if not all 18 positions specified, motors who were not explicitly provided will move to where the hexapod has memory of them being. (What is in the parsers _leg_positions array)
    else if (movement == "MTPS") {
        if (valid_MTPS){ 
            for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
                for (uint8_t axis = 0; axis < NUM_AXES_PER_LEG; axis++) {
                    _Hexapod.legs[leg].axes[axis].moveToPosAtSpeed(_leg_positions[leg][axis], _speed);
                }
            }
            SERIAL_OUTPUT.printf("JSON move to pos at speed x18 parsing success.\n");
            valid_MTPS = false;
        }
    }
    else if (movement == "TUNE") {
        if ((_tune_leg >= 0 && _tune_leg < NUM_LEGS) && (_tune_axis >= 0 && _tune_axis < NUM_AXES_PER_LEG)) {
            _Hexapod.moveLegAxisToPos(_tune_leg, _tune_axis, _tune_pos);
            SERIAL_OUTPUT.printf("TUNE move parsing success; moving Leg %d, Axis %d to position %f\n", _tune_leg, _tune_axis, _tune_pos);
        }
    }
    else if (movement == "3B1") {
        if (_movement_time != 0) {
            for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
                _Hexapod.legEnqueue(leg, ThreeByOne(_leg_positions[leg]), _movement_time, true);
              }
        }
        SERIAL_OUTPUT.printf("JSON leg enqueue parsing success.\n");
    }
    else if (movement == "LMS") {
        SERIAL_OUTPUT.printf("JSON linear move setup parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", _x, _y, _z, _roll, _pitch, _yaw, _speed);
        _Hexapod.linearMoveSetup(_position, _speed);
    }
    else {
        SERIAL_OUTPUT.printf("ERROR: JSON parser detected input for a movement that is not yet supported: %hu. \n", movement);
    }
    return;
}

void JsonParser::updateVariables(const String &command_str) {

    JsonDocument command;
    DeserializationError json_error = deserializeJson(command, command_str);

    if (json_error) {
      SERIAL_OUTPUT.printf("Error converting command to JSON object! Could not parse\n");
      SERIAL_OUTPUT.println(json_error.f_str());
      return;
    }

    JsonObject json_command = command.as<JsonObject>();
    for (JsonPair kv : json_command) {
        String key = String(kv.key().c_str());

        if (key == "MV") {
            movement_sel = String(json_command["MV"]);
        }
        else if (key == "X") {
            _x = kv.value().as<double>();
        }
        else if (key == "Y") {
            _y = kv.value().as<double>();
        }
        else if (key == "Z") {
            _z = kv.value().as<double>();
        }
        else if (key == "ROLL") {
            _roll = kv.value().as<double>();
        }
        else if (key == "PTCH") {
            _pitch = kv.value().as<double>();
        }
        else if (key == "YAW") {
            _yaw = kv.value().as<double>();
        }
        else if (key == "SPD") {
            _speed = kv.value().as<double>();
        }
        else if (key == "TIME") {
            _movement_time = kv.value().as<uint16_t>(); 
        }
        else if ((key.indexOf('S') != -1) && (key.indexOf('L') != -1)) {
            if (movement_sel == "TUNE") {
                if (json_command.size() != 2) {
                    SERIAL_OUTPUT.println("ERROR: TUNE command only allows one position specification.");
                    return;
                }
                auto [tune_leg, tune_axis, tune_pos] = parseJsonJoint(key, kv);
                _tune_leg = tune_leg;
                _tune_axis = tune_axis;
                _tune_pos = tune_pos;
                return;
            }
            
            else if (movement_sel == "MTPS") {
                valid_MTPS = true;
                if (json_command.size() != 19) {
                    SERIAL_OUTPUT.println("ERROR: MTPS command requires all 18 positions to be specified.");
                    valid_MTPS = false;
                    return;
                }
                auto [leg, axis, pos] = parseJsonJoint(key, kv);
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

std::tuple<uint8_t, uint8_t, double> JsonParser::parseJsonJoint(String joint, JsonPair json_pos) {

    uint8_t leg = 0;
    uint8_t axis = 0;
    double pos = 0.0;
    std::tuple<uint8_t, uint8_t, double> error_return = std::make_tuple(25, 25, 0);

    int leg_start_idx = joint.indexOf('L') + 1;
    int axis_start_idx = joint.indexOf('S');

    if (leg_start_idx == 0 || axis_start_idx == -1) {
        SERIAL_OUTPUT.println("ERROR: Joint key requires the form 'L<leg>S<axis>'.");
        return error_return;
    }
    if (axis_start_idx == -1 || axis_start_idx == int(joint.length() - 1)) {
        SERIAL_OUTPUT.println("ERROR: Joint key requires a valid 'S' value after 'L<leg>S<axis>'.");
        return error_return;
    }

    String leg_str = joint.substring(leg_start_idx, axis_start_idx);
    String axis_str = joint.substring(axis_start_idx + 1);
    if (!isNumeric(leg_str) || !isNumeric(axis_str)) {
        SERIAL_OUTPUT.println("ERROR: Both the leg and axis values must be integers.");
        return error_return;
    }
    
    leg = leg_str.toInt(); 
    axis = axis_str.toInt();
    if (leg < 0 || leg >= NUM_LEGS || axis <= 0 || axis > NUM_AXES_PER_LEG) {
        SERIAL_OUTPUT.println("ERROR: Leg or axis value is invalid.");
        return error_return;
    }
    axis -= 1; //do this after check because if axis = 0, we would roll back to 255 and bypass the return

    std::string pos_str = json_pos.value(); 
    bool is_valid_number = checkPositionString(pos_str);
    if (!is_valid_number) {
        SERIAL_OUTPUT.printf("ERROR: Invalid position format for leg %d, axis %d. Position contains invalid characters: '%s'\n", leg, axis, pos_str.c_str());
        return error_return;
    }

    pos = json_pos.value().as<double>();
    return std::make_tuple(leg, axis, pos);
}

_Bool JsonParser::checkPositionString(std::string pos_str) {
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

_Bool JsonParser::isNumeric(const String &str) {
    for (unsigned int i = 0; i < str.length(); i++) {
        if (!isdigit(str.charAt(i))) {
            return false;
        }
    }
    return true;
}