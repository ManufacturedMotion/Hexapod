#include "serial_parser.hpp"
#include "user_config.hpp"
#include "hexapod.hpp"
#include <stdbool.h>
#include "position.hpp"
#include <ArduinoJson.h>
#include "log_levels.hpp"

SerialParser::SerialParser(
    Hexapod &hexapod
)
: _Hexapod(hexapod)
{}

void SerialParser::parseCommand(String command_str) {

    String caps_command = command_str.toUpperCase();
    JsonDocument command;
    DeserializationError json_error = deserializeJson(command, caps_command);

    if (json_error) {
        Serial.printf("Error converting command to JSON object! Could not parse\n");
        Serial.println(json_error.f_str());
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
        _position.set(x, y, z, roll, pitch, yaw);
        performMovement(movement_sel);
        movement_sel = "None";
    }
    else {
        Serial.println("ERROR: Unsupported JSON command received via Serial\n");
    }
    return;
}

String SerialParser::getCommandType(const String &command_str) {
    
    JsonDocument command;
    DeserializationError json_error = deserializeJson(command, command_str);

    if (json_error) {
        Serial.printf("Error converting command to JSON object! Could not parse\n");
        Serial.println(json_error.f_str());
        return "None";
    }

    JsonObject obj = command.as<JsonObject>();
    String key = "";
    for (JsonPair kv : obj) {
        return String(kv.key().c_str());
    }
    return key;
}

void SerialParser::performPreset(String preset) {

    if (preset == "Z") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; starfish preset selected (move all motors to zero).\n");
        #endif
        _Hexapod.moveToZeros();
    }
    else if (preset == "DTCH"){
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; detach all servos preset selected.\n");
        #endif
        _Hexapod.detachAllServos();
    }
    else if (preset == "SIT"){
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; sit preset selected.\n");
        #endif
        _Hexapod.sit();
    }
    else if (preset == "STND"){
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; stand preset selected.\n");
        #endif
        _Hexapod.stand();
    }
    else if (preset == "DNC0") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; dance0 preset selected.\n");
        #endif
        _Hexapod.dance0();
    }
    else if (preset == "DNC1") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; dance1 preset selected.\n");
        #endif
        _Hexapod.dance1();
    }
    else if (preset == "DNC2") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; dance2 preset selected.\n");
        #endif
        _Hexapod.dance2();
    }
    else if (preset == "DNC3") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; dance3 preset selected.\n");
        #endif
        _Hexapod.dance3();
    }
    else if (preset == "DNC4") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; dance4 preset selected.\n");
        #endif
        _Hexapod.dance4();
    }
    else if (preset == "DNC5") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON parsing success; dance5 preset selected.\n");
        #endif
        _Hexapod.dance5();
    }
    else {
        Serial.println("ERROR: JSON parser detected input for a preset that is not yet supported: " + String(preset) + "\n"); 
    }
    // JsonDocument ack;
    // ack["MOVE_TIME"] = 750;
    // serializeJson(ack, Serial4);
    return;
}

void SerialParser::performMovement(String movement) {
    
    if (movement == "RPD") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON rapid move parsing success; x, y, z is " + String (x) + " " + String (y) + " " + String (z) + "\nroll, pitch, yaw, speed are " + 
                    String (roll) + " " + String (pitch) + " " + String (yaw) + " " + String(speed) + "\n");
        #endif
        _Hexapod.enqueueRapidMove(_position);

    }
    else if (movement == "WLK") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON walk setup parsing success; x, y, z is " + String (x) + " " + String (y) + " " + String (z) + "\nroll, pitch, yaw, speed are " + 
                    String (roll) + " " + String (pitch) + " " + String (yaw) + " " + String(speed) + "\n");
        #endif
        _Hexapod.walkSetup(_position, speed);
    }
    else if (movement == "VSET") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON VSET parsing successx, y, z is " + String (x) + " " + String (y) + " " + String (z) + "\nroll, pitch, yaw, speed are " + 
                    String (roll) + " " + String (pitch) + " " + String (yaw) + " " + String(speed) + "\n");
        #endif
        _Hexapod.setWalkVelocity(_position);
        return;
    }
    //NOTE: if not all 18 positions specified, motors who were not explicitly provided will move to where the hexapod has memory of them being. (What is in the parsers _leg_positions array)
    else if (movement == "MTPS") {
        if (valid_MTPS){ 
            for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
                for (uint8_t axis = 0; axis < NUM_AXES_PER_LEG; axis++) {
                    _Hexapod.legs[leg].axes[axis].moveToPosAtSpeed(_leg_positions[leg][axis], speed);
                }
            }
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.println("JSON move to pos at speed x18 parsing success.\n");
            #endif
            valid_MTPS = false;
        }
    }
    else if (movement == "TUNE") {
        _msg = "";
        if ((_tune_leg >= 0 && _tune_leg < NUM_LEGS) && (_tune_axis >= 0 && _tune_axis < NUM_AXES_PER_LEG)) {
            #if LOG_LEVEL >= BASIC_DEBUG
                Serial.println("TUNE move parsing success; moving Leg: " + String (_tune_leg) + ", Axis: " + String (_tune_axis) + "to position " + String (_tune_pos) + "\n");
            #endif
            _Hexapod.moveLegAxisToPos(_tune_leg, _tune_axis, _tune_pos);
        }
    }
    else if (movement == "3B1") {
        if (_movement_time != 0) {
            for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
                _Hexapod.legEnqueue(leg, ThreeByOne(_leg_positions[leg]), _movement_time, true);
              }
        }
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON leg enqueue parsing success.\n");
        #endif
    }
    else if (movement == "LMS") {
        #if LOG_LEVEL >= BASIC_DEBUG
            Serial.println("JSON linear move setup parsing success; x, y, z is " + String (x) + " " + String (y) + " " + String (z) + "\nroll, pitch, yaw, speed are " + 
                    String (roll) + " " + String (pitch) + " " + String (yaw) + " " + String(speed) + "\n");
        #endif
        _Hexapod.linearMoveSetup(_position, speed);
    }
    else {
        Serial.println("ERROR: JSON parser detected input for a movement that is not yet supported: " + String(movement) + "\n");
    }
    return;
}

void SerialParser::updateVariables(const String &command_str) {

    JsonDocument command;
    DeserializationError json_error = deserializeJson(command, command_str);

    if (json_error) {
        Serial.printf("Error converting command to JSON object! Could not parse\n");
        Serial.println(json_error.f_str());
        return;
    }

    JsonObject json_command = command.as<JsonObject>();
    for (JsonPair kv : json_command) {
        String key = String(kv.key().c_str());

        if (key == "MV") {
            movement_sel = String(json_command["MV"]);
        }
        else if (key == "X") {
            x = kv.value().as<double>();
        }
        else if (key == "Y") {
            y = kv.value().as<double>();
        }
        else if (key == "Z") {
            z = kv.value().as<double>();
        }
        else if (key == "ROLL") {
            roll = kv.value().as<double>();
        }
        else if (key == "PTCH") {
            pitch = kv.value().as<double>();
        }
        else if (key == "YAW") {
            yaw = kv.value().as<double>();
        }
        else if (key == "SPD") {
            speed = kv.value().as<double>();
        }
        else if (key == "TIME") {
            _movement_time = kv.value().as<uint16_t>(); 
        }
        else if ((key.indexOf('S') != -1) && (key.indexOf('L') != -1)) {
            if (movement_sel == "TUNE") {
                if (json_command.size() != 2) {
                    Serial.println("ERROR: TUNE command only allows one position specification.\n");
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
                    Serial.println("ERROR: MTPS command requires all 18 positions to be specified.\n");
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
                Serial.println("Warning: Provided Joint key to command that does not consume it.\n");
            }
        }
    }
    return;
}

std::tuple<uint8_t, uint8_t, double> SerialParser::parseJsonJoint(String joint, JsonPair json_pos) {

    uint8_t leg = 0;
    uint8_t axis = 0;
    double pos = 0.0;
    std::tuple<uint8_t, uint8_t, double> error_return = std::make_tuple(25, 25, 0);

    int leg_start_idx = joint.indexOf('L') + 1;
    int axis_start_idx = joint.indexOf('S');

    if (leg_start_idx == 0 || axis_start_idx == -1) {
        Serial.println("ERROR: Joint key requires the form 'L<leg>S<axis>'.\n");
        return error_return;
    }
    if (axis_start_idx == -1 || axis_start_idx == int(joint.length() - 1)) {
        Serial.println("ERROR: Joint key requires a valid 'S' value after 'L<leg>S<axis>'.\n");
        return error_return;
    }

    String leg_str = joint.substring(leg_start_idx, axis_start_idx);
    String axis_str = joint.substring(axis_start_idx + 1);
    if (!isNumeric(leg_str) || !isNumeric(axis_str)) {
        Serial.println("ERROR: Both the leg and axis values must be integers.\n");
        return error_return;
    }
    
    leg = leg_str.toInt(); 
    axis = axis_str.toInt();
    if (leg < 0 || leg >= NUM_LEGS || axis <= 0 || axis > NUM_AXES_PER_LEG) {
        Serial.println("ERROR: Leg or axis value is invalid.\n");
        return error_return;
    }
    axis -= 1; //do this after check because if axis = 0, we would roll back to 255 and bypass the return

    std::string pos_str = json_pos.value(); 
    bool is_valid_number = checkPositionString(pos_str);
    if (!is_valid_number) {
        Serial.println("ERROR: Invalid position format for leg: " + String(leg) + " axis: " + String(axis) + ". Position contains invalid characters: " + String(pos_str.c_str()) + "\n");
        return error_return;
    }

    pos = json_pos.value().as<double>();
    return std::make_tuple(leg, axis, pos);
}

_Bool SerialParser::checkPositionString(std::string pos_str) {
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

_Bool SerialParser::isNumeric(const String &str) {
    for (unsigned int i = 0; i < str.length(); i++) {
        if (!isdigit(str.charAt(i))) {
            return false;
        }
    }
    return true;
}
