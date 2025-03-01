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
    json_joints = getJsonJoints();
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
        SERIAL_OUTPUT.printf("ERROR! Unsupported JSON command received via Serial\n");
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

_Bool JsonParser::optimizableCommand(String command_str) {

    String caps_command = command_str.toUpperCase();
    JsonDocument command;
    DeserializationError json_error = deserializeJson(command, caps_command);

    if (json_error) {
        SERIAL_OUTPUT.printf("Error converting command to JSON object! Could not parse\n");
        SERIAL_OUTPUT.println(json_error.f_str());
        return false;
      }
  
    String command_type = getCommandType(caps_command);

    if ((command_type == "PRE") or (command_type == "")) {
        return false;
    }
    else {
        String movement_sel = "None";
        movement_sel = String(command["MV"]);
        if (movement_sel == "WALK") { 
            return true;
        }
        else {
            return false;
        }
    }
    return false;

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
        SERIAL_OUTPUT.printf("ERROR! JSON parser detected input for a preset that is not yet supported: %s.\n", preset);
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
        for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
            for (uint8_t axis = 0; axis < NUM_AXES_PER_LEG; axis++) {
                _Hexapod.legs[leg].axes[axis].moveToPosAtSpeed(_leg_positions[leg][axis], _speed);
            }
        }
        SERIAL_OUTPUT.printf("JSON move to pos at speed x18 parsing success.\n");
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
        SERIAL_OUTPUT.printf("ERROR! JSON parser detected input for a movement that is not yet supported: %hu. \n", movement);
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
            uint8_t leg = json_joints[key]["leg"];
            uint8_t axis = json_joints[key]["axis"];
            double pos = kv.value().as<double>();
            if ((leg >= 0 && leg < NUM_LEGS) && (axis >= 0 && axis < NUM_AXES_PER_LEG)) {
                _leg_positions[leg][axis] = pos;
            }
        }
    }
    return;
}
