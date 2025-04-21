#include "serial_handler.hpp"
#include "config.hpp"
#include <stdint.h>
#include <Arduino.h>

//TODO - move all hexapod execute code inside of hexapod class
SerialHandler::SerialHandler(Hexapod &hexapod,
    double &x, double &y, double &z,
    double &roll, double &pitch, double &yaw, double &speed)
: _JsonParser(hexapod, x, y, z, roll, pitch, yaw, speed)
#if LOG_LEVEL > 0
, _GcodeParser(hexapod, x, y, z, roll, pitch, yaw, speed)
#endif
{
    Serial.begin(250000);
    Serial4.begin(250000);
}

void SerialHandler::writeMsg(String message, uint8_t log_threshold) {

    Serial4.println(message);

    if(LOG_LEVEL >= log_threshold){
        Serial.println(message);
    }

}

void SerialHandler::logMsg(String message) {

    Serial.println(message);

}

bool SerialHandler::msgAvailable(){

    bool msg_available = false;
    if (Serial.available() > 0){
        msg_available = true;
        _read_channel = 1;
    }
    else if (Serial4.available() > 0) {
        msg_available = true;
        _read_channel = 2;
    }

    return msg_available;

}

String SerialHandler::getMsg(){

    String msg = "";

    switch(_read_channel){
        default:
            Serial.printf("ERROR! getMsg called with incorrect read channel selected, %u! Only supports 1 & 2\n", _read_channel);
            break;
        case 1://serial
            msg = Serial.readStringUntil('\n');
            break;
        case 2: //serial 4
            msg = Serial4.readStringUntil('\n');
            break;
    }

    if(LOG_LEVEL >= BASIC_DEBUG){
        Serial.printf("Teensy recieved message: %s\n", msg.c_str());
    }

}

void SerialHandler::errorMsg(String error_msg){

    Serial.printf("ERROR! %s \n", error_msg.c_str());
    Serial4.printf("ERROR! %s \n", error_msg.c_str());
    
}

void SerialHandler::writeJsonMsg(JsonDocument json_msg, uint8_t log_threshold){

    serializeJson(json_msg, Serial4);
    Serial4.println();

    if (LOG_LEVEL >= log_threshold){
        serializeJson(json_msg, Serial);
        Serial.println();
    }

}

void SerialHandler::parseCommand(String command) {

    #if LOG_LEVEL == 0
        if (!command.startsWith("{")) {
            errorMsg("ERROR! Serial command not in json format. This is not supported unles DEBUG set to true in user_config.hpp\n");
            return;
        }
    #endif

    if (command.startsWith("{")) {
        _JsonParser.parseCommand(command);
    }
    else {
        #if LOG_LEVEL
            _GcodeParser.parseCommand(command);
        #endif
    }
    
}