#include "serial_handler.hpp"
#include "config.hpp"
#inlcude <stdint.h>
#include <Arduino.h>

SerialHandler::SerialHandler(){
    Serial.begin(250000);
    Serial4.begin(250000);
}

void SerialHandler::writeMsg(String message, uint8_t log_threshold) {

    Serial4.printf(message);

    if(LOG_LEVEL >= log_threshold){
        Serial.printf(message);
    }

}

void SerialHandler::logMsg(String message) {

    Serial.printf(message);

}

bool SerialHandler::msgAvailable(){

    bool msg_available = false;
    if (Serial.available() > 0){
        msg_available = true;
        _read_channel = 1;
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
        case(1)://serial
            msg = Serial.readStringUntil('\n');
            break;
        case(2): //serial 4
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