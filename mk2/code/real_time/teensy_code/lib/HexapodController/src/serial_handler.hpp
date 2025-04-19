#include "user_config.hpp"
#include <stdint.h>
#include <Arduino.h>
#include <ArduinoJson.h>

#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

    //available log levels
    #define BASIC_DEBUG 1 
    #define VOLTAGE_DEBUG
    #define CALCULATION_LOGGING 3

    class SerialHandler {

        public:
            SerialHandler();
            void writeMsg(String message, uint8_t log_threshold);
            bool msgAvailable();
            String getMsg();
            void logMsg(String message, uint8_t log_threshold);
            void errorMsg(String error_msg);
            void writeJsonMsg(JsonDocument json_msg, uint8_t log_threshold);

        private:
            _read_channel = 2;
            
    };

#endif