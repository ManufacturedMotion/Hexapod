#include "user_config.hpp"
#include <stdint.h>
#include <Arduino.h>
#include <ArduinoJson.h>

#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

    class Hexapod;

    #include "json_parser.hpp"

    #if LOG_LEVEL > 0
        #include "gcode_parser.hpp"
    #endif

    class JsonParser;
    class GcodeParser;

    //available log levels
    #define BASIC_DEBUG 1 
    #define VOLTAGE_DEBUG
    #define CALCULATION_LOGGING 3

    class SerialHandler {

        public:
            //SerialHandler(Hexapod &hexapod,
             //   double &x, double &y, double &z,
              //  double &roll, double &pitch, double &yaw, double &speed);
            SerialHandler();
            void writeMsg(String message, uint8_t log_threshold);
            bool msgAvailable();
            String getMsg();
            void logMsg(String message);
            void errorMsg(String error_msg);
            void writeJsonMsg(JsonDocument json_msg, uint8_t log_threshold);
            void parseCommand(String command);

        private:
            uint8_t _read_channel = 2;
            JsonParser _JsonParser;
            #if LOG_LEVEL > 0
                GcodeParser _GcodeParser;
            #endif
    };

#endif