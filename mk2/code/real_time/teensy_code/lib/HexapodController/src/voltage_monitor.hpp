#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <ArduinoJson.h>

#define VSENSE_PIN 38 
#define VSENSE_FACTOR 0.016113 

#define VOLTAGE_DEBUG false

#ifndef VOLT_SENSE
#define VOLT_SENSE

    #define NUM_MEASUREMENTS 7

    class VoltageSensor {
        public:
            VoltageSensor(SerialHandler* serial_handler);
            void checkVoltage();
            _Bool canRead();
            float takeReading();
            _Bool canSend(float voltage);
            float getMode(float raw_vdds[], const uint8_t num_measurements);
            JsonDocument json_voltage;
            
        private:
            uint16_t _measure_interval = 0;
            uint16_t _report_interval = 0;
            float _change_threshold = 0;
            uint32_t _last_measure_time = 0;
            float _last_reported_vdd = 0;
            float _current_vdd = 0;
            float _last_raw_vdd = 0;
            static uint32_t _voltage_sensor_timer;
            SerialHandler* _serial = nullptr;
    };

#endif
