#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#define VSENSE_PIN 38 
//#define VSENSE_FACTOR 0.016113
#define VSENSE_FACTOR 0.016325 //still too low? sensor seems current based not vdd??

#define VOLTAGE_DEBUG false

#ifndef VOLT_SENSE
#define VOLT_SENSE

    class VoltageSensor {
        public:
            VoltageSensor();
            float checkVoltage();
            _Bool canRead();
            float takeReading();
            _Bool canSend(float voltage);
            float getMode(float raw_vdds[], const uint8_t num_measurements);
            
        private:
            uint16_t _measure_interval = 0;
            uint16_t _record_interval = 0;
            uint8_t _num_measurements = 0;
            float _change_threshold = 0;
            uint32_t _last_measure_time = 0;
            float _last_reported_vdd = 0;
            float _current_vdd = 0;
            float _last_raw_vdd = 0;
    };

#endif
