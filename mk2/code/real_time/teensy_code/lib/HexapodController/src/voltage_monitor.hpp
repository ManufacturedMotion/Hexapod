#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <ArduinoJson.h>

#define VSENSE_PIN 38 
#define VSENSE_FACTOR 0.01612903225 // 5.0 * (3.3 / 1023.0)

#ifndef VOLT_SENSE
#define VOLT_SENSE

    #define NUM_MEASUREMENTS 50

    class VoltageSensor {
        public:
            VoltageSensor(uint8_t sense_pin=VSENSE_PIN, double voltage_divider_factor=VSENSE_FACTOR);
            double filteredRead();
            double directRead();

        private:
            uint32_t _last_read_time = 0;
            double _voltage = -1.0; // Initialize to -1.0 to indicate uninitialized
            uint8_t _sense_pin;
            double _voltage_divider_factor;
            double round2(double value);
    };

#endif
