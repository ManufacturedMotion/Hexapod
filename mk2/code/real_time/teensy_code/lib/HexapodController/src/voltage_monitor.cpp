#include "voltage_monitor.hpp"
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmath>
#include "config.hpp"
#include <ArduinoJson.h>
#include "log_levels.hpp"

VoltageSensor::VoltageSensor(uint8_t sense_pin, double voltage_divider_factor) {
    _sense_pin = sense_pin;
    _voltage_divider_factor = voltage_divider_factor;
    pinMode(_sense_pin, INPUT); //INPUT_DISABLE
}

double VoltageSensor::directRead() {
    return analogRead(_sense_pin) * _voltage_divider_factor; 
}

double VoltageSensor::filteredRead() {
    if (millis() - _last_read_time < 1000) {
        return _voltage; // Return last value if less than 1s since last read
    }
    _last_read_time = millis();
    if (_voltage < 0.01) {
        _voltage = directRead(); // Initialize voltage if it is less than 0.01V
    }
    _voltage = round2(directRead() * 1.0 / NUM_MEASUREMENTS + _voltage * (NUM_MEASUREMENTS - 1.0) / NUM_MEASUREMENTS);
    return _voltage;
}

double VoltageSensor::round2(double value) {
    return (int)(value * 100 + 0.5) / 100.0;
}