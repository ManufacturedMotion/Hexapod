#include "voltage_monitor.hpp"
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmath>


//TODO - check if this stuff is blocking or nonblocking (can we move motors while measuring?)
//after above fixed can work on polishing factor and change threshold

VoltageSensor::VoltageSensor() {
    _record_interval = 3000;
    _measure_interval = 428;
    _num_measurements = uint8_t(floor(_record_interval / _measure_interval));
    _change_threshold = 5;
    pinMode(VSENSE_PIN, INPUT);
}

float VoltageSensor::checkVoltage() {
    if (canRead()) {
        //TODO - two options; only report new vdd OR always report measured vdd every record interval
        float current_vdd = takeReading();
        //if (canSend(current_vdd)) {
        if (current_vdd != 0) {
            _last_measure_time = millis();
            _last_reported_vdd = current_vdd;
            return current_vdd;
        }
        return 0;
    }
    else {
        return 0; 
    }
}

_Bool VoltageSensor::canRead() {
    if ((millis() - _last_measure_time) >= _record_interval) {
        return true;
    }
    else {
        return false; 
    }
}

float VoltageSensor::takeReading() {
    uint32_t measure_start_time = millis();
    float raw_vdds[_num_measurements];

    uint32_t measure_times[_num_measurements];
    for (uint8_t measure_index = 0; measure_index < _num_measurements; measure_index++) {
        measure_times[measure_index] = measure_start_time + (measure_index * _measure_interval);
    }

    for (uint8_t measure_index = 0; measure_index < _num_measurements; measure_index++) {
        while(1) {
            if (millis() >= measure_times[measure_index]) {
                raw_vdds[measure_index] = analogRead(VSENSE_PIN);
                #if VOLTAGE_DEBUG
                    Serial.printf("raw vdd %hu is %f \n", measure_index, raw_vdds[measure_index]);
                #endif
                break;
            }
        }
    }

    float raw_vdd = getMode(raw_vdds, _num_measurements);
    return (round((raw_vdd * VSENSE_FACTOR) * 100) / 100); 
}

_Bool VoltageSensor::canSend(float voltage) {
    if (abs(voltage - _last_reported_vdd) > _change_threshold) {
        return true;
    }
    else {
        return false;
    }
}

float VoltageSensor::getMode(float voltages[], const uint8_t num_measurements) {

    uint8_t most_occurrences = 0;
    float voltage = 0;

    for (uint8_t i = 0; i < num_measurements; i++) {
        uint8_t occurrences = 0;

        for (uint8_t j = 0; j < num_measurements; j++) {
            if (voltages[i] == voltages[j]){
                occurrences++;
            }
        }

        if (occurrences > most_occurrences) {
            most_occurrences = occurrences;
            voltage = voltages[i];
        }

        //something wrong if we have x different measurements
        else if (most_occurrences == 1) {
            voltage = 0;
        }
    }
    #if VOLTAGE_DEBUG
        Serial.printf("most frequent raw voltage was %f \n", voltage);
    #endif
    return voltage;
}
