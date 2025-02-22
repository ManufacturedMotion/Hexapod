#include "voltage_monitor.hpp"
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmath>
#include "config.hpp"
#include <ArduinoJson.h>

uint32_t VoltageSensor::_voltage_sensor_timer = 0;

VoltageSensor::VoltageSensor() {
    _report_interval = 7000;
    _measure_interval = 1000;
    _num_measurements = uint8_t(floor(_report_interval / _measure_interval));
    _change_threshold = 0.15;
    pinMode(VSENSE_PIN, INPUT); //INPUT_DISABLE
}

void VoltageSensor::checkVoltage() {
    if (canRead()) {
        float current_vdd = takeReading();
        if (canSend(current_vdd)) {
            if (current_vdd != 0) {
                _last_measure_time = _voltage_sensor_timer;
                _last_reported_vdd = current_vdd;
                json_voltage["VDD"] = round(current_vdd * 100) / 100.0;
                serializeJson(json_voltage, SERIAL_OUTPUT);
                SERIAL_OUTPUT.println();
                _voltage_sensor_timer = 0;
            }
        }
    }
    _voltage_sensor_timer++;
}

_Bool VoltageSensor::canRead() {
    if ((_voltage_sensor_timer - _last_measure_time) >= _report_interval) {
        return true;
    }
    else {
        return false; 
    }
}

float VoltageSensor::takeReading() {
    uint16_t measure_start_time = _voltage_sensor_timer;
    float raw_vdds[_num_measurements];

    uint16_t measure_times[_num_measurements];
    for (uint8_t measure_index = 0; measure_index < _num_measurements; measure_index++) {
        measure_times[measure_index] = measure_start_time + (measure_index * _measure_interval);
    }

    for (uint8_t measure_index = 0; measure_index < _num_measurements; measure_index++) {
        while(1) {
            if (_voltage_sensor_timer >= measure_times[measure_index]) {
                raw_vdds[measure_index] = analogRead(VSENSE_PIN);
                #if VOLTAGE_DEBUG
                    SERIAL_OUTPUT.printf("raw vdd %hu is %f \n", measure_index, raw_vdds[measure_index]);
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
        SERIAL_OUTPUT.printf("most frequent raw voltage was %f \n", voltage);
    #endif
    return voltage;
}
