#include "voltage_monitor.hpp"
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmath>
#include "config.hpp"
#include <ArduinoJson.h>
#include "log_levels.hpp"

uint32_t VoltageSensor::_voltage_sensor_timer = 0;

VoltageSensor::VoltageSensor() {
    _report_interval = 7000;
    _measure_interval = uint16_t(floor(_report_interval / NUM_MEASUREMENTS));
    _change_threshold = 0.1;
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
                serializeJson(json_voltage, Serial4);
                Serial4.println("");
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
    static uint32_t measure_start_time = 0;
    static float raw_vdds[NUM_MEASUREMENTS];
    static uint32_t measure_times[NUM_MEASUREMENTS];
    
    if (measure_start_time == 0)  {
        measure_start_time = _voltage_sensor_timer;
        for (uint8_t measure_index = 0; measure_index < NUM_MEASUREMENTS; measure_index++) {
            measure_times[measure_index] = measure_start_time + (measure_index * _measure_interval);
        }
    }

    for (uint8_t measure_index = 0; measure_index < NUM_MEASUREMENTS; measure_index++) {
        if ((_voltage_sensor_timer >= measure_times[measure_index]) && measure_times[measure_index] != 0) {
            raw_vdds[measure_index] = analogRead(VSENSE_PIN);
            //overwrite measure time after saving vdd. prevents same reading from being recorded multiple times
            measure_times[measure_index] = 0; 
            #if LOG_LEVEL >= VOLTAGE_DEBUG 
                Serial.println("raw vdd " + String(measure_index) + " is " + String(raw_vdds[measure_index]) + "\n");
            #endif
        }
    }

    //only report vdd if we took all measurements
    if (measure_times[NUM_MEASUREMENTS - 1] == 0) {
        float raw_vdd = getMode(raw_vdds, NUM_MEASUREMENTS);
        measure_start_time = 0;
        for (uint8_t i = 0; i < NUM_MEASUREMENTS; i++) {
            raw_vdds[i] = 0;
        }
        return (raw_vdd * VSENSE_FACTOR); 
    }
    else {
        return 0;
    }
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
    #if LOG_LEVEL >= VOLTAGE_DEBUG
        Serial.println("most frequent raw voltage was " + String(voltage) + "\n");
    #endif
    return voltage;
}
