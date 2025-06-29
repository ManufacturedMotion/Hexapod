#include "hexapod_controller.hpp"
#include <math.h>
#include <stdbool.h>
#include <Arduino.h>

Hexapod hexapod;
SerialParser parser(hexapod);
double last_voltage_measurement = 0;
Position position;
commandQueue command_queue;

VoltageSensor * voltage_sensor;
JsonDocument voltage_json;

void setup() {
  voltage_sensor = new VoltageSensor();
  hexapod.startUp();
  #if LOG_LEVEL > 0
    Serial.begin(250000);
  #endif
  Serial4.begin(250000);
}

void loop() {

  String command = "";
  double voltage_measurement = voltage_sensor->filteredRead();
  if (fabs(voltage_measurement - last_voltage_measurement) > 0.01) {
    last_voltage_measurement = voltage_measurement;
    #if LOG_LEVEL > 0
      Serial.printf("Voltage: %.2f V\n", last_voltage_measurement);
    #endif
    voltage_json.clear();
    voltage_json["VDD"] = last_voltage_measurement;
    serializeJson(voltage_json, Serial4);
  }

  #if LOG_LEVEL > 0
    if (Serial.available() > 0 || Serial4.available() > 0) {
      if (Serial4.available() > 0) {
        command = Serial4.readStringUntil('\n');
        command_queue.enqueue(command);
      } else {
        command = Serial.readStringUntil('\n');
        command_queue.enqueue(command);
      }  
    }
  #else
    if (Serial4.available() > 0) {
      command = Serial4.readStringUntil('\n');
      command_queue.enqueue(command);
    }
  #endif 

  if (!command_queue.isEmpty()) {

    if (!hexapod.isBusy()) {
      parser.parseCommand(command_queue.dequeue()); 
    }
  
  }
  //call move every iteration of loop
  // hexapod.run();
  hexapod.walkPerform();
  hexapod.comboMovePerform();
  hexapod.runSpeed();
}
