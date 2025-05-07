#include "hexapod_controller.hpp"
#include <math.h>
#include <stdbool.h>
#include <Arduino.h>

Hexapod hexapod;
SerialParser parser(hexapod);
float voltage_measurement = 0;
Position position;
commandQueue command_queue;

void setup() {
  hexapod.startUp();
  #if LOG_LEVEL > 0
    Serial.begin(250000);
  #endif
  Serial4.begin(250000);
}

void loop() {

  String command = "";
  hexapod.voltageSensor.checkVoltage(); 
  
  #if LOG_LEVEL > 0
    if (Serial.available() > 0 || Serial4.available() > 0) {
      if (Serial4.available() > 0) {
        command = Serial4.readStringUntil('\n');
      } else {
        command = Serial.readStringUntil('\n');
      }
      command_queue.enqueue(command);
    }
  #else
    if (Serial4.available() > 0) {
      command = Serial4.readStringUntil('\n');
    }
    command_queue.enqueue(command);
  #endif 

  if (!command_queue.isEmpty()) {

    if (!hexapod.isBusy()) {
      String next_command = command_queue.dequeue();
      if (next_command.length() > 0 && next_command.startsWith("{") && next_command.endsWith("}")) {
        parser.parseCommand(next_command);
    }
  
  }
  //call move every iteration of loop
  // hexapod.run();
  hexapod.walkPerform();
  hexapod.comboMovePerform();
  hexapod.runSpeed();
}
