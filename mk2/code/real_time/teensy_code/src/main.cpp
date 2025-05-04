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
  Serial.begin(250000);
  Serial4.begin(250000);
  hexapod.startUp();
}

void loop() {

  String command = "";
  hexapod.voltageSensor.checkVoltage(); 

  if (Serial.available() > 0 || Serial4.available() > 0) {
    if (Serial4.available() > 0) {
      command = Serial4.readStringUntil('\n');
    } else {
      command = Serial.readStringUntil('\n');
    }
    #if DEBUG
      SERIAL_OUTPUT.print("Teensy Received: " + command + ".\n");
    #endif
    command_queue.enqueue(command);
  }

  if (!command_queue.isEmpty()) {

    if (hexapod.isBusy()) {
    } else {
      parser.parseCommand(command_queue.dequeue());
    }
  
  }
  //call move every iteration of loop
  hexapod.walkPerform();
  hexapod.comboMovePerform();
  hexapod.runSpeed();
}
