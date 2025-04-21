#include "hexapod_controller.hpp"
#include <math.h>
#include <stdbool.h>
#include <Arduino.h>

Hexapod hexapod;
float voltage_measurement = 0;
Position position;
commandQueue command_queue;

void setup() {
  hexapod.startUp();
}

void loop() {

  String command = "";
  hexapod.voltageSensor.checkVoltage(); 
 
  if (hexapod.serial.msgAvailable()){
    command = hexapod.serial.getMsg();
    command_queue.enqueue(command);
  }

  if (!command_queue.isEmpty()) {

    if (hexapod.isBusy()) {
    } else {
      hexapod.serial.parseCommand(command_queue.dequeue());
    }
  
  }
  //call move every iteration of loop
  hexapod.comboMovePerform();
  hexapod.runSpeed();
}
