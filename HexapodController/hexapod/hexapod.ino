#include "hexapod_controller.hpp"

Hexapod hexapod; 

void setup() {

  Serial.begin(115200);

}

void loop() {
  double pos;
  uint8_t leg;
  uint8_t motor;


  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    Serial.println("Received");
    sscanf(command.c_str(), "%hu %hu %lf", &leg, &motor, &pos);
    
    Serial.printf("leg: %d; motor: %d; pos: %f", leg, motor, pos);
    hexapod.legs[leg - 1].axes[motor -1].moveToPos(pos);
  }


}
