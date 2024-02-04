#include "HexapodController.h"

Axis axes[6][3] = {
    {Axis(2, -PI * 0.75, PI * 0.75), Axis(3, -PI * 0.75, PI * 0.75), Axis(4, -PI * 0.75, PI * 0.75)},
    {Axis(5, -PI * 0.75, PI * 0.75), Axis(6, -PI * 0.75, PI * 0.75), Axis(7, -PI * 0.75, PI * 0.75)},
    {Axis(8, -PI * 0.75, PI * 0.75), Axis(9, -PI * 0.75, PI * 0.75), Axis(10, -PI * 0.75, PI * 0.75)},
    {Axis(11, -PI * 0.75, PI * 0.75), Axis(12, -PI * 0.75, PI * 0.75), Axis(13, -PI * 0.75, PI * 0.75)},
    {Axis(14, -PI * 0.75, PI * 0.75), Axis(15, -PI * 0.75, PI * 0.75), Axis(18, -PI * 0.75, PI * 0.75)},
    {Axis(19, -PI * 0.75, PI * 0.75), Axis(22, -PI * 0.75, PI * 0.75), Axis(23, -PI * 0.75, PI * 0.75)}
};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  for (uint8_t i = 0; i < 6; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      axes[i][j].set_mapping(1,0.75);
    }
  }
}

void loop() {
  double pos;
  uint8_t leg;
  uint8_t motor;

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    Serial.println("Received");
    sscanf(command.c_str(), "%c %c %lf", &leg, &motor, &pos);
    
    Serial.printf("leg: %d; motor: %d; pos: %f", leg, motor, pos);
    axes[leg][motor].move_to_pos(pos);
  }

}
