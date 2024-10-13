#include "hexapod_controller.hpp"

Axis axes[6][3] = {};

void setup() {
  // put your setup code here, to run once:

    axes[0][0].initializePositionLimits(2, -PI * 0.75, PI * 0.75);
    axes[0][1].initializePositionLimits(3, -PI * 0.75, PI * 0.75);
    axes[0][2].initializePositionLimits(4, -PI * 0.75, PI * 0.75);
    axes[1][0].initializePositionLimits(5, -PI * 0.75, PI * 0.75);
    axes[1][1].initializePositionLimits(6, -PI * 0.75, PI * 0.75);
    axes[1][2].initializePositionLimits(7, -PI * 0.75, PI * 0.75);
    axes[2][0].initializePositionLimits(8, -PI * 0.75, PI * 0.75);
    axes[2][1].initializePositionLimits(9, -PI * 0.75, PI * 0.75);
    axes[2][2].initializePositionLimits(10, -PI * 0.75, PI * 0.75);
    axes[3][0].initializePositionLimits(11, -PI * 0.75, PI * 0.75);
    axes[3][1].initializePositionLimits(12, -PI * 0.75, PI * 0.75);
    axes[3][2].initializePositionLimits(13, -PI * 0.75, PI * 0.75);
    axes[4][0].initializePositionLimits(14, -PI * 0.75, PI * 0.75);
    axes[4][1].initializePositionLimits(15, -PI * 0.75, PI * 0.75);
    axes[4][2].initializePositionLimits(18, -PI * 0.75, PI * 0.75);
    axes[5][0].initializePositionLimits(19, -PI * 0.75, PI * 0.75);
    axes[5][1].initializePositionLimits(22, -PI * 0.75, PI * 0.75);
    axes[5][2].initializePositionLimits(23, -PI * 0.75, PI * 0.75);

  Serial.begin(115200);

  for (uint8_t i = 0; i < 6; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      axes[i][j].setMapping(1, 0.75, false);
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
    axes[leg][motor].moveToPos(pos);
  }

}
