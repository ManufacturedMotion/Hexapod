#include "HexapodController.h"

User user = Dillon;
int pwm_pins[6][3] = PWM_PINS;
double min_pos[6][3] = MIN_POS; double max_pos[6][3] = MAX_POS; double scale_fact[6][3] = SCALE_FACT;

Axis axes[6][3] = {{Axis(pwm_pins[0][0], min_pos[0][0] * scale_fact[0][0], max_pos[0][0] * scale_fact[0][0]), Axis(pwm_pins[0][1], min_pos[0][1] * scale_fact[0][1], max_pos[0][1] * scale_fact[0][1]), Axis(pwm_pins[0][2], min_pos[0][2] * scale_fact[0][2], max_pos[0][2] * scale_fact[0][2])}, 
{Axis(pwm_pins[1][0], min_pos[1][0] * scale_fact[1][0], max_pos[1][0] * scale_fact[1][0]), Axis(pwm_pins[1][1], min_pos[1][1] * scale_fact[1][1], max_pos[1][1] * scale_fact[1][1]), Axis(pwm_pins[1][2], min_pos[1][2] * scale_fact[1][2], max_pos[1][2] * scale_fact[1][2])}, 
{Axis(pwm_pins[2][0], min_pos[2][0] * scale_fact[2][0], max_pos[2][0] * scale_fact[2][0]), Axis(pwm_pins[2][1], min_pos[2][1] * scale_fact[2][1], max_pos[2][1] * scale_fact[2][1]), Axis(pwm_pins[2][2], min_pos[2][2] * scale_fact[2][2], max_pos[2][2] * scale_fact[2][2])}, 
{Axis(pwm_pins[3][0], min_pos[3][0] * scale_fact[3][0], max_pos[3][0] * scale_fact[3][0]), Axis(pwm_pins[3][1], min_pos[3][1] * scale_fact[3][1], max_pos[3][1] * scale_fact[3][1]), Axis(pwm_pins[3][2], min_pos[3][2] * scale_fact[3][2], max_pos[3][2] * scale_fact[3][2])}, 
{Axis(pwm_pins[4][0], min_pos[4][0] * scale_fact[4][0], max_pos[4][0] * scale_fact[4][0]), Axis(pwm_pins[4][1], min_pos[4][1] * scale_fact[4][1], max_pos[4][1] * scale_fact[4][1]), Axis(pwm_pins[4][2], min_pos[4][2] * scale_fact[4][2], max_pos[4][2] * scale_fact[4][2])},
{Axis(pwm_pins[5][0], min_pos[5][0] * scale_fact[5][0], max_pos[5][0] * scale_fact[5][0]), Axis(pwm_pins[5][1], min_pos[5][1] * scale_fact[5][1], max_pos[5][1] * scale_fact[5][1]), Axis(pwm_pins[5][2], min_pos[5][2] * scale_fact[5][2], max_pos[5][2] * scale_fact[5][2])}};

void setup() {

  Serial.begin(115200);
  setMapping(user, axes);

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
    axes[leg - 1][motor -1].move_to_pos(pos);
  }

}
