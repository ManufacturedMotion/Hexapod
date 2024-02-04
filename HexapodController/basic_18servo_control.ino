#include "HexapodController.h"

void move_to_pos_at_speed(double position_value, double speed_value);
Axis l1_m1(2, -3.14, 3.14);
Axis l1_m2(3, -3.14, 3.14);
Axis l1_m3(4, -3.14, 3.14);

Axis l2_m1(5, -3.14, 3.14);
Axis l2_m2(6, -3.14, 3.14);
Axis l2_m3(7, -3.14, 3.14);

Axis l3_m1(8, -3.14, 3.14);
Axis l3_m2(9, -3.14, 3.14);
Axis l3_m3(10, -3.14, 3.14);

Axis l4_m1(11, -3.14, 3.14);
Axis l4_m2(12, -3.14, 3.14);
Axis l4_m3(13, -3.14, 3.14);

Axis l5_m1(14, -3.14, 3.14);
Axis l5_m2(15, -3.14, 3.14);
Axis l5_m3(18, -3.14, 3.14);

Axis l6_m1(19, -3.14, 3.14);
Axis l6_m2(22, -3.14, 3.14);
Axis l6_m3(23, -3.14, 3.14);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  l1_m1.set_mapping(0,1);
  l1_m2.set_mapping(0,1);
  l1_m3.set_mapping(0,1);
  
  l2_m1.set_mapping(0,1);
  l2_m2.set_mapping(0,1);
  l2_m3.set_mapping(0,1);
  
  l3_m1.set_mapping(0,1);
  l3_m2.set_mapping(0,1);
  l3_m3.set_mapping(0,1);
  
  l4_m1.set_mapping(0,1);
  l4_m2.set_mapping(0,1);
  l4_m3.set_mapping(0,1);

  l5_m1.set_mapping(0,1);
  l5_m2.set_mapping(0,1);
  l5_m3.set_mapping(0,1);

  l6_m1.set_mapping(0,1);
  l6_m2.set_mapping(0,1);
  l6_m3.set_mapping(0,1);
}


String remove_to_space(String command) {
  while (command.charAt(0) != ' ' && command.charAt(0)) {
    command.remove(0,1);
  }
  command.remove(0,1);
  return command;
}

void loop() {
  double position_value;

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    position_value = command.toFloat();

    Serial.println(position_value);
    move_to_pos(position_value);
    Serial.println("lemon");
  }

}

// Function to update the Axis instance based on axis number, position value, and speed value
void move_to_pos(double position_value) {
  l1_m1.move_to_pos(position_value);
  l1_m2.move_to_pos(position_value);
  l1_m3.move_to_pos(position_value);
  
  l2_m1.move_to_pos(position_value);
  l2_m2.move_to_pos(position_value);
  l2_m3.move_to_pos(position_value);
  
  l3_m1.move_to_pos(position_value);
  l3_m2.move_to_pos(position_value);
  l3_m3.move_to_pos(position_value);
  
  l4_m1.move_to_pos(position_value);
  l4_m2.move_to_pos(position_value);
  l4_m3.move_to_pos(position_value);

  l5_m1.move_to_pos(position_value);
  l5_m2.move_to_pos(position_value);
  l5_m3.move_to_pos(position_value);

  l6_m1.move_to_pos(position_value);
  l6_m2.move_to_pos(position_value);
  l6_m3.move_to_pos(position_value);
}
