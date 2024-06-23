#include "hexapod_controller.hpp"
#include <math.h>
#include <inttypes.h>
Hexapod hexapod; 

void setup() {

  Serial.begin(115200);

}

void loop() {
  short unsigned int _, leg, motor;
  double pos;
  double x, y, z, roll, pitch, yaw, speed;

  	if (Serial.available() > 0) {
    	String command = Serial.readStringUntil('\n');
    	Serial.println("Received\n");
      uint8_t command_type = command.toInt();
		//temp switch for user interaction
		switch(command_type) {
			default:
			case 0:
				break;
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
        sscanf(command.c_str(), "%hu %hu %lf", &leg, &motor, &pos);
				Serial.printf("leg: %d; motor: %d; pos: %f\n", leg, motor, pos);
				hexapod.moveLegAxisToPos((uint8_t) leg, (uint8_t) motor, pos);
				break;
			case 7:
				Serial.printf("Instructed to set to zeros");
        hexapod.moveToZeros();
				break;
			case 8:
				hexapod.stand();
				break;
      case 9:
        sscanf(command.c_str(), "%hu %hu %lf %lf %lf", &_, &leg, &x, &y, &z);
        Serial.printf("leg: %d; x: %f; y: %f, z: %f\n", leg, x, y, z);
        Serial.println(hexapod.moveLegToPos((uint8_t)leg, x, y, z));
        break;
      case 10:
        sscanf(command.c_str(), "%hu %lf %lf %lf", &_, &x, &y, &z);
        Serial.printf("angle0: %f; angle1: %f, angle2: %f\n", x, y, z);
        hexapod.forwardKinematics(x, y, z);
        break;
      case 11:
        sscanf(command.c_str(), "%hu %lf %lf %lf %lf %lf %lf", &_, &x, &y, &z, &roll, &pitch, &yaw);
        hexapod.rapidMove(x, y, z, roll, pitch, yaw);
        break;
      case 12:
        sscanf(command.c_str(), "%hu %lf %lf %lf %lf %lf %lf %lf", &_, &x, &y, &z, &roll, &pitch, &yaw, &speed);
        hexapod.linearMoveSetup(x, y, z, roll, pitch, yaw, speed);
        break;
      case 13:
        sscanf(command.c_str(), "%hu %hu %lf %lf %lf %lf", &_, &leg, &x, &y, &z, &speed);
  		  hexapod.legLinearMoveSetup(leg, x, y, z, speed, true);
  		  break;
      case 14:
        sscanf(command.c_str(), "%hu %lf %lf %lf %lf %lf %lf %lf", &_, &x, &y, &z, &roll, &pitch, &yaw, &speed);
        hexapod.legLinearMoveSetup(2, x, y, z, speed);
        hexapod.legLinearMoveSetup(4, x, y, z, speed);
        hexapod.legLinearMoveSetup(6, x, y, z, speed);
        
        hexapod.legLinearMoveSetup(1, x, y + 50, z - 50, speed*2.0);
        hexapod.legLinearMoveSetup(3, x, y + 50, z - 50, speed*2.0);
        hexapod.legLinearMoveSetup(5, x, y + 50, z - 50, speed*2.0);
  		  break;
  		case 15:
        sscanf(command.c_str(), "%hu %lf %lf %lf", &_, &x, &y, &speed);
        hexapod.stepSetup(x, y, 0.0, speed);
        break;
  		case 16:
        hexapod.opQueueTest();
        break;
      case 17:
        sscanf(command.c_str(), "%hu %hu %lf %lf %lf %lf", &_, &leg, &x, &y, &z, &speed);
        hexapod.legLinearMoveSetup(leg, x, y, z, speed, false);
        break;
  		}
	}
  hexapod.comboMovePerform();
  hexapod.linearMovePerform();
  
}