#include "leg.hpp"
#include "axis.hpp"
#include "hexapod.hpp"
#include "position.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "three_by_matrices.hpp"
#include <Arduino.h>
#include "voltage_monitor.hpp"
#include "serial_handler.hpp"

Hexapod::Hexapod() 
//: serial(*this, _current_pos.X, _current_pos.Y, _current_pos.Z, _current_pos.roll, _current_pos.pitch, _current_pos.yaw, _current_speed),
//voltageSensor(&serial)
{ 
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        legs[i].initializeAxes(i);
		_leg_queues[i].setLeg(&legs[i]);
    }

	serial = SerialHandler();
	voltageSensor = VoltageSensor(&serial);

}

void Hexapod::startUp() {

	delay(5000);
	sit();

}

void Hexapod::moveLegAxisToPos(uint8_t leg_number, uint8_t axis_number, double target_position) {
	legs[leg_number].axes[axis_number].moveToPos(target_position);
}

_Bool Hexapod::moveLegToPos(uint8_t leg_number, double x, double y, double z) {
	return legs[leg_number].rapidMove(x, y, z);
}

// _Bool Hexapod::moveLegToPos(uint8_t leg_number, double x, double y, double z) {
// 	return legs[leg_number].linearMoveSetup(x, y, z);
// }

void Hexapod::moveToZeros() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			legs[i].axes[j].moveToPos(0);
		}        
    }
} 

void Hexapod::sit() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		//slow speed to not slam into the ground
		legs[i].axes[0].moveToPosAtSpeed(0, 2);
		legs[i].axes[1].moveToPosAtSpeed(-2, 2);
		legs[i].axes[2].moveToPosAtSpeed(3, 2);
	}
}
            
void Hexapod::stand() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		legs[i].axes[0].moveToPos(0);
		legs[i].axes[1].moveToPos(1);
		legs[i].axes[2].moveToPos(0.75);
	}
}

void Hexapod::rapidMove(double x, double y, double z, double roll, double pitch, double yaw) {
	Position pos;
	pos.set(x, y, z, roll, pitch, yaw);
	rapidMove(pos);
	_current_pos.setPos(pos);
}

void Hexapod::rapidMove(Position next_pos) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	}
	rapidMove(next_pos, active_legs);
}

void Hexapod::rapidMove(Position next_pos, _Bool active_legs[NUM_LEGS]) {
	// Updated to use new _inverseKinematics caller, but not tested since this
	// function is not currently used
	ThreeByOne next_positions[NUM_LEGS];
	_inverseKinematics(next_pos, active_legs, next_positions);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			_next_leg_pos[i][j] = next_positions[i].values[j];
		}
	}
	_moveLegs();
	_current_pos.setPos(next_pos);
}

uint8_t Hexapod::linearMoveSetup(double x, double y, double z, double roll, double pitch, double yaw, double target_speed, _Bool active_legs[NUM_LEGS]) {
	_end_pos.set(x, y, z, roll, pitch, yaw);
	_start_pos.setPos(_current_pos);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		_linear_move_legs[i] = active_legs[i];
	}
	uint8_t retval = 0;
    double speed = target_speed;
    if (target_speed > _max_speed) {
        speed = _max_speed;
        retval = 1; // move speed capped
    }
	_move_progress = 0;
	_move_start_time = millis();
	Position pos_delta = _end_pos - _start_pos;
	_move_time = (fabs(pos_delta.magnitude()) / speed) * 1000; //convert to seconds
	_moving_flag = true;
	return retval;
}

uint8_t Hexapod::linearMoveSetup(Position next_pos, double target_speed, _Bool active_legs[NUM_LEGS]) {
	_end_pos.setPos(next_pos); 
	_start_pos.setPos(_current_pos);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		_linear_move_legs[i] = active_legs[i];
	}
	uint8_t retval = 0;
	double speed = target_speed;
	if (target_speed > _max_speed) {
		speed = _max_speed;
		retval = 1; // move speed capped
	}
	_move_progress = 0;
	_move_start_time = millis();
	Position pos_delta = _end_pos - _start_pos;
	_move_time = (fabs(pos_delta.magnitude()) / speed) * 1000; //convert to seconds
	_moving_flag = true;
	return retval;
}

uint8_t Hexapod::linearMoveSetup(Position next_pos, double target_speed) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	}
	return linearMoveSetup(next_pos, target_speed, active_legs);
}

uint8_t Hexapod::linearMoveSetup(double x, double y, double z, double roll, double pitch, double yaw, double target_speed) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	}
	return linearMoveSetup(x, y, z, roll, pitch, yaw, target_speed, active_legs);
}

uint8_t Hexapod::legLinearMoveSetup(uint8_t leg, double x,  double y, double z, double target_speed, _Bool relative) {
	ThreeByOne coord = ThreeByOne(x, y, z);
	coord.rotateYaw(_home_yaws[leg]);
	if (!relative) {
		coord += _stance_offset;
	}
	if (relative) {
		coord += ThreeByOne(_next_leg_pos[leg]);
	}
	_next_leg_pos[leg][0] = coord.values[0];
	_next_leg_pos[leg][1] = coord.values[1];
	_next_leg_pos[leg][2] = coord.values[2];
	
	return legs[leg].linearMoveSetup(coord.values[0], coord.values[1], coord.values[2], target_speed, relative);
}

uint8_t Hexapod::legLinearMoveSetup(uint8_t leg, ThreeByOne end_coord, double target_speed, _Bool relative) {
	return legLinearMoveSetup(leg, end_coord.values[0], end_coord.values[1], end_coord.values[2], target_speed, relative);
}

void Hexapod::opQueueTest() {
	ThreeByOne coord1 = ThreeByOne(0.0,   0.0,   100.0);
	ThreeByOne coord5 = ThreeByOne(0.0,   0.0,   100.0);
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		_leg_queues[i].enqueue(coord1, 100.0, false);
		_leg_queues[i].enqueue(ThreeByOne(0.0, 0.0, 0.0), 100.0, true, 1000);
		_leg_queues[i].enqueue(coord5, 100.0, true);
	}
}

void Hexapod::linearMovePerform() {
	if (isLowLevelBusy()) {
		double move_progress = (float)(millis() - _move_start_time) / ( _move_time);
		if (move_progress <= 1.0) {
			Position next_pos = (_end_pos - _start_pos) * move_progress + _start_pos;
			rapidMove(next_pos, _linear_move_legs);
			#if LOG_LEVEL >= CALCULATION_LOGGING
				String msg = "linear move legs: " + String(_linear_move_legs[0]) + " " + String(_linear_move_legs[1]) + " " + String(_linear_move_legs[2]) + " "
				+ String(_linear_move_legs[3]) + " " + String(_linear_move_legs[4]) + " " + String(_linear_move_legs[5]) + "\n";
				serial.logMsg(msg);
			#endif
		}
		for (uint8_t i = 0; i < NUM_LEGS; i++) {
			if (legs[i].isMoving()) {
				legs[i].linearMovePerform();
			}
		}
		if (move_progress > 1.0) {
			_moving_flag = false;
		}
	}
}

void Hexapod::_moveLegs() {
	#if LOG_LEVEL >= CALCULATION_LOGGING
		serial.logMsg("Next leg positions]");
	#endif
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (!legs[i].isMoving()) {
        	legs[i].rapidMove(_next_leg_pos[i][0], _next_leg_pos[i][1],_next_leg_pos[i][2]);
			#if LOG_LEVEL >= CALCULATION_LOGGING
				String msg = "leg: " + String(i) + " x:" + String(_next_leg_pos[i][0]) + " y:" + String(_next_leg_pos[i][1]) + " z:" + String(_next_leg_pos[i][2]) + "\n";
				serial.logMsg(msg);
			#endif
		}
	}
}

double Hexapod::get_max_step_magnitude() {
	return _current_step_permutation[_next_step_group % 2].magnitude() + MAX_STEP_MAGNITUDE;
}

uint8_t Hexapod::walkSetup(double x, double y, double speed, _Bool return_to_neutral) {
	ThreeByOne relative_end_coord = ThreeByOne(x, y, 0.0);
	return walkSetup(relative_end_coord, speed, return_to_neutral);
}

uint8_t Hexapod::walkSetup(Position relative_end_pos, double speed, _Bool return_to_neutral) {
	return walkSetup(relative_end_pos.coord(), speed, return_to_neutral);
}

uint8_t Hexapod::walkSetup(ThreeByOne relative_end_coord, double speed, _Bool return_to_neutral) {
	relative_end_coord.values[2] = 0.0;
	ThreeByOne end_unit_vector = relative_end_coord / relative_end_coord.magnitude();
	ThreeByOne distance_traveled = ThreeByOne();
	uint32_t counter = 0;
	while (distance_traveled < relative_end_coord) {
		ThreeByOne max_step_size = end_unit_vector * get_max_step_magnitude();
		ThreeByOne distance_to_go = relative_end_coord - distance_traveled;
		ThreeByOne this_step = (distance_to_go > max_step_size) ? ThreeByOne(max_step_size) : ThreeByOne(distance_to_go);
		#if LOG_LEVEL >= CALCULATION_LOGGING
			String msg = "Taking step: x: " + String(this_step.values[0]) + " y:" + String(this_step.values[1]) + " z:" + String(this_step.values[2]) + " speed:"
			+ String(speed) + "\n";
			serial.logMsg(msg);
		#endif
		stepSetup(this_step, speed);
		distance_traveled += this_step;
		counter++;
		if (return_to_neutral) {
			stepToNeutral(speed);
		}
	}
	return counter;
}

void Hexapod::wait(uint32_t time_ms) {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		legs[i].wait(time_ms);
	} 
}

uint8_t Hexapod::stepToNeutral(double speed) {
	uint8_t retval = 0;
	uint8_t num_segments = 3;
	double segment_z_offsets[num_segments] = {-30.0, 0.0, 30.0};
	ThreeByOne step_segment[num_segments];
	#if LOG_LEVEL >= CALCULATION_LOGGING
		String msg = "\nCurrent permutations:\nStep group 0:\n\tx:" + String(_current_step_permutation[0].values[0]) + " y:" + String(_current_step_permutation[0].values[1])
		+ " z:" + String(_current_step_permutation[0].values[2]) + "\nStep group 1:\n\tx:" String(_current_step_permutation[1].values[0]) + " y:" + String(_current_step_permutation[1].values[1])
		+ " z:" + String(_current_step_permutation[1].values[2]) + "\n";
		serial.logMsg(msg);
	#endif
	double step_path_length = 0.0;
	for (uint8_t i = 0; i < NUM_STEP_GROUPS; i++) {
		if (_current_step_permutation[i].magnitude() > 0.1) {
			for (uint8_t j = 0; j < num_segments; j++) {
				step_segment[j] = _current_step_permutation[i] / (-1.0 * double(num_segments));
				step_segment[j].values[2] += segment_z_offsets[j];
				step_path_length += step_segment[j].magnitude();
			}
			double speed_mult = 1; //step_path_length / _current_step_permutation[i].magnitude();
			uint32_t move_time = uint32_t(step_path_length / (speed_mult * speed) * 1000.0);
			for (uint8_t j = 0; j < NUM_LEGS / NUM_STEP_GROUPS; j++) {
				for (uint8_t k = 0; k < num_segments; k++) {
					//Apply to leg group i+1
					_leg_queues[_step_groups[i % 2][j]-1].enqueue(step_segment[k], speed * speed_mult, true);
				}
				_leg_queues[_step_groups[(i+1) % 2][j]-1].enqueue(ThreeByOne(0.0, 0.0, 0.0), 0.0, true, move_time);
			}
			_current_step_permutation[i] = ThreeByOne();
			retval += 1;
		}
	}
	_neutral_position_flag = true;
	return retval;
}

uint8_t Hexapod::stepSetup(double x, double y, double z, double speed) {
	ThreeByOne relative_end_coord = ThreeByOne(x, y, z);
	return stepSetup(relative_end_coord, speed);
}

uint8_t Hexapod::stepSetup(ThreeByOne relative_end_coord, double speed) {
	relative_end_coord.values[2] = 0.0;
	ThreeByOne step_unit_vector = relative_end_coord.unit_vector();

	if (_previous_step_unit_vector * -1.0 != step_unit_vector) {
		if (_previous_step_unit_vector != step_unit_vector) {
			stepToNeutral(speed);
		}
	}
	else {
		_next_step_group--;
	}

	_neutral_position_flag = false;
	double linear_path_length = relative_end_coord.magnitude();
	if (linear_path_length > get_max_step_magnitude()) {
		serial.errorMsg("Step size too big, try again with a smaller step");
		return 255; //Error code for too big step size
	}
	uint8_t num_step_segments = 5;
	double step_z_offsets[num_step_segments] = {-30.0, -20.0, 0.0, 20.0, 30.0};
	ThreeByOne step_segment[num_step_segments];
	double step_path_length = 0.0;
	for (uint8_t i = 0; i < num_step_segments; i++) {
		step_segment[i] = relative_end_coord / double(num_step_segments);
		step_segment[i].values[2] += step_z_offsets[i];
		step_path_length += step_segment[i].magnitude();
	}

	double speed_mult = step_path_length / linear_path_length;
	for (uint8_t i = 0; i < (NUM_LEGS / 2); i++) {
		_leg_queues[_step_groups[(_next_step_group + 1) % NUM_STEP_GROUPS][i]-1].enqueue(relative_end_coord * -1.0, speed, true);
		for (uint8_t j = 0; j < num_step_segments; j++) {
			_leg_queues[_step_groups[_next_step_group % NUM_STEP_GROUPS][i]-1].enqueue(step_segment[j], speed * speed_mult, true);
		} 

	}
	_current_step_permutation[(_next_step_group + 1) % NUM_STEP_GROUPS] += (relative_end_coord * -1.0);
	_current_step_permutation[_next_step_group % NUM_STEP_GROUPS] += relative_end_coord;
	_previous_step_unit_vector = ThreeByOne(step_unit_vector);

	_next_step_group++;
	return 0;
}

uint8_t Hexapod::legWaitSetup(uint8_t leg, uint32_t wait_time) {
	if (wait_time) {
		legs[leg].wait(wait_time);
		return 255;
	}
	return 0;
}

void Hexapod::returnToNeutral() {
	_return_to_neutral_flag = true;
}

uint16_t Hexapod::comboMovePerform() {
	// Return code is two 8 bit numbers
	// 8 grand bits are number of legs that got new end positions
	// 8 lesser bits are number of legs that are currently moving
	uint16_t retval = 0;
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (legs[i].isMoving()) {
			retval += 1;
		}
		else {
			if (!_leg_queues[i].isEmpty()) {
				Operation * queue_head = _leg_queues[i].head;
				legWaitSetup(i, queue_head->wait_time_ms);
				if (!queue_head->wait_time_ms) {
					legLinearMoveSetup(i, queue_head->end_pos, queue_head->speed, queue_head->relative);
					#if LOG_LEVEL >= CALCULATION_LOGGING
						String msg = "Leg " + String(i) + " moving to x:" + String(_leg_queues[i].head->end_pos.values[0]) + ", y: " + String(_leg_queues[i].head->end_pos.values[1]) + ", z: "
						+ String(_leg_queues[i].head->end_pos.values[2]) + "\nRelative: " + String(_leg_queues[i].head->relative) + " speed: " + String(_leg_queues[i].head->speed) + "\n";
						serial.logMsg(msg);
					#endif
				}
				_leg_queues[i].dequeue();
				retval += (1 << 8);
			}
		}
	}
	if (retval == 0) {
		if (!_neutral_position_flag) {
			if (_return_to_neutral_flag) {
				_return_to_neutral_flag = false;
				stepToNeutral(STEP_TO_NEUTRAL_SPEED);
			}
		}
	}
	linearMovePerform();
	return retval;
}

_Bool Hexapod::isBusy() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (legs[i].isMoving())
			return true;
	}
	return _moving_flag || _high_level_move_flag;
}

_Bool Hexapod::isLowLevelBusy() {
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (legs[i].isMoving())
			return true;
	}
	return _moving_flag;
}

uint8_t Hexapod::_inverseKinematics(double x, double y, double z, double roll, double pitch, double yaw, ThreeByOne *results) {
	Position pos;
	pos.set(x, y, z, roll, pitch, yaw);
	return _inverseKinematics(pos, results);
}

uint8_t Hexapod::_inverseKinematics(Position pos, ThreeByOne * results) {
	_Bool active_legs[NUM_LEGS];
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		active_legs[i] = true;
	} 
	return _inverseKinematics(pos, active_legs, results);
}

uint8_t Hexapod::_inverseKinematics(Position pos, _Bool active_legs[NUM_LEGS], ThreeByOne * results) {
	// Just to get something workign assume yaw = 0 
	// Must rework leg IK or set points align all coordinate systems
	// Right now we have 6 coordinate systems harder than we want to do
	// Even if we did like 2 coordinate systems (left / right) would be a big help for this
	// Discuss with Danny/Dillon best way to accomplish this as there are many confusing effects of either way 

	if (!_preCheckSafePos(pos))
        return 254; // Pre-check fail
	
	double potential_results[NUM_LEGS][3];
	// trig stuff is unique to each leg to can't use loop efficientlys
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		// if (i < 3) {
		// 	potential_results[i][0] = -1.0 * (pos.X * cos(pos.yaw) - pos.Y * sin(pos.yaw)); // + trig stuff with yaw
		// 	potential_results[i][1] = -1.0 * (pos.Y * cos(pos.yaw) + pos.X * sin(pos.yaw)); // + trig stuff with yaw
		// 	potential_results[i][2] = pos.Z + sin(pos.pitch) * _leg_X_offset[i] + sin(pos.roll) * _leg_Y_offset[i];
		// }
		// else {
			potential_results[i][0] = pos.X;
			potential_results[i][1] = pos.Y;
			potential_results[i][2] = pos.Z + sin(pos.pitch) * (_leg_X_offset[i] + pos.X) + sin(pos.roll) * (_leg_Y_offset[i] + pos.Y);
		// }
	}

	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		ThreeByOne temp = ThreeByOne(potential_results[i][0], potential_results[i][1], potential_results[i][2]);
		temp.rotateYaw(_home_yaws[i]);
		temp += _stance_offset;
		temp.rotateYaw(pos.yaw);
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			potential_results[i][j] = temp.values[j];
		}
	}

	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if (!_postCheckSafeCoords(potential_results[i][0], potential_results[i][1], potential_results[i][2]))
			return 255; // Post-check fail
	}
	uint8_t results_index = 0;
	for (uint8_t i = 0; i < NUM_LEGS; i++) {
		if(active_legs[i]) {
			results[results_index] = ThreeByOne(potential_results[i]);
			results_index++;
		}
		#if LOG_LEVEL >= CALCULATION_LOGGING
			String msg = "IK value; leg: " + String(i) + " x:" + String(results[i].values[0]) + " y:" + String(results[i].values[1]) + " z:" + String(results[i].values[2]) + "\n";
			serial.logMsg(msg);
		#endif
	}
	return 0;
}


_Bool Hexapod::_preCheckSafePos(Position pos) {
	// this function might be unnecessary but keeping as a placeholder
	//maybe dynamically set motor limits? check them here based on pos before moving. after movement complete calc new limits?
	return true;
}

_Bool Hexapod::_postCheckSafeCoords(double x, double y, double z) {
	// this function should check if legs will be in impossible positions or within the bot
	return true;
}

void Hexapod::forwardKinematics(double angle0, double angle1, double angle2) {
	ThreeByOne resulting_pos = legs[0].forwardKinematics(angle0, angle1, angle2);
	#if LOG_LEVEL >= CALCULATION_LOGGING
		String msg = "FK value; x: " + String(resulting_pos.values[0]) + " y:" + String(resulting_pos.values[1]) + " z:" + String(resulting_pos.values[2]) + "\n";
		serial.logMsg(msg);
	#endif
}

void Hexapod::runSpeed() {
	for (uint8_t i =0; i < NUM_LEGS; i++) {
		for (uint8_t j = 0; j < NUM_AXES_PER_LEG; j++) {
			legs[i].axes[j].runSpeed();
		}
	}
}

double Hexapod::getDistance(Position target_pos) {
  double distance = 0;
  Position current_pos = _current_pos;
  double dx = current_pos.X - target_pos.X;
  double dy = current_pos.Y - target_pos.Y;
  distance = sqrt(dx * dx + dy * dy); // + dz * dz);
  return distance;
}

void Hexapod::legEnqueue(uint8_t leg, ThreeByOne op_end_pos, 
						 double op_speed, _Bool op_relative, uint32_t op_wait_time_ms) {
	_leg_queues[leg].enqueue(op_end_pos, op_speed, op_relative);
}

void Hexapod::legEnqueue(uint8_t leg, ThreeByOne op_end_pos, 
						 uint32_t op_time, _Bool op_relative, uint32_t op_wait_time_ms) {
	double op_speed;
	if (op_end_pos.magnitude() > 0.001) {
		if (!op_relative) {
			ThreeByOne current_queue_end_pos = _leg_queues[leg].getCurrentQueueEndPos();
			ThreeByOne distance_to_go = current_queue_end_pos - op_end_pos;
			op_speed = distance_to_go.magnitude() / (double(op_time) / 1000.00);
		}
		else {
			op_speed = op_end_pos.magnitude() / (double(op_time) / 1000.00);
		}
	}
	else {
		op_speed = 100.00;
	}
	legEnqueue(leg, op_end_pos, op_speed, op_relative, op_wait_time_ms);
}
