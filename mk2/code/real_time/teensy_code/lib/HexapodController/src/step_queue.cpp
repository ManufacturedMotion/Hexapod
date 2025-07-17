#include "step_queue.hpp"

StepQueue::StepQueue() {
    head = NULL;
    tail = NULL;
    _end_pos.clear();
    length = 0;
}

uint32_t StepQueue::enqueue(Position op_end_pos, double op_speed, StepType op_step_type) { // steps are ALWAYS relative, linear moves can be either relative or absolute, rapid moves are always absolute
    #ifdef STEP_QUEUE_DEBUG
        Serial.printf("Enqueueing step with end pos: %f, %f, %f, %f, %f, %f and of type %d\n", op_end_pos.x, op_end_pos.y, op_end_pos.z, op_end_pos.roll, op_end_pos.pitch, op_end_pos.yaw, op_step_type);
    #endif

    if (state == StepQueueState::UNINITIALIZED) {
        if (op_step_type != StepType::RAPID_MOVE) {
            return 0;
        }
    }

    double op_time;
    switch(op_step_type) {
        case StepType::GROUP0:
        case StepType::GROUP1:
            if (!(state == StepQueueState::NEUTRAL || last_step_type == op_step_type)) {
                // If switching groups from a non neutral position, then flip X, Y, and Yaw
                _end_pos.x *= -1.0;
                _end_pos.y *= -1.0;
                _end_pos.yaw *= -1.0;
            }
            _end_pos += op_end_pos;
            op_time = double((op_end_pos.magnitude() / op_speed) * 1000.0);
            state = StepQueueState::STEPPED;
        break;
        case StepType::LINEAR_MOVE_RELATIVE:
            _end_pos += op_end_pos;
            op_time = double((op_end_pos.magnitude() / op_speed) * 1000.0);
        break;
        case StepType::LINEAR_MOVE_ABSOLUTE:
            op_time = double(((op_end_pos - getCurrentQueueEndPos()).magnitude() / op_speed) * 1000.0);
            _end_pos.setPos(op_end_pos);
            if (op_time < 0.0001) {
                // If the move is too small, then don't enqueue it
                return 0;
            } 
        break;
        case StepType::RETURN_TO_NEUTRAL:
            if (state == StepQueueState::NEUTRAL) {
                // If already in neutral, no need to enqueue a step
                return 0;
            }
            op_time = double(((op_end_pos - getCurrentQueueEndPos()).magnitude() / op_speed) * 1000.0 * 2.0);
            _end_pos.setPos(op_end_pos);
            state = StepQueueState::NEUTRAL;
        break;
        case StepType::RAPID_MOVE:
            op_time = double(100.0); // 100ms for rapid move
            _end_pos.setPos(op_end_pos);
            state = StepQueueState::NEUTRAL;
        break;
        default:
            op_time = 0.0;
            break;
    }
    #ifdef STEP_QUEUE_DEBUG
        Serial.printf("Which puts hexapod at an end pos of %f, %f, %f, %f, %f, %f\n", _end_pos.x, _end_pos.y, _end_pos.z, _end_pos.roll, _end_pos.pitch, _end_pos.yaw);
    #endif

	Step* new_operation = new Step(op_end_pos, op_speed, op_step_type, op_time);
	if (head == NULL) {
        head = new_operation;
		head->next = NULL;
    }
    else {
        tail->next = new_operation;
    }
    tail = new_operation;
	tail->next = NULL;
	length++;
    last_step_type = op_step_type;

    return static_cast<uint32_t>(op_time);
}

_Bool StepQueue::dequeue() {
	if (head != NULL) {
        Step* temp = head;
		head = temp->next;
		delete temp;
		length--;
        return true;
    }
    return false;
}

Step::Step(Position op_end_pos, double op_speed, StepType op_step_type, double op_time) {
    end_pos.setPos(op_end_pos);
    speed = op_speed;
    step_type = op_step_type;
    time = op_time;
}

Step::Step() {
    end_pos.set(0.00, 0.00, 0.00, 0.00, 0.00, 0.00);
    speed = DEFAULT_MOVE_SPEED;
}

_Bool StepQueue::isEmpty() {
	return head == NULL;
}

Position StepQueue::getCurrentQueueEndPos() {
    return _end_pos;
}

