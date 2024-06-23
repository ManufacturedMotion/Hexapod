#include <stdbool.h>
#include "operation_queue.hpp"

void OperationQueue::enqueue(ThreeByOne op_end_pos, double op_speed, _Bool op_relative, uint32_t op_wait_time_ms) {
	Operation* new_operation = new Operation(op_end_pos, op_speed, op_relative, op_wait_time_ms);
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
}

void OperationQueue::dequeue() {
	if (head != NULL) {
        Operation* temp = head;
		head = temp->next;
		delete temp;
		length--;
    }
}

Operation::Operation(ThreeByOne op_end_pos, double op_speed, _Bool op_relative, uint32_t op_wait_time_ms) {
    end_pos = ThreeByOne(op_end_pos);
    speed = op_speed;
    wait_time_ms = op_wait_time_ms;
    relative = op_relative;
}

_Bool OperationQueue::isEmpty(){
	return head == NULL;
}

