#include <stdbool.h>
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "three_by_matrices.hpp"

#ifndef OPERATION_QUEUE
#define OPERATION_QUEUE

class MovementOperation {
    public:
        double speed;
        ThreeByOne end_pos;
        _Bool relative;
};

class Wait {
    public:
    uint32_t wait_time_ms;
};

class Operation {
    public:
        Operation(ThreeByOne op_end_pos, double op_speed, _Bool op_relative, uint32_t op_wait_time_ms);
        ThreeByOne end_pos = ThreeByOne(0.0, 0.0, 0.0);
        uint32_t wait_time_ms;
        double speed;
        _Bool relative;
        Operation * next;
    private:
};

class OperationQueue {
    public:
		Operation * head = NULL;
        Operation * tail = NULL;
        void enqueue(ThreeByOne op_end_pos, double op_speed, _Bool op_relative, uint32_t op_wait_time_ms = 0);
        void dequeue();
		_Bool isEmpty();
		uint32_t length = 0;
    private:
		
};

#endif
