
#ifndef STEP_QUEUE
#define STEP_QUEUE

#include "position.hpp"
#define DEFAULT_MOVE_SPEED 100

enum class StepType {
    GROUP0 = 0,
    GROUP1 = 1,
    LINEAR_MOVE = 255,
    RETURN_TO_NEUTRAL = 254,
    RAPID_MOVE = 253,
};

enum class StepQueueState {
    NEUTRAL = 0,
    STEPPED = 1,
};

class Step {
    // Relative motion of COM to be completed at one speed and in a straight line
    public:
        Step();
        Step(Position op_end_pos, double op_speed, StepType op_step_type, double op_time);
        Position end_pos;
        double speed;
        StepType step_type; // 0 (step group 0), 1 (step group 1), or 255 (all legs) 
        Step * next;
        double time;
    private:
};

class StepQueue {
    public: 
        StepQueue();
        StepQueueState state = StepQueueState::NEUTRAL;
		Step * head = NULL;
        Step * tail = NULL;
        uint32_t enqueue(Position op_end_pos, double op_speed, StepType op_step_type);
        StepType last_step_type;
        _Bool dequeue();
		_Bool isEmpty();
        Position getCurrentQueueEndPos();
		uint32_t length = 0;
    private:	
        Position _end_pos;
};

#endif