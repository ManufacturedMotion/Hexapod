#include <stdbool.h>
#include <Arduino.h>
#include <stdint.h>

#ifndef FIFO_COMMAND
#define FIFO_COMMAND

class Command {
    public:
      Command(String command);
      String command;
      Command * next;
    private:
};

class commandQueue {
    public:
      commandQueue();
		  Command * head;
      Command * tail;
      void enqueue(String str_command);
      String dequeue();
      String readIndex(int index);
		  _Bool isEmpty();
      _Bool isIdle();
		  uint32_t length = 0;
      _Bool isIdleTimer();
      void resetIdleTimer();
    private:
      uint64_t _last_enqueue_timestamp = 0;
	    uint64_t _expand_timer_start = 0;	
};

#endif
