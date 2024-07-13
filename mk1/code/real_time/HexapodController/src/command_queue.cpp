#include <stdbool.h>
#include <Arduino.h>
#include "command_queue.hpp"
#include "config.hpp"

void commandQueue::enqueue(String str_command) {
	Command* new_command = new Command(str_command);
	if (head == NULL) {
        head = new_command;
		head->next = NULL;
    }
    else {
        tail->next = new_command;
    }
    tail = new_command;
	tail->next = NULL;
	length++;
  _last_enqueue_timestamp = millis();
}

String commandQueue::dequeue() {
	if (head != NULL) {
        String ret_string = head->command;
        Command* temp = head;
		head = temp->next;
		delete temp;
		length--;
        return ret_string;
    }
    else {
        return String("");
    }
}

String commandQueue::readIndex(int index = 0) {
  
  if (head == NULL) {
    return String("");
  }

  Command* current_command = head;
  int count = 0;

  while (current_command != NULL && count < index) {
    current_command = current_command->next;
    count++;
  }
  
  if (current_command == NULL) {
    return String("");
  }

  return current_command->command;

}

Command::Command(String str_command) {
    command = str_command;
}

_Bool commandQueue::isEmpty(){
	return head == NULL;
}

commandQueue::commandQueue() {

	head = NULL;
	tail = NULL;
  _last_enqueue_timestamp = 0;

}

_Bool commandQueue::isIdle() {

  if ((millis() - _last_enqueue_timestamp) > FIFO_IDLE_THRESHOLD) { 
    return true;
  } else {
    return false;
  }

}

_Bool commandQueue::isIdleTimer() {

  if ((millis() - _expand_timer_start) > FIFO_IDLE_THRESHOLD){
    return false;
  }
  else {
    return true;
  }

}

void commandQueue::resetIdleTimer() {

  _expand_timer_start = millis();

}
