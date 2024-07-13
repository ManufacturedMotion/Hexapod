#include "hexapod_controller.hpp"
#include "hexapod.hpp"
#include "command_queue.hpp"
#include <math.h>
#include <stdbool.h>
Hexapod hexapod;

_Bool commandQueueNeedsExpansion();

#if DEBUG
#define SERIAL_OUTPUT Serial
#else
#define SERIAL_OUTPUT Serial4
#endif

const int bufferSize = 64;
String buffer[bufferSize];
String split_command[bufferSize];
uint32_t num_words = 0;
double x = 0, y = 0, z = 200, roll = 0, pitch = 0, yaw = 0, speed = 100;
_Bool wait = false;
_Bool return_to_neutral = false;

Position position;
commandQueue command_queue;

void setup() {
  Serial.begin(115200);
  Serial4.begin(115200);
}

void loop() {

  String command = "";
  String current_command = "";
  _Bool need_optimize = false;
  static _Bool expand_queue_flag = false;
  static _Bool started_idle_timer = false;
  
  if (Serial.available() > 0 || Serial4.available() > 0) {
    if (Serial4.available() > 0) {
      command = Serial4.readStringUntil('\n');
    } else {
      command = Serial.readStringUntil('\n');
    }
    SERIAL_OUTPUT.print("Teensy Received: " + command + ".\n");
    command_queue.enqueue(command);
  }

  if (!command_queue.isEmpty()) {

    //TODO - need to find fix for hexapod.isBusy() - seems to get stuck at 1
  
    if (hexapod.isBusy()) {

    } else {
      //set wait to default of false & get the command from the command_queue after checking for optimizations
      wait = false;

      //if we are letting the command_queue grow we do not want to execute any commands
      if (!command_queue.isIdleTimer()){
        //if the first command is a step we should try to optimize the command queue
        if (getCommandType(command_queue.readIndex(0)).equals("step")){
          need_optimize = true;
        }
        else {
          need_optimize = false;
        }
        if (!need_optimize) {
          SERIAL_OUTPUT.print("No optimization needed, executing command as is\n");
          current_command = command_queue.dequeue();
          executeCommand(current_command);
        }
        else {         
          //if the command queue could not optimize we check to see if we tried letting the command queue grow 
          expand_queue_flag = commandQueueNeedsExpansion();
          //if we havent let the command_queue grow give it some time
          if (expand_queue_flag and !started_idle_timer){
            SERIAL_OUTPUT.print("command_queue is too short / could not make full step. letting the command_queue grow\n");
            command_queue.resetIdleTimer();
            started_idle_timer = true;
          }
          //if we let the command_queue grow and the combined step is still too small we have to execute it anyways, afterwards we should return to idle to make sure the hexapod is in a stable position
          else if (expand_queue_flag and started_idle_timer){ 
            SERIAL_OUTPUT.print("command_queue growing complete - still cannot make a full step. Getting partial command\n");
            current_command = getOptimizedCommand();
            SERIAL_OUTPUT.print("partial step formed: " + current_command + ".\n");
            return_to_neutral = true;
            executeCommand(current_command);
            started_idle_timer = false; 
          }
          //otherwise we have a set of commands in the command_queue that now combine to meet our step threshold
          else {
            SERIAL_OUTPUT.print("command_queue growing complete - full command could be made\n");
            current_command = getOptimizedCommand();
            SERIAL_OUTPUT.print("full step formed: " + current_command + ".\n");
            return_to_neutral = false;
            executeCommand(current_command);
            started_idle_timer = false;
          }
        }
      }
    } 
  }
  //call move every iteration of loop
  hexapod.comboMovePerform();
}

//function to split string on specified substring
void splitString(String command, String sub_string, String split_words[], uint32_t& num_words) {
  uint32_t start_index = 0;
  int16_t substring_index = 0;
  uint32_t word_index = 0;

  // Initialize split_words array with empty strings
  for (uint32_t i = 0; i < bufferSize; i++) {
    split_words[i] = "";
  }

  command.toLowerCase();
  sub_string.toLowerCase();

  while (true) {
    substring_index = command.indexOf(sub_string, start_index);
    if (substring_index == -1) {
      split_words[word_index] = command.substring(start_index);
      break;
    }
    split_words[word_index] = command.substring(start_index, substring_index);
    start_index = substring_index + sub_string.length();  // Move the start index past the substring
    word_index++;
  }

  num_words = word_index + 1;
}

//update x y z roll pitch yaw, etc
void updateVariables(String current_command_substring) {

  if (current_command_substring.startsWith('x')) {
    splitString(current_command_substring, 'X', buffer, num_words);
    String x_str = buffer[1];
    x = x_str.toFloat();
  } else if (current_command_substring.startsWith('y')) {
    splitString(current_command_substring, 'Y', buffer, num_words);
    String y_str = buffer[1];
    y = y_str.toFloat();
  } else if (current_command_substring.startsWith('z')) {
    splitString(current_command_substring, 'Z', buffer, num_words);
    String z_str = buffer[1];
    z = z_str.toFloat();
  } else if (current_command_substring.startsWith('r')) {
    splitString(current_command_substring, 'R', buffer, num_words);
    String roll_str = buffer[1];
    roll = roll_str.toFloat();
  } else if (current_command_substring.startsWith('p')) {
    splitString(current_command_substring, 'P', buffer, num_words);
    String pitch_str = buffer[1];
    pitch = pitch_str.toFloat();
  } else if (current_command_substring.startsWith('w')) {
    splitString(current_command_substring, 'W', buffer, num_words);
    String yaw_str = buffer[1];
    yaw = yaw_str.toFloat();
  } else if (current_command_substring.startsWith('s')) {
    splitString(current_command_substring, 'S', buffer, num_words);
    String speed_str = buffer[1];
    speed = speed_str.toFloat();
  } else if (current_command_substring.startsWith('h')) {
    splitString(current_command_substring, 'H', buffer, num_words);
    wait = buffer[1].toInt();
  }
}

//execute a command string
void executeCommand(String command) {

  //split on spaces
  splitString(command, ' ', split_command, num_words);
  uint32_t cmd_line_word_count = 0;
  cmd_line_word_count = num_words;

  //G-code commands
  if (split_command[0].startsWith('g')) {
    splitString(split_command[0], 'G', buffer, num_words);
    if (!buffer[1].equals("0") and !buffer[1].equals("1") and !buffer[1].equals("9")) {
      SERIAL_OUTPUT.printf("Error: only G0 and G1 implemented.\n");
    } else {
      String current_command_substring;
      if (buffer[1].equals("0")) {
        for (uint8_t i = 0; i < cmd_line_word_count; i++) {
          current_command_substring = split_command[i];
          updateVariables(current_command_substring);
          position.set(x, y, z, roll, pitch, yaw);
        }
        SERIAL_OUTPUT.printf("rapid move parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", x, y, z, roll, pitch, yaw, speed);
        //hexapod.rapidMove(position);
      }
      else if (buffer[1].equals("1")) {
        for (uint8_t i = 0; i < cmd_line_word_count; i++) {
          current_command_substring = split_command[i];
          updateVariables(current_command_substring);
          position.set(x, y, z, roll, pitch, yaw);
        }
        SERIAL_OUTPUT.printf("walk setup parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", x, y, z, roll, pitch, yaw, speed);
        hexapod.walkSetup(position, speed, return_to_neutral);
      }
      else if (buffer[1].equals("9")) {
        for (uint8_t i = 0; i < cmd_line_word_count; i++) {
          current_command_substring = split_command[i];
          updateVariables(current_command_substring);
          position.set(x, y, z, roll, pitch, yaw);                                                                                                                  }                                                                                                                                                           SERIAL_OUTPUT.printf("linear move parsing success; x, y, z is %f, %f, %f\n roll, pitch, yaw, speed are %f, %f, %f, %f.\n", x, y, z, roll, pitch, yaw, speed);
        hexapod.linearMoveSetup(position, speed);
      }
    }
  }
  else if (split_command[0].startsWith('p')) {
    splitString(split_command[0], 'P', buffer, num_words);
    if (buffer[1].equals("0")) {
      SERIAL_OUTPUT.printf("parsing success; starfish preset selected (move all motors to zero).\n");
      hexapod.moveToZeros();
      return;
    }
    else {
      SERIAL_OUTPUT.printf("parser detected input for a preset that is not yet supported.\n");
    }
  }
  else {
    SERIAL_OUTPUT.printf("Unsupported input recieved.\n");
  }
}


//check if we can make a full step with the commands at the top of the command_queue. If we do not change command types and can not make a full step we need to try letting the command queue grow
_Bool commandQueueNeedsExpansion() {
  Position position = getPosFromCommand(command_queue.readIndex(0));
  double step_size = hexapod.getDistance(position);
  for (uint32_t i = 1; i < command_queue.length; i++){
      //if we hit a command that is not a step we stop the optimization. We do not need to wait for the queue to expand
      if ((step_size < STEP_THRESHOLD) and (!getCommandType(command_queue.readIndex(i)).equals("step"))){
        return false;
      }
      Position next_position = getPosFromCommand(command_queue.readIndex(i));
      position = position + next_position;
      step_size = hexapod.getDistance(position);
  } 
  //if all of the commands in the command_queue were steps but we are still below the threshold we need to let the queue expand
  if (step_size < STEP_THRESHOLD) {
    return true;
  }
  //otherwise we would be able to make a full step; we do not need to idle
  return false;
}

//combine command_queue commands to produce either a full step or as large of a step as we can
String getOptimizedCommand() {
  String command = command_queue.dequeue();
  //pop the first command. The next command is now at index 0, we need to read this before entering the loop
  String next_command = command_queue.readIndex(0);
  //save command queue length external to loop -- we cannot do this in the loop because the popping will change the loop size 
  uint32_t command_queue_length = command_queue.length;
  for (uint32_t i = 0; i < command_queue_length; i++){
    //if we see the next command is not a step, return what we currently optimized to
    if (!getCommandType(next_command).equals("step")) {
      return command;
    }
    //otherwise we can add the steps and see if we have made a full step. If so we can return the optimized command early
    command = combineSteps(command, next_command);
    next_command = command_queue.dequeue();
    double step_size = hexapod.getDistance(getPosFromCommand(command));
    if (step_size > STEP_THRESHOLD) {
      return command;
    }
  }
  //if we get here that means we combined all fifo steps but could not meet the step threshold. This command will be executed and the hexapod will return to an idle resting position
  return command; 
}

String combineSteps(String step_1, String step_2) {
  if (step_2.equals(""))
    return step_1;
  Position new_pos = getPosFromCommand(step_1) + getPosFromCommand(step_2);
  float speed_1 = step_1.substring(step_1.indexOf('S') + 1).toFloat();
  float speed_2 = step_2.substring(step_2.indexOf('S') + 2).toFloat();
  double new_speed = 0;
  if (speed_2 == 0) {
    new_speed = speed_1;
  }
  else {
    new_speed = min(speed_1, speed_2);
  }
  bool new_wait = 1;
  String new_step = "G1 X" + String(new_pos.X) + " Y" + String(new_pos.Y) + " Z" + String(new_pos.Z) + " R" + String(new_pos.roll) + " P" + String(new_pos.pitch) + " W" + String(new_pos.yaw) + " S" + String(new_speed) + " H" + String(new_wait);
  return new_step;
}

//check the first index of the command and return a string summarizing the type
String getCommandType(String command) {
  String ret_val = "unknown";
  splitString(command, ' ', split_command, num_words);
  if (split_command[0].toLowerCase().equals("g1")) {
   ret_val = "step"; 
  }
  else if (split_command[0].toLowerCase().startsWith('p')) {
   ret_val = "preset";
  }
  return ret_val;
}

