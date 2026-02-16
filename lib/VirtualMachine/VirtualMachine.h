#ifndef VIRTUAL_MACHINE_H
#define VIRTUAL_MACHINE_H

// LIBRARIES
#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "VmEvents.h"

#ifdef ENABLE_MOTOR_PINS
    #include "RobotController.h"
#endif

// --- VM PARAMETERS --- 
#define VARIABLES_SIZE 128 
#define STRING_POOL_SIZE 1024
#define MAX_STRINGS 128

// --- VM INSTRUCTION OPCODES ---
enum Opcode {
    // Flow Control & Core
    OP_NOP = 0,
    OP_SETUP_START = 1,
    OP_LOOP_START = 2,
    OP_HALT = 3,

    // Stack & Memory
    OP_PUSH = 4,
    OP_POP = 5,
    OP_LOAD = 6,
    OP_STORE = 7,

    // I/O Operations
    OP_PIN_MODE = 8,
    OP_DIGITAL_WRITE = 9,
    OP_DIGITAL_READ = 10,
    OP_ANALOG_WRITE = 11,
    OP_ANALOG_READ = 12,
    OP_DELAY = 13,

    // Arithmetic Operations (Basics)
    OP_ADD = 14,
    OP_SUB = 15,
    OP_MUL = 16,
    OP_DIV = 17,

    // Logical Operations
    OP_AND = 18,
    OP_OR = 19,
    OP_NOT = 20,

    // Comparison Operations
    OP_EQ = 21,
    OP_NEQ = 22,
    OP_GT = 23,
    OP_LT = 24,
    OP_GTE = 25,
    OP_LTE = 26,

    // Jump & Function Control
    OP_JUMP = 27,
    OP_JUMP_IF_FALSE = 28,
    OP_CALL = 29,
    OP_RET = 30,

    // Console Output
    OP_PRINT_INT = 31,
    OP_PRINT_STRING = 32,

    // Added Arithmetic operation ()
    OP_MOD = 33,
    OP_MAP = 34,
    OP_POW = 35, 

    // Servo Instructions
    OP_SERVO_ATTACH = 36,
    OP_SERVO_WRITE = 37,
    OP_SERVO_READ = 38,

    // Ultrasonic Sensor
    OP_ULTRASONIC_READ_CM = 39,

    // OLED Display Opcodes
    OP_OLED_INIT = 40,
    OP_OLED_CLEAR = 41,
    OP_OLED_UPDATE = 42,
    OP_OLED_DRAW_PIXEL = 43,
    OP_OLED_DRAW_LINE = 44,
    OP_OLED_DRAW_RECT = 45,
    OP_OLED_FILL_RECT = 46,
    OP_OLED_SET_CURSOR = 47,
    OP_OLED_SET_TEXT_SIZE = 48,
    OP_OLED_SET_TEXT_COLOR = 49,
    OP_OLED_PRINT_TEXT = 50,
    OP_OLED_PRINT_NUMBER = 51,

    // NEOPIXEL OPCODES
    OP_NEOPIXEL_INIT = 52,           // Stack: [Pin, Count]
    OP_NEOPIXEL_SET_BRIGHTNESS = 53, // Stack: [0-255]
    OP_NEOPIXEL_SET_PIXEL = 54,      // Stack: [Color, Index] (Top)
    OP_NEOPIXEL_FILL = 55,           // Stack: [Color]
    OP_NEOPIXEL_CLEAR = 56,          // Clear buffer
    OP_NEOPIXEL_SHOW = 57,

    #ifndef ENABLE_MOTOR_PINS

    OPCODE_COUNT = 58

    #else

        // Motor Controls for robots only
        OP_MOVE_CM = 58,
        OP_TURN_DEG = 59,
        OP_MOTOR_SET_SPEED = 60,
        OP_ROBOT_STOP = 61,

        // Buzzer 
        OP_TONE = 62,    
        OP_NO_TONE = 63,
         
        OPCODE_COUNT = 64,
    
    #endif

    // Helper for bounds checking. Must always be the last entry.
};

struct Instruction {
    Opcode opcode;
    int32_t arg1;
};

struct CallFrame {
    int vars[VARIABLES_SIZE];
};

enum VmState { VM_STOPPED, VM_RUNNING, VM_HALTED, VM_ERROR };

// --- VM CLASS DEFINITION ---
class VirtualMachine {
public:
    VirtualMachine(QueueHandle_t log_queue_handle);

    void reset();
    void run_tick();
    bool load_program_binary(const uint8_t* payload, size_t length);
    
    void set_state(VmState new_state);
    VmState get_state();
    void reset_gpio_pins();

private:

    #ifdef ENABLE_MOTOR_PINS
        RobotController robot;                      //Robot motor controls
    #endif

    unsigned long delay_time;

    void execute_instruction();

    void send_print(const char* msg);           // User output
    void abort_with_error(const char* msg);     // User mistake (halts VM)
    void send_debug(const char* msg);           // System info

    // Stack Ops
    void push(int value);
    int pop();

    QueueHandle_t log_queue;

    // VM Memory & State 
    Instruction program[PROGRAM_SIZE];
    int stack[STACK_SIZE];
    CallFrame call_frames[CALL_STACK_SIZE];
    int return_stack[CALL_STACK_SIZE];
    char string_pool[STRING_POOL_SIZE];
    int string_addresses[MAX_STRINGS];

    int* variables_ptr; 
    int frame_ptr, pc, sp, rsp;
    int setup_start_addr, loop_start_addr, program_length;
    int string_pool_ptr, string_count;
    
    VmState vm_state;

    // Hardware Objects
    Servo servos[VM_MAX_SERVOS];
    int servo_pins[VM_MAX_SERVOS];

    Adafruit_SSD1306 display;
    bool display_initialized;

    Adafruit_NeoPixel* neostrip; 
    bool neo_initialized;
};

#endif // VIRTUAL_MACHINE_H