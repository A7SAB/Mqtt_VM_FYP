#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "config.h"

#ifdef ENABLE_MOTOR_PINS

#include <Arduino.h>


enum RobotState {
    ROBOT_IDLE,
    ROBOT_MOVING,  // PID Controlled linear move
    ROBOT_TURNING, // PID Controlled turn
    ROBOT_MANUAL   // Direct speed control
};

class RobotController {
public:
    RobotController();

    // Initialization
    void begin();
    
    // Main Loop - Must be called frequently (e.g. in a separate task or main loop)
    void update(); 

    // VM Commands
    void move_cm(float distance_cm);
    void turn_degrees(float degrees);
    void set_speed(int motor_index, int speed); // 0=Left, 1=Right
    void stop();
    
    // State Check (for VM stalling)
    bool is_busy();
    void reset_state();

private:
    // PID Internals
    void run_pid_loop();
    void set_motor_pwm(int motor_index, int speed);
    
    // State
    RobotState state;
    unsigned long last_pid_time;
    
    // Geometry Constants 
    float ticks_per_cm;
    float ticks_per_degree;

    // Thresholds
    int move_error_threshold;
    int turn_error_threshold;
};

#endif

#endif



