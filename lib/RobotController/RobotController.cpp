#include "RobotController.h"

#ifdef ENABLE_MOTOR_PINS

// --- GLOBAL ISR GLUE ---
volatile long g_m1_ticks = 0;
volatile long g_m2_ticks = 0;

void IRAM_ATTR isr_m1_enc_a() {
    if (digitalRead(M1_ENC_B_PIN) == LOW){
        g_m1_ticks++;}
    else{ 
        g_m1_ticks--;}
}

void IRAM_ATTR isr_m2_enc_a() {
    if (digitalRead(M2_ENC_B_PIN) == LOW){
        g_m2_ticks++;}
    else{ 
        g_m2_ticks--;}
}


// Internal PID State variables
long m1_target_ticks = 0;
long m2_target_ticks = 0;

// Inner Loop (Velocity) State
volatile long m1_last_ticks_vel = 0;
volatile long m2_last_ticks_vel = 0;
double m1_vel_integral = 0, m2_vel_integral = 0;
double m1_vel_prev_meas = 0, m2_vel_prev_meas = 0;
float m1_current_rpm = 0, m2_current_rpm = 0;


RobotController::RobotController() : state(ROBOT_IDLE), last_pid_time(0) {}

void RobotController::begin() {
    pinMode(M1_IN1_PIN, OUTPUT); 
    pinMode(M1_IN2_PIN, OUTPUT); 
    pinMode(M1_PWM_PIN, OUTPUT);

    pinMode(M1_ENC_A_PIN, INPUT);
    pinMode(M1_ENC_B_PIN, INPUT);

    pinMode(M2_IN1_PIN, OUTPUT);
    pinMode(M2_IN2_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);

    pinMode(M2_ENC_A_PIN, INPUT);
    pinMode(M2_ENC_B_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), isr_m1_enc_a, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A_PIN), isr_m2_enc_a, RISING);

    float wheel_circ_cm = (WHEEL_DIAMETER_MM / 10.0) * 3.14159;
    ticks_per_cm = TICKS_PER_REV / wheel_circ_cm;

    float robot_turn_circ_cm = (WHEEL_TRACK_MM / 10.0) * 3.14159;
    ticks_per_degree = (robot_turn_circ_cm * ticks_per_cm) / 360.0;

    move_error_threshold = (int)(ticks_per_cm * 0.2); // 2mm tolerance
    turn_error_threshold = (int)(ticks_per_degree * 1.5); // 1.5 deg tolerance
}

void RobotController::set_motor_pwm(int motor_index, int speed) {
    int in1, in2, pwm_pin;
    if (motor_index == 0) { 
        in1 = M1_IN1_PIN;
        in2 = M1_IN2_PIN;
        pwm_pin = M1_PWM_PIN;
    }
    else{ 
        in1 = M2_IN1_PIN;
        in2 = M2_IN2_PIN;
        pwm_pin = M2_PWM_PIN;
    }

    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(pwm_pin, speed);
    } else if (speed < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(pwm_pin, abs(speed));
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwm_pin, 0);
    }
}

void RobotController::move_cm(float distance_cm) {
    reset_state(); 
    long ticks = (long)(distance_cm * ticks_per_cm);
    
    m1_target_ticks = g_m1_ticks + ticks;
    m2_target_ticks = g_m2_ticks + ticks; // Both move forward
    
    state = ROBOT_MOVING;
}

void RobotController::turn_degrees(float degrees) {
    reset_state();
    long ticks = (long)(degrees * ticks_per_degree);

    m1_target_ticks = g_m1_ticks + ticks;
    m2_target_ticks = g_m2_ticks - ticks; // Opposing directions
    
    state = ROBOT_TURNING;
}

void RobotController::set_speed(int motor_index, int speed) {
    state = ROBOT_MANUAL; 
    set_motor_pwm(motor_index, speed);
}

void RobotController::stop() {
    state = ROBOT_IDLE;
    set_motor_pwm(0, 0);
    set_motor_pwm(1, 0);
}

bool RobotController::is_busy() {
    return (state == ROBOT_MOVING || state == ROBOT_TURNING);
}

void RobotController::reset_state() {
    m1_vel_integral = 0; m2_vel_integral = 0;
    m1_vel_prev_meas = 0; m2_vel_prev_meas = 0;
    
    // Reset velocity calculation baseline
    m1_last_ticks_vel = g_m1_ticks;
    m2_last_ticks_vel = g_m2_ticks;
}

void RobotController::update() {
    unsigned long now = millis();
    if (now - last_pid_time < (PID_SAMPLE_TIME_S * 1000)) return;
    last_pid_time = now;

    if (state != ROBOT_MOVING && state != ROBOT_TURNING) return;

    run_pid_loop();
}

void RobotController::run_pid_loop() {
    // Velocity (RPM)
    long m1_curr = g_m1_ticks;
    long m2_curr = g_m2_ticks;

    long m1_delta = m1_curr - m1_last_ticks_vel;
    long m2_delta = m2_curr - m2_last_ticks_vel;
    
    m1_last_ticks_vel = m1_curr;
    m2_last_ticks_vel = m2_curr;

    m1_current_rpm = ((float)m1_delta / TICKS_PER_REV / PID_SAMPLE_TIME_S) * 60.0;
    m2_current_rpm = ((float)m2_delta / TICKS_PER_REV / PID_SAMPLE_TIME_S) * 60.0;

    // Position Loop (Outer)
    long m1_err = m1_target_ticks - m1_curr;
    long m2_err = m2_target_ticks - m2_curr;

    // Check completion
    int thresh = (state == ROBOT_MOVING) ? move_error_threshold : turn_error_threshold;
    if (abs(m1_err) <= thresh && abs(m2_err) <= thresh) {
        stop(); // Target reached
        return;
    }

    // P-Controller for Target Velocity
    float pos_P = PID_POS_P; // interpret as ticks_per_sec per tick-error
    float m1_target_ticks_per_sec = pos_P * (float)m1_err;
    float m2_target_ticks_per_sec = pos_P * (float)m2_err;

    float m1_target_rpm = (m1_target_ticks_per_sec / (float)TICKS_PER_REV) * 60.0f;
    float m2_target_rpm = (m2_target_ticks_per_sec / (float)TICKS_PER_REV) * 60.0f;

    // Clamp Max RPM
    float max_rpm = 150.0; 
    m1_target_rpm = constrain(m1_target_rpm, -max_rpm, max_rpm);
    m2_target_rpm = constrain(m2_target_rpm, -max_rpm, max_rpm);

    // Velocity Loop (Inner)
    float m1_vel_err = m1_target_rpm - m1_current_rpm;
    float m2_vel_err = m2_target_rpm - m2_current_rpm;

    m1_vel_integral += m1_vel_err * PID_SAMPLE_TIME_S;
    m2_vel_integral += m2_vel_err * PID_SAMPLE_TIME_S;

    // Anti-windup
    m1_vel_integral = constrain(m1_vel_integral, -200, 200);
    m2_vel_integral = constrain(m2_vel_integral, -200, 200);

    float m1_output = (PID_VEL_P * m1_vel_err) + (PID_VEL_I * m1_vel_integral);
    float m2_output = (PID_VEL_P * m2_vel_err) + (PID_VEL_I * m2_vel_integral);

    set_motor_pwm(0, (int)m1_output);
    set_motor_pwm(1, (int)m2_output);
}

#endif