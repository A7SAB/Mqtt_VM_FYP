#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =========================================================
// 1. WIFI & NETWORK SETTINGS
// =========================================================
constexpr const char* WIFI_SSID       = "YOUR_WIFI_SSID";
constexpr const char* WIFI_PASSWORD   = "YOUR_WIFI_PASSWORD";
constexpr const char* MQTT_SERVER     = "YOUR_MQTT_SERVER";
constexpr int         MQTT_PORT       = 1883;
constexpr const char* MQTT_CLIENT_ID  = "robot_1"; //This is the topic prefix

// Heartbeat Interval 
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 2000;

// Toggle this to hide system debug 
#define ENABLE_SYSTEM_LOGS true 


// =========================================================
// 2. VM PARAMETERS & LIMITS
// =========================================================
#define VM_MAX_SERVOS   4
#define MAX_NEOPIXELS   64
#define PROGRAM_SIZE    1024
#define STACK_SIZE      64
#define CALL_STACK_SIZE 64


// =========================================================
// 3. ROBOT MOTOR PINS
// =========================================================

//This setting pretty much removes all of the motor customization & commands from the VM 
#define ENABLE_MOTOR_PINS // Comment this line out to disable motor pins

#ifdef ENABLE_MOTOR_PINS

    #define Buzzer_PIN     2

    // Motor 1 (Left)
    #define M1_PWM_PIN    25
    #define M1_IN1_PIN    13
    #define M1_IN2_PIN    14
    #define M1_ENC_A_PIN  32 
    #define M1_ENC_B_PIN  34 

    // Motor 2 (Right)
    #define M2_PWM_PIN    15
    #define M2_IN1_PIN    18
    #define M2_IN2_PIN    19
    #define M2_ENC_A_PIN  16
    #define M2_ENC_B_PIN  17

    // =========================================================
    // 4. ROBOT GEOMETRY & PHYSICS
    // =========================================================
    #define WHEEL_DIAMETER_MM   43.0
    #define WHEEL_TRACK_MM      126
    #define TICKS_PER_REV       680.0
    // Max RPM clamp for PID
    constexpr float MAX_RPM_LIMIT = 150.0;

    // =========================================================
    // 5. PID CONTROLLER CONSTANTS
    // =========================================================
    // Position Loop (Outer) (Currently only Basic P controller is used for position)
    constexpr double PID_POS_P = 1.25;
    //constexpr double PID_POS_I = 0.08;
    //constexpr double PID_POS_D = 0.12;

    // Velocity Loop (Inner) (Currently only Basic PI controller is used for velocity)
    constexpr double PID_VEL_P = 0.85;
    constexpr double PID_VEL_I = 3.75;
    //constexpr double PID_VEL_D = 0.0075;

    // Sampling time
    constexpr float PID_SAMPLE_TIME_S = 0.05; // 50ms

    // =========================================================
    // 6. SAFE BOOT PINS
    // =========================================================
    // Pins that are safe to reset to INPUT mode (prevents robot running off)
    // Exclude Motor pins & NeoPixel LED pins. 

    constexpr int SAFE_BOOT_PINS[] = {2,5, 12, 21, 22, 23, 26, 27, 33}; //For the robot
    constexpr int SAFE_BOOT_PIN_COUNT = sizeof(SAFE_BOOT_PINS) / sizeof(SAFE_BOOT_PINS[0]); 


    #else

    constexpr int SAFE_BOOT_PINS[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33}; //This is for Vanilla Version
    constexpr int SAFE_BOOT_PIN_COUNT = sizeof(SAFE_BOOT_PINS) / sizeof(SAFE_BOOT_PINS[0]); 


#endif




// =========================================================
// 7. NEOPIXEL CONFIG
// =========================================================
#define ENABLE_BOOT_NEOPIXEL // Comment this line out to disabele boot code for NeoPixels

#ifdef ENABLE_BOOT_NEOPIXEL
    constexpr int NEO_BOOT_PIN        = 4;  
    constexpr int NEO_BOOT_LEDS       = 5;   
    constexpr int NEO_BOOT_BRIGHTNESS = 50; 
    
    // Color Order: NEO_GRB, NEO_RGB, or NEO_GRBW
    #define NEO_COLOR_ORDER   (NEO_GRB + NEO_KHZ800)
#endif 

#endif