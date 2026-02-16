#ifndef VM_EVENTS_H
#define VM_EVENTS_H

#include <Arduino.h>

// --- COMMANDS (Network -> VM) ---
// These are instructions for the VM Controller
enum VmCommandType {
    CMD_STOP,
    CMD_RUN,
    CMD_RESET,
    CMD_LOAD_PROGRAM
};

struct VmCommand {
    VmCommandType type;
    uint8_t* payload_ptr; // Pointer to the binary data (for loading)
    size_t length;        // Size of the binary data
};

// --- LOGS & OUTPUTS (VM -> Network) ---
// These separate "User Print" from "System Errors"
enum LogType {
    TYPE_USER_PRINT,    // User typed print("hello") -> Console
    TYPE_RUNTIME_ERROR, // User divided by zero -> [ERROR] Msg
    TYPE_SYSTEM_DEBUG   // VM internal info -> Debug log
};

struct VmLog {
    LogType type;
    char message[64]; 
};

#endif