#include <Arduino.h>
#include <Wire.h> 
#include "VirtualMachine.h"
#include "NetworkManager.h"
#include "Config.h"

// --- RTOS OBJECTS ---
QueueHandle_t q_vm_commands; 
QueueHandle_t q_vm_logs;     

// --- GLOBAL OBJECTS ---
VirtualMachine* vm = nullptr;
NetworkManager* net = nullptr;

// --- TASK 1: VM ENGINE (High Priority, Core 1) ---
void task_vm_engine(void *pvParameters) {
    VmCommand cmd;
    const TickType_t tick_rate = pdMS_TO_TICKS(10);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // Check for commands from Network Task
        while (xQueueReceive(q_vm_commands, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case CMD_STOP: vm->set_state(VM_STOPPED); break;
                case CMD_RUN:  vm->set_state(VM_RUNNING); break;
                case CMD_RESET: vm->reset(); break;
                
                case CMD_LOAD_PROGRAM:
                    vm->set_state(VM_STOPPED);
                    vm->load_program_binary(cmd.payload_ptr, cmd.length);
                    
                    // Free the memory allocated in NetworkManager for the new program
                    free(cmd.payload_ptr); 
                    break;
            }
        }

        // Run one VM tick
        if (vm) vm->run_tick();

        // Wait until next tick 
        vTaskDelayUntil(&last_wake_time, tick_rate);
    }
}

// --- TASK 2: NETWORK MANAGER (Low Priority, Core 0) ---
void task_network_manager(void *pvParameters) {
    // Initialize the Manager
    net->begin();

    while (true) {
        // Maintain Connection
        net->update();

        // Publish Logs
        net->process_logs();

        vTaskDelay(10); 
    }
}


void setup() {
    Serial.begin(115200);
    while(!Serial) { delay(10); } 
    //Serial.println("\n\n--- BOOTING ---");

    // Initialize I2C (Hardware) BEFORE the VM
    //Serial.println("[1] Initializing Hardware...");
    Wire.begin(21, 22); 

    //Serial.println("[2] Creating Queues...");
    q_vm_commands = xQueueCreate(10, sizeof(VmCommand));
    q_vm_logs = xQueueCreate(20, sizeof(VmLog));
    
    if (q_vm_commands == NULL || q_vm_logs == NULL) {
        //Serial.println("ERROR: Queue creation failed!");
        while(1); 
    }

    // Initialize Logic Objects
    //Serial.println("[3] Initializing Managers...");
    vm = new VirtualMachine(q_vm_logs);
    net = new NetworkManager(q_vm_commands, q_vm_logs);
    
    vm->reset();

    //Serial.println("[4] Starting RTOS Tasks...");
    
    xTaskCreatePinnedToCore(task_network_manager, "NetTask", 10240, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_vm_engine, "VMTask", 10240, NULL, 2, NULL, 1);

    //Serial.println("[5] System Running.");
}

void loop() {
    vTaskDelete(NULL); 
}