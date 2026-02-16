#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "Config.h"
#include "VmEvents.h"

class NetworkManager {
public:
    NetworkManager(QueueHandle_t cmd_queue, QueueHandle_t log_queue);

    void begin();
    void update();      
    void process_logs(); 

private:
    void connect_wifi();
    void reconnect_mqtt();
    void send_heartbeat();
    
    static void mqtt_callback_static(char* topic, byte* payload, unsigned int length);
    void handle_mqtt_message(char* topic, byte* payload, unsigned int length);

    WiFiClient espClient;
    PubSubClient client;
    
    QueueHandle_t q_commands;
    QueueHandle_t q_logs;

    char format_buffer[128]; 

    // Dynamic Topic Buffers
    char topic_program[64];
    char topic_control[64];
    char topic_console[64];
    char topic_log[64];
    char topic_error[64];
    char topic_status[64];

    //Heartbeat instance 
    unsigned long last_heartbeat_time;

    static NetworkManager* instance;

};

#endif