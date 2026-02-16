#include "NetworkManager.h"

NetworkManager* NetworkManager::instance = nullptr;

NetworkManager::NetworkManager(QueueHandle_t cmd_queue, QueueHandle_t log_queue) 
    : q_commands(cmd_queue), q_logs(log_queue), client(espClient) {
    instance = this;
}

void NetworkManager::begin() {

    // Combines "MQTT_CLIENT_ID" + "/program" -> "MQTT_CLIENT_ID/program"
    snprintf(topic_program, sizeof(topic_program), "%s/program", MQTT_CLIENT_ID); //The topic that will recieve the commands from 
    snprintf(topic_control, sizeof(topic_control), "%s/control", MQTT_CLIENT_ID); //The control it will receve the run/stop commands
    snprintf(topic_console, sizeof(topic_console), "%s/console", MQTT_CLIENT_ID); //The user console 
    //snprintf(topic_error,   sizeof(topic_error),   "%s/console",   MQTT_CLIENT_ID); Optional if you want separate error topic
    snprintf(topic_status,  sizeof(topic_status),  "%s/status",  MQTT_CLIENT_ID); //Sends heartbeat and the current status to the IDE 

    #if ENABLE_SYSTEM_LOGS
        snprintf(topic_log,     sizeof(topic_log),     "%s/log",     MQTT_CLIENT_ID); //for debug only
    #endif


    //Debug print to confirm topics are correct
    //Serial.print("Listening on: "); Serial.println(topic_program);

    connect_wifi();
    
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(NetworkManager::mqtt_callback_static);
    client.setBufferSize(4096); 
}

void NetworkManager::connect_wifi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
}

void NetworkManager::reconnect_mqtt() {
    if (!client.connected()) {
        if (client.connect(MQTT_CLIENT_ID)) {
            Serial.println("MQTT Connected");
            client.subscribe(topic_program);
            client.subscribe(topic_control);
        }
    }
}

void NetworkManager::update() {
    if (!client.connected()) {
        reconnect_mqtt();
    }
    client.loop();

    // --- NON-BLOCKING HEARTBEAT ---
    unsigned long now = millis();
    if (now - last_heartbeat_time > HEARTBEAT_INTERVAL_MS) {
        last_heartbeat_time = now;
        send_heartbeat();
    }
}

void NetworkManager::send_heartbeat() {
    if (!client.connected()) return;

    // Create a small JSON payload
    // "rssi": Signal Strength (dBm).
    char payload[64];
    long rssi = WiFi.RSSI();
    
    snprintf(payload, sizeof(payload), "{\"alive\":1, \"rssi\":%ld}", rssi);

    // Publish to "robot_109/status"
    client.publish(topic_status, payload);
}

void NetworkManager::mqtt_callback_static(char* topic, byte* payload, unsigned int length) {
    if (instance) {
        instance->handle_mqtt_message(topic, payload, length);
    }
}

void NetworkManager::handle_mqtt_message(char* topic, byte* payload, unsigned int length) {
    VmCommand cmd;

    // PROGRAM UPLOAD
    if (strcmp(topic, topic_program) == 0) {
        cmd.type = CMD_LOAD_PROGRAM;
        cmd.payload_ptr = (uint8_t*)malloc(length);
        
        if (cmd.payload_ptr) {
            memcpy(cmd.payload_ptr, payload, length);
            cmd.length = length;
            xQueueSend(q_commands, &cmd, 0); 
        } else {
            // PUBLISH TO SPECIFIC CONSOLE
            client.publish(topic_console, "[SYSTEM] Out of Memory during load");
        }
    }
    // CONTROL COMMANDS
    else if (strcmp(topic, topic_control) == 0) {
        String msg = "";
        for(unsigned int i=0; i<length; i++) msg += (char)payload[i];
        
        if (msg == "stop") { cmd.type = CMD_STOP; xQueueSend(q_commands, &cmd, 0); }
        else if (msg == "run") { cmd.type = CMD_RUN; xQueueSend(q_commands, &cmd, 0); }
        else if (msg == "reset") { cmd.type = CMD_RESET; xQueueSend(q_commands, &cmd, 0); }
    }
}

void NetworkManager::process_logs() {
    VmLog log;
    
    while (xQueueReceive(q_logs, &log, 0) == pdTRUE) {
        if (!client.connected()) continue;

        switch (log.type) {
            case TYPE_USER_PRINT:
                client.publish(topic_console, log.message);
                break;

            case TYPE_RUNTIME_ERROR:
                snprintf(format_buffer, sizeof(format_buffer), "[ERROR] %s", log.message);
                client.publish(topic_console, format_buffer);
                break;

            case TYPE_SYSTEM_DEBUG:
                #if ENABLE_SYSTEM_LOGS
                    client.publish(topic_log, log.message);
                #endif
                break;
        }
    }
}