# ESP32 Basic VM Over MQTT
This project is used to send basic instructions over MQTT from a basic ESP32 based robot. Its main purpose is to be used in educational enviroments to teach students basic programming skills. Its used to run with the web app found in https://github.com/A7SAB/Blockly_Proj_fyp. 

## Project Structure 
- **include/** – Additional project-level headers.  
- **lib/** – Custom libraries:  
  - `Config` – User configuration.  
  - `NetworkManager` – Handles network communication.  
  - `RobotController` – Controls robot motors controls.   
  - `VirtualMachine` – Executes instructions and logic.  
  - `VmEvents` – Defines events for the virtual machine.  
- **src/** – Main application code (`main.cpp`).  
- **test/** – Unit tests for libraries and core functions.  

## Configuration

Before running the project, you need to configure your network and MQTT settings in `lib/Config/Config.h`. Update the following variables with your own credentials:

```cpp
constexpr const char* WIFI_SSID       = "YOUR_WIFI_SSID";
constexpr const char* WIFI_PASSWORD   = "YOUR_WIFI_PASSWORD";
constexpr const char* MQTT_SERVER     = "YOUR_MQTT_SERVER";
constexpr int         MQTT_PORT       = 1883;
constexpr const char* MQTT_CLIENT_ID  = "robot_1"; 
```


