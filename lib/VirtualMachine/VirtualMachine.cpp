#include "VirtualMachine.h"

// --- PUBLIC METHODS ---

VirtualMachine::VirtualMachine(QueueHandle_t log_queue_handle):
    display(128, 64, &Wire, -1),
    log_queue(log_queue_handle),
    display_initialized(false),
    neostrip(nullptr),     
    neo_initialized(false),
    delay_time(0)
{   
    #ifdef ENABLE_MOTOR_PINS
        robot.begin();       //start robot
    #endif


    #ifdef ENABLE_BOOT_NEOPIXEL
        
        neostrip = new Adafruit_NeoPixel(NEO_BOOT_LEDS, NEO_BOOT_PIN, NEO_COLOR_ORDER);
        neostrip->begin();
        neostrip->setBrightness(NEO_BOOT_BRIGHTNESS);
        neostrip->clear();
        
        // Boot Sequence: Green Flash
        neostrip->fill(neostrip->Color(0, 255, 0)); 
        neostrip->show();
        delay(250); 
        neostrip->clear();
        neostrip->show();

        neo_initialized = true;

    #endif
}

void VirtualMachine::send_print(const char* msg) {
    VmLog log;
    log.type = TYPE_USER_PRINT;
    strncpy(log.message, msg, 63);
    log.message[63] = '\0';
    // Send with 0 wait time (non-blocking)
    xQueueSend(log_queue, &log, 0);
}

void VirtualMachine::abort_with_error(const char* msg) {
    VmLog log;
    log.type = TYPE_RUNTIME_ERROR;
    strncpy(log.message, msg, 63);
    log.message[63] = '\0';
    
    // IMPORTANT: Wait up to 10ms to ensure error gets queued before dying
    xQueueSend(log_queue, &log, pdMS_TO_TICKS(10));
    
    vm_state = VM_ERROR;
}

void VirtualMachine::send_debug(const char* msg) {
    VmLog log;
    log.type = TYPE_SYSTEM_DEBUG;
    strncpy(log.message, msg, 63);
    log.message[63] = '\0';
    xQueueSend(log_queue, &log, 0);
}

void VirtualMachine::reset() {
    frame_ptr = 0;
    memset(&call_frames[0], 0, sizeof(CallFrame));
    variables_ptr = call_frames[0].vars;
    pc = 0; sp = -1; rsp = -1;
    setup_start_addr = -1; loop_start_addr = -1;
    program_length = 0; string_pool_ptr = 0; string_count = 0;
    vm_state = VM_STOPPED;

    // Hardware Reset
    for (int i = 0; i < VM_MAX_SERVOS; i++) {
        if (servos[i].attached()) servos[i].detach();
        servo_pins[i] = -1;
    }
    // OLED
    if (display_initialized) {
        display.clearDisplay();
        display.setCursor(0, 0); 
        display.display();
    }

    display_initialized = false; //Resets OLED screen flag

    // NEOPIXEL
    #ifndef ENABLE_BOOT_NEOPIXEL
        // Standard Behavior: Clean up to prevent memory leaks
        if (neostrip != nullptr) {
            neostrip->clear();
            neostrip->show(); // Turn off LEDs physically
            delete neostrip;  // Free the memory
            neostrip = nullptr;
            neo_initialized = false;
        }
    #else
        // Boot Behavior: Keep the object alive.
        // We ensure neo_initialized remains true so the next program can use it immediately.
        if (neostrip != nullptr) {
            neostrip->clear();
            neostrip->show(); // Turn off LEDs physically
            neo_initialized = true; 
        }
    #endif

    reset_gpio_pins();
    
    delay_time = 0;     //Reset the delay time to zero

    #ifdef ENABLE_MOTOR_PINS
        robot.stop();       //stop robot
    #endif
 
    
    send_debug("VM Reset Complete");
}


bool VirtualMachine::load_program_binary(const uint8_t* payload, size_t length) {
    if (length < 8) { 
        abort_with_error("Binary too small"); 
        return false; 
    }

    //Parse string_length
    uint32_t string_len = 
        ((uint32_t)payload[0] << 24) | 
        ((uint32_t)payload[1] << 16) | 
        ((uint32_t)payload[2] << 8)  | 
        (uint32_t)payload[3];

    //
    uint32_t instr_len = 
        ((uint32_t)payload[4] << 24) | 
        ((uint32_t)payload[5] << 16) | 
        ((uint32_t)payload[6] << 8)  | 
        (uint32_t)payload[7];

    if (string_len >= STRING_POOL_SIZE || instr_len > sizeof(program)) {
        abort_with_error("Program too large");
        return false;
    }

    reset(); // Clear old state

    // Load Strings
    if (string_len > 0) {
        memcpy(string_pool, payload + 8, string_len);
        string_pool_ptr = string_len;
        string_addresses[0] = 0;
        string_count = 1;
        for (int i = 0; i < string_len - 1 && string_count < MAX_STRINGS; i++) {
            if (string_pool[i] == '\0') string_addresses[string_count++] = i + 1;
        }
    }

    // Load Instructions
    memcpy(program, payload + 8 + string_len, instr_len);
    program_length = instr_len / sizeof(Instruction);

    // Scan for entry points
    for (int i = 0; i < program_length; i++) {
        if (program[i].opcode == OP_SETUP_START) setup_start_addr = i + 1;
        if (program[i].opcode == OP_LOOP_START) loop_start_addr = i + 1;
    }

    send_debug("Program Loaded Successfully");
    return true;
}


void VirtualMachine::run_tick() {
    
    #ifdef ENABLE_MOTOR_PINS
    robot.update();
    #endif

    if (vm_state != VM_RUNNING) return;

    // NON-BLOCKING DELAY CHECK 
    // If we are currently in a "delay", check if time is up.
    // If not, return immediately so we don't block the CPU.
    if (delay_time > 0) {
        if (millis() < delay_time) {
            return; // Still waiting -> Yield to main loop
        } else {
            delay_time = 0; // Time is up -> Continue execution
        }
    }

    // Robot Busy Check
    #ifdef ENABLE_MOTOR_PINS
    if (robot.is_busy()) return;
    #endif 

    // Execute Batch
    for (int i = 0; i < 100; i++) {
        if (vm_state != VM_RUNNING) break;
        
        #ifdef ENABLE_MOTOR_PINS
        if (robot.is_busy()) break;
        #endif
        // If an instruction sets a delay, break immediately to yield
        if (delay_time > 0) break; 

        execute_instruction();
    }
}



// --- PRIVATE HELPER METHODS ---


/**
 * The heart of the VM. Executes one instruction.
 */
void VirtualMachine::execute_instruction() {
    if (vm_state != VM_RUNNING) return; 

    if (pc >= program_length || pc < 0) {
        if (loop_start_addr != -1) {
            pc = loop_start_addr; // Auto-loop 
        } else {
            vm_state = VM_HALTED; // No loop, halt 
            send_debug("VM Halted (program finished).");
            return;
        }
    }

    Instruction instr = program[pc];

    // Check for HALT at the top
    if (instr.opcode == OP_HALT) { 
        vm_state = VM_HALTED;
        send_debug("VM Halted by OP_HALT instruction.");
        return; 
    }

    pc++; 

    // --- Opcode Execution ---
    int a, b, c, d, e;
    int color;
    int servo_index, servo_pin, servo_angle;
    int trigPin, echoPin;
    long duration, distance_cm;

    switch (instr.opcode) {
        case OP_NOP: 
            break; 
        case OP_SETUP_START: 
            break; 
        case OP_LOOP_START: 
            break; 
        case OP_PUSH: 
            push(instr.arg1);
            break;
        case OP_POP: 
            pop();
            break;

        case OP_LOAD: 
            if (instr.arg1 >= 0 && instr.arg1 < VARIABLES_SIZE && variables_ptr) {
                push(variables_ptr[instr.arg1]);
            } else abort_with_error("Invalid variable address in OP_LOAD");
            break;

        case OP_STORE: 
            if (instr.arg1 >= 0 && instr.arg1 < VARIABLES_SIZE && variables_ptr) {
                variables_ptr[instr.arg1] = pop();
            } else abort_with_error("Invalid variable address in OP_STORE");
            break;

        case OP_PIN_MODE: 
            b = pop(); a = pop();
            if (b == 0) pinMode(a, INPUT);
            else if (b == 1) pinMode(a, OUTPUT);
            //else if (b == 2) pinMode(a, INPUT_PULLUP);
            break;

        case OP_DIGITAL_WRITE: 
            b = pop();
            a = pop();
            digitalWrite(a, b);
            break; 
        case OP_DIGITAL_READ:  
            push(digitalRead(pop()))
            ; break; 
        case OP_ANALOG_WRITE:  
            b = pop();
            a = pop();
            analogWrite(a, b);
            break; 
        case OP_ANALOG_READ:   
            push(analogRead(pop()));
            break; 

        //Will be Removed later
        case OP_DELAY:
            //vTaskDelay(pdMS_TO_TICKS(pop())); 
            a = pop();
            if (a > 0) {
                delay_time = millis() + a; // Set the target time
            }
            break;

        case OP_ADD: 
            b = pop();
            a = pop();
            push(a + b);
            break; 

        case OP_SUB: 
            b = pop();
            a = pop();
            push(a - b);
            break; 

        case OP_MUL: 
            b = pop(); 
            a = pop(); 
            push(a * b);
            break;

        case OP_DIV: 
            b = pop(); a = pop();
            if (b == 0) { abort_with_error("Division by zero"); } else { push(a / b); }
            break;

        case OP_MOD: 
            b = pop(); a = pop();
            if (b == 0) { abort_with_error("Division by zero"); } else { push(a % b); }
            break;

        case OP_POW: 
            b = pop();
            a = pop();
            push(pow(a, b));
            break; 


        case OP_AND: 
            b = pop();
            a = pop(); 
            push(a && b);
            break; 


        case OP_OR:  
            b = pop();
            a = pop();
            push(a || b);
            break;

        case OP_NOT: 
            push(!pop());
            break;

        case OP_EQ:  
            b = pop();
            a = pop();
            push(a == b);
            break; 

        case OP_NEQ: 
            b = pop();
            a = pop();
            push(a != b);
            break;

        case OP_GT:  
            b = pop();
            a = pop();
            push(a > b);
            break;

        case OP_LT:  
            b = pop();
            a = pop();
            push(a < b);
            break;

        case OP_GTE: 
            b = pop();
            a = pop();
            push(a >= b);
            break;

        case OP_LTE: 
            b = pop();
            a = pop();
            push(a <= b);
            break;

        case OP_JUMP: 
            pc = instr.arg1;
            break;

        case OP_JUMP_IF_FALSE:
            a = pop(); 
            if (a == 0) { 
                pc = instr.arg1; 
            } 
            break;

        case OP_CALL: 
            if (rsp >= CALL_STACK_SIZE - 1 || frame_ptr >= CALL_STACK_SIZE - 1) {
                abort_with_error("Call stack overflow"); return;
            }
            return_stack[++rsp] = pc;
            frame_ptr++;
            memset(&call_frames[frame_ptr], 0, sizeof(CallFrame));
            variables_ptr = call_frames[frame_ptr].vars;
            pc = instr.arg1;
            break;

        case OP_RET: 
            if (rsp < 0 || frame_ptr < 1) {
                abort_with_error("Call stack underflow or RET from global scope"); return;
            }
            pc = return_stack[rsp--];
            frame_ptr--;
            variables_ptr = call_frames[frame_ptr].vars;
            break;

        case OP_PRINT_INT: { 
            char buffer[12];
            snprintf(buffer, sizeof(buffer), "%d", pop());
            send_print(buffer);
            break;
        }
        case OP_PRINT_STRING: { 
            int string_index = instr.arg1;
            if (string_index >= 0 && string_index < string_count) {
                send_print(&string_pool[string_addresses[string_index]]);
            } else {
                abort_with_error("Invalid string index");
            }
            break;
        }
        case OP_MAP: { 
            e = pop(); d = pop(); c = pop(); b = pop(); a = pop();
            push(map(a, b, c, d, e));
            break;
        }
        case OP_SERVO_ATTACH: 
            servo_pin = pop(); servo_index = pop();
            if (servo_index >= 0 && servo_index < MAX_SERVOS) {
                if (servos[servo_index].attached()) { servos[servo_index].detach(); }
                servos[servo_index].attach(servo_pin);
                servo_pins[servo_index] = servo_pin;
            } else { abort_with_error("Invalid servo index"); }
            break;

        case OP_SERVO_WRITE: 
            servo_angle = pop(); servo_index = pop();
            if (servo_index >= 0 && servo_index < MAX_SERVOS) {
                if (servos[servo_index].attached()) {
                    servos[servo_index].write(servo_angle);
                } else { abort_with_error("Servo not attached"); }
            } else { abort_with_error("Invalid servo index"); }
            break;

        case OP_SERVO_READ: 
            servo_index = pop();
            if (servo_index >= 0 && servo_index < MAX_SERVOS) {
                if (servos[servo_index].attached()) {
                    push(servos[servo_index].read());
                } else { abort_with_error("Servo not attached"); push(0); }
            } else { abort_with_error("Invalid servo index"); push(0); }
            break;

        case OP_ULTRASONIC_READ_CM: 
            echoPin = pop(); trigPin = pop();
            pinMode(trigPin, OUTPUT);
            pinMode(echoPin, INPUT);
            digitalWrite(trigPin, LOW);
            delayMicroseconds(2);
            digitalWrite(trigPin, HIGH);
            delayMicroseconds(10);
            digitalWrite(trigPin, LOW);
            duration = pulseIn(echoPin, HIGH);
            distance_cm = duration / 58;
            push(distance_cm);
            break;
        
        // --- OLED OPCODES ---
        case OP_OLED_INIT: {
            // specific I2C Address check (usually 0x3C)
            if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
                abort_with_error("OLED Hardware Failure (Check Wiring)");
            } else {
                display_initialized = true;
                display.clearDisplay();
                display.display();
                // defaults 
                display.setTextColor(SSD1306_WHITE);
                display.setTextSize(1);
            }
            break;
        }

        case OP_OLED_CLEAR:
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.clearDisplay();
            break;

        case OP_OLED_UPDATE:
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.display();
            break;

        case OP_OLED_DRAW_PIXEL: {
            int color = pop();
            int y = pop();
            int x = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.drawPixel(x, y, color);
            break;
        }

        case OP_OLED_DRAW_LINE: {
            int color = pop();
            int y2 = pop();
            int x2 = pop();
            int y1 = pop();
            int x1 = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.drawLine(x1, y1, x2, y2, color);
            break;
        }

        case OP_OLED_DRAW_RECT: {
            int color = pop();
            int h = pop();
            int w = pop();
            int y = pop();
            int x = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.drawRect(x, y, w, h, color);
            break;
        }

        case OP_OLED_FILL_RECT: {
            int color = pop();
            int h = pop();
            int w = pop();
            int y = pop();
            int x = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.fillRect(x, y, w, h, color);
            break;
        }

        case OP_OLED_SET_CURSOR: {
            int y = pop();
            int x = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.setCursor(x, y);
            break;
        }

        case OP_OLED_SET_TEXT_SIZE: {
            int size = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.setTextSize(size);
            break;
        }

        case OP_OLED_SET_TEXT_COLOR: {
            int color = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.setTextColor(color);
            break;
        }

        case OP_OLED_PRINT_TEXT: {
            int string_index = instr.arg1;
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            
            if (string_index >= 0 && string_index < string_count) {
                display.print(&string_pool[string_addresses[string_index]]);
            } else {
                abort_with_error("Invalid String Index");
            }
            break;
        }

        case OP_OLED_PRINT_NUMBER: {
            int value = pop();
            if (!display_initialized) { abort_with_error("OLED not initialized"); break; }
            display.print(value);
            break;
        }

        case OP_NEOPIXEL_INIT: {
            int pin = pop();
            int count = pop();

            // Safety Limits
            if (count <= 0 || count > MAX_NEOPIXELS) {
                abort_with_error("NeoPixel Count Invalid (Max 64)");
                break;
            }
            if (pin < 0 || pin > 34) {
                abort_with_error("Invalid NeoPixel Pin");
                break;
            }

            //Clean up existing strip if user calls Init twice
            if (neostrip != nullptr) {
                delete neostrip;
            }

            // Allocate New Strip
            neostrip = new Adafruit_NeoPixel(count, pin, NEO_GRB + NEO_KHZ800);
            if (neostrip) {
                neostrip->begin();
                neostrip->show(); // Set to off
                neostrip->setBrightness(50); // Default
                neo_initialized = true;
            } else {
                abort_with_error("NeoPixel Alloc Failed");
            }
            break;
        }

        case OP_NEOPIXEL_SET_BRIGHTNESS: {
            int val = pop();
            if (!neo_initialized) { abort_with_error("NeoPixel not initialized"); break; }
            neostrip->setBrightness(constrain(val, 0, 255));
            break;
        }

        case OP_NEOPIXEL_SET_PIXEL: {
            // User Logic: Stack has [Color, Index] 
            int index = pop(); 
            uint32_t color = (uint32_t)pop(); 

            if (!neo_initialized) { abort_with_error("NeoPixel not initialized"); break; }

            if (index >= 0 && index < neostrip->numPixels()) {
                neostrip->setPixelColor(index, color);
            } else {
                // Report error if index is wrong, or ignore safely
                abort_with_error("NeoPixel Index Out of Bounds");
            }
            break;
        }

        case OP_NEOPIXEL_FILL: {
            uint32_t color = (uint32_t)pop();
            if (!neo_initialized) { abort_with_error("NeoPixel not initialized"); break; }
            neostrip->fill(color);
            break;
        }

        case OP_NEOPIXEL_CLEAR:
            if (!neo_initialized) { abort_with_error("NeoPixel not initialized"); break; }
            neostrip->clear();
            break;

        case OP_NEOPIXEL_SHOW:
            if (!neo_initialized) { abort_with_error("NeoPixel not initialized"); break; }
            neostrip->show(); 
            break;

        // Motor Controls 
        
        #ifdef ENABLE_MOTOR_PINS
        case OP_MOVE_CM: {
            int cm = pop();
            robot.move_cm((float)cm);
            break;
        }
        case OP_TURN_DEG: {
            int deg = pop();
            robot.turn_degrees((float)deg);
            break;
        }
        case OP_MOTOR_SET_SPEED: {
            int speed = pop();
            int index = pop();
            robot.set_speed(index, speed);
            break;
        }
        case OP_ROBOT_STOP:
            robot.stop();
            break;

        
        case OP_TONE: 
            a = pop(); // Frequency (Hz)
            // Safety: Ensure freq is reasonable
            if (a > 0) {
                tone(Buzzer_PIN, a); 
            } else {
                abort_with_error("Invalid Tone Params");
            }
            break;

        case OP_NO_TONE: 
            noTone(Buzzer_PIN);
            break;


        #endif

        case OP_HALT: break; // Handled at the top
        default: abort_with_error("Unknown opcode ID"); break;
    }
        
}

void VirtualMachine::push(int value) {
    if (sp >= STACK_SIZE - 1) abort_with_error("Stack Overflow");
    else stack[++sp] = value;
}

int VirtualMachine::pop() {
    if (sp < 0) {
        abort_with_error("Stack Underflow");
        return 0;
    }
    return stack[sp--];
}

void VirtualMachine::set_state(VmState new_state) {
    vm_state = new_state;
    if (vm_state == VM_STOPPED){
        reset_gpio_pins();

        delay_time = 0; 
        #ifdef ENABLE_MOTOR_PINS
            robot.stop();
        #endif
    }
}

VmState VirtualMachine::get_state() { return vm_state; }


void VirtualMachine::reset_gpio_pins() {
    // Uses the SAFE_BOOT_PINS array defined in Config.h
    for (int i = 0; i < SAFE_BOOT_PIN_COUNT; i++) {
        pinMode(SAFE_BOOT_PINS[i], INPUT);
    }
}

