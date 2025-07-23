/*********************************************************************
 Mouse Remapper with TinyUSB Host/Device and KM Command Compatibility
 
 This firmware acts as a USB mouse remapper that:
 - Receives mouse reports from a connected USB mouse (host mode)
 - Mirrors all reports to PC regardless of UART connection (device mode)
 - Provides km. serial command interface for button control and movement
 - Uses NeoPixel for status indication (red->green->rainbow->active)
 
 Serial Commands (km. format):
 - km.left(1) / km.left(0) - Press/release left button
 - km.right(1) / km.right(0) - Press/release right button  
 - km.middle(1) / km.middle(0) - Press/release middle button
 - km.side1(1) / km.side1(0) - Press/release side button 1
 - km.side2(1) / km.side2(0) - Press/release side button 2
 - km.click(0-4) - Click button (0=left, 1=right, 2=middle, 3=side1, 4=side2)
 - km.move(x, y) - Move mouse by x,y pixels
 - km.wheel(amount) - Scroll wheel up (+) or down (-)
 - km.lock_mx() / km.lock_mx(0/1) - Get/set X axis lock
 - km.lock_my() / km.lock_my(0/1) - Get/set Y axis lock
 - km.lock_ml(0/1) - Lock/unlock left button
 - km.lock_mr(0/1) - Lock/unlock right button
 - km.lock_mm(0/1) - Lock/unlock middle button
 - km.lock_ms1(0/1) - Lock/unlock side button 1
 - km.lock_ms2(0/1) - Lock/unlock side button 2
 - km.buttons() - Get current button state
 *********************************************************************/

// Pin configuration
#ifndef NEOPIXEL_PIN
#define NEOPIXEL_PIN PIN_NEOPIXEL
#endif

#ifndef NEOPIXEL_COUNT
#define NEOPIXEL_COUNT 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN PIN_LED  // Default for most ESP32 boards
#endif

// USBHost configuration
#define CFG_TUH_MAX3421 1
#include "usbh_helper.h"
#include <Adafruit_NeoPixel.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//--------------------------------------------------------------------+
// Constants and Configuration
//--------------------------------------------------------------------+

#define REMAPPER_CMD_BUFFER_SIZE 128
#define MAX_QUEUED_REPORTS 16  // Power of 2 for faster masking
#define QMASK (MAX_QUEUED_REPORTS-1)
#define BOOT_PHASE_DURATION_MS 2000
#define REPORT_ACTIVITY_TIMEOUT_MS 500
#define RAINBOW_SPEED 50
#define REPORT_ACTIVITY_SPEED 25
#define RELEASE_MIN_TIME_MS 125
#define RELEASE_MAX_TIME_MS 175
#define CLICK_PRESS_MIN_TIME_MS 75
#define CLICK_PRESS_MAX_TIME_MS 125

// Debug control - comment out to disable USB debug spam
// #define DEBUG_USB

#ifndef DEBUG_USB
  #define DLOG(...)
#else
  #define DLOG(...) Serial.printf(__VA_ARGS__)
#endif

//--------------------------------------------------------------------+
// Data Structures
//--------------------------------------------------------------------+

typedef struct {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
    int8_t pan;
} mouse_report_t;

typedef struct {
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycode[6];
} keyboard_report_t;

typedef enum {
    STATUS_BOOT_RED = 0,
    STATUS_BOOT_GREEN,
    STATUS_RAINBOW,
    STATUS_REPORT_ACTIVE
} neopixel_status_t;

typedef struct {
    bool is_pressed;
    bool is_forced;
    uint32_t release_time;
    bool is_clicking;
    uint32_t click_release_start;
    uint32_t click_end_time;
    bool is_locked;
} km_button_state_t;

typedef struct {
    mouse_report_t reports[MAX_QUEUED_REPORTS];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} report_queue_t;

typedef struct {
    char buffer[REMAPPER_CMD_BUFFER_SIZE];
    uint8_t buffer_pos;
    bool in_command;
    bool skip_next_terminator;
    char last_terminator;
    char command_terminator[3];
    uint8_t terminator_len;
} command_parser_t;

typedef struct {
    // KM-style button states
    km_button_state_t km_buttons[5];
    
    // Report queue for injection
    report_queue_t inject_queue;
    
    // Parser state
    command_parser_t parser;
    
    // Status tracking
    neopixel_status_t current_status;
    uint32_t last_status_update;
    bool reports_active;
    uint32_t last_report_time;
    
    // Statistics
    uint32_t reports_processed;
    uint32_t reports_injected;
    
    // Timing
    uint32_t boot_start_time;
    bool boot_complete;
    
    // KM movement accumulators
    int16_t km_mouse_x_accumulator;
    int16_t km_mouse_y_accumulator;
    int8_t km_wheel_accumulator;
    
    // Physical button state from actual mouse
    uint8_t physical_buttons;
    
    // Axis locks
    bool lock_mx;
    bool lock_my;
} remapper_state_t;

//--------------------------------------------------------------------+
// Global Variables
//--------------------------------------------------------------------+

static remapper_state_t g_state = {0};
static uint32_t g_rand_seed = 0x12345678;

// HID report descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID device
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_MOUSE, 2, false);

// NeoPixel strip
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Button names and masks
static const char* button_names[5] = {"left", "right", "middle", "side1", "side2"};
static const char* lock_button_names[5] = {"ml", "mr", "mm", "ms1", "ms2"};
static const uint8_t button_masks[5] = {0x01, 0x02, 0x04, 0x08, 0x10};

// Heartbeat tracking
static uint32_t last_heartbeat = 0;
static uint8_t heartbeat_brightness = 0;
static bool heartbeat_direction = true;  // true = fading up, false = fading down

// Performance optimization caches
static uint32_t last_neopixel_color = 0;
static mouse_report_t last_sent_report = {0};

//--------------------------------------------------------------------+
// Utility Functions
//--------------------------------------------------------------------+

static uint32_t get_random_release_time(void)
{
    g_rand_seed = (g_rand_seed * 1103515245 + 12345) & 0x7FFFFFFF;
    uint32_t range = RELEASE_MAX_TIME_MS - RELEASE_MIN_TIME_MS + 1;
    uint32_t random_offset = (g_rand_seed >> 16) % range;
    return RELEASE_MIN_TIME_MS + random_offset;
}

static uint32_t get_random_click_press_time(void)
{
    g_rand_seed = (g_rand_seed * 1103515245 + 12345) & 0x7FFFFFFF;
    uint32_t range = CLICK_PRESS_MAX_TIME_MS - CLICK_PRESS_MIN_TIME_MS + 1;
    uint32_t random_offset = (g_rand_seed >> 16) % range;
    return CLICK_PRESS_MIN_TIME_MS + random_offset;
}

static uint8_t parse_button_name(const char* name)
{
    for (int i = 0; i < 5; i++) {
        if (strcmp(name, button_names[i]) == 0) {
            return i;
        }
    }
    return 255;
}

static uint8_t parse_lock_button_name(const char* name)
{
    for (int i = 0; i < 5; i++) {
        if (strcmp(name, lock_button_names[i]) == 0) {
            return i;
        }
    }
    return 255;
}

//--------------------------------------------------------------------+
// NeoPixel Functions
//--------------------------------------------------------------------+

static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b)
{
    uint8_t region, remainder, p, q, t;

    if (s == 0) {
        *r = v; *g = v; *b = v;
        return;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

static void neopixel_push(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    if (color == last_neopixel_color) return;  // Skip if color unchanged
    
    last_neopixel_color = color;
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

static void update_neopixel_status(uint32_t current_time_ms)
{
    uint8_t r = 0, g = 0, b = 0;
    
    switch (g_state.current_status) {
        case STATUS_BOOT_RED:
            r = 255; g = 0; b = 0;
            break;
            
        case STATUS_BOOT_GREEN:
            r = 0; g = 255; b = 0;
            break;
            
        case STATUS_RAINBOW: {
            uint16_t hue = (current_time_ms / RAINBOW_SPEED) % 256;
            hsv_to_rgb(hue, 255, 128, &r, &g, &b);
            break;
        }
        
        case STATUS_REPORT_ACTIVE: {
            // Use triangle wave instead of sin() for better performance
            uint32_t phase = (current_time_ms / REPORT_ACTIVITY_SPEED) % 512;
            uint8_t intensity;
            if (phase < 256) {
                intensity = phase;
            } else {
                intensity = 511 - phase;
            }
            intensity = intensity / 2;  // Scale to 0-127
            
            r = intensity / 4;
            g = intensity / 4;
            b = intensity;
            break;
        }
    }
    
    neopixel_push(r, g, b);
    g_state.last_status_update = current_time_ms;
}

static void update_heartbeat(uint32_t current_time_ms)
{
    // Update heartbeat LED every 10ms for smooth fading
    if (current_time_ms - last_heartbeat >= 10) {
        uint8_t old_brightness = heartbeat_brightness;
        
        if (heartbeat_direction) {
            // Fading up
            heartbeat_brightness += 3;
            if (heartbeat_brightness >= 255) {
                heartbeat_brightness = 255;
                heartbeat_direction = false;  // Start fading down
            }
        } else {
            // Fading down
            if (heartbeat_brightness <= 3) {
                heartbeat_brightness = 0;
                heartbeat_direction = true;  // Start fading up
            } else {
                heartbeat_brightness -= 3;
            }
        }
        
        // Only write if brightness changed
        if (heartbeat_brightness != old_brightness) {
            analogWrite(LED_BUILTIN, heartbeat_brightness);
        }
        
        last_heartbeat = current_time_ms;
    }
}

//--------------------------------------------------------------------+
// Report Queue Functions
//--------------------------------------------------------------------+

static bool queue_is_empty(const report_queue_t* queue)
{
    return queue->count == 0;
}

static bool queue_is_full(const report_queue_t* queue)
{
    return queue->count >= MAX_QUEUED_REPORTS;
}

static bool queue_push(report_queue_t* queue, const mouse_report_t* report)
{
    if (queue->count >= MAX_QUEUED_REPORTS) {
        return false;
    }
    
    queue->reports[queue->head] = *report;
    queue->head = (queue->head + 1) & QMASK;  // Power of 2 masking
    queue->count++;
    
    return true;
}

static bool queue_pop(report_queue_t* queue, mouse_report_t* report)
{
    if (queue->count == 0) {
        return false;
    }
    
    *report = queue->reports[queue->tail];
    queue->tail = (queue->tail + 1) & QMASK;  // Power of 2 masking
    queue->count--;
    
    return true;
}

//--------------------------------------------------------------------+
// KM Button Management
//--------------------------------------------------------------------+

static void km_update_button_states(uint32_t current_time_ms)
{
    for (int i = 0; i < 5; i++) {
        km_button_state_t* btn = &g_state.km_buttons[i];
        
        // Handle click sequence
        if (btn->is_clicking) {
            if (current_time_ms >= btn->click_end_time) {
                // Click sequence complete
                btn->is_clicking = false;
                btn->is_forced = false;
                btn->click_release_start = 0;
                btn->click_end_time = 0;
                
                // Restore physical button state unless locked
                if (!btn->is_locked) {
                    bool physical_pressed = (g_state.physical_buttons & button_masks[i]) != 0;
                    btn->is_pressed = physical_pressed;
                }
            } else if (current_time_ms >= btn->click_release_start) {
                // In release phase of click
                btn->is_pressed = false;
            }
        }
        // Check if a forced release has expired
        else if (btn->is_forced && !btn->is_pressed && btn->release_time > 0) {
            if (current_time_ms >= btn->release_time) {
                btn->is_forced = false;
                btn->release_time = 0;
                
                if (!btn->is_locked) {
                    bool physical_pressed = (g_state.physical_buttons & button_masks[i]) != 0;
                    btn->is_pressed = physical_pressed;
                }
            }
        }
        // Update non-forced button states from physical state (unless locked)
        else if (!btn->is_forced && !btn->is_locked) {
            bool physical_pressed = (g_state.physical_buttons & button_masks[i]) != 0;
            btn->is_pressed = physical_pressed;
        }
    }
}

static void remapper_km_button_press(uint8_t button_index, uint32_t current_time_ms)
{
    if (button_index >= 5) return;
    
    km_button_state_t* btn = &g_state.km_buttons[button_index];
    btn->is_pressed = true;
    btn->is_forced = true;
    btn->release_time = 0;
    btn->is_clicking = false;
}

static void remapper_km_button_release(uint8_t button_index, uint32_t current_time_ms)
{
    if (button_index >= 5) return;
    
    km_button_state_t* btn = &g_state.km_buttons[button_index];
    if (btn->is_forced && btn->is_pressed) {
        btn->is_pressed = false;
        btn->release_time = current_time_ms + get_random_release_time();
        btn->is_clicking = false;
    }
}

static void remapper_km_button_click(uint8_t button_index, uint32_t current_time_ms)
{
    if (button_index >= 5) return;
    
    km_button_state_t* btn = &g_state.km_buttons[button_index];
    btn->is_clicking = true;
    btn->is_pressed = true;
    btn->is_forced = true;
    
    uint32_t press_duration = get_random_click_press_time();
    uint32_t release_duration = get_random_release_time();
    
    btn->click_release_start = current_time_ms + press_duration;
    btn->click_end_time = btn->click_release_start + release_duration;
    btn->release_time = 0;
}

static void remapper_km_move(int16_t x, int16_t y)
{
    if (!g_state.lock_mx) {
        g_state.km_mouse_x_accumulator += x;
    }
    
    if (!g_state.lock_my) {
        g_state.km_mouse_y_accumulator += y;
    }
}

static void remapper_km_wheel(int8_t wheel_amount)
{
    g_state.km_wheel_accumulator += wheel_amount;
    
    if (g_state.km_wheel_accumulator > 127) {
        g_state.km_wheel_accumulator = 127;
    } else if (g_state.km_wheel_accumulator < -128) {
        g_state.km_wheel_accumulator = -128;
    }
}

//--------------------------------------------------------------------+
// Command Parser
//--------------------------------------------------------------------+

static void parse_command(const char* cmd, uint32_t current_time_ms)
{
    // Echo the command back
    printf("%s%.*s", cmd, g_state.parser.terminator_len, g_state.parser.command_terminator);
    
    // Check if command starts with "km."
    if (strncmp(cmd, "km.", 3) != 0) {
        return; // Silently ignore non-km commands
    }
    
    const char* subcmd = cmd + 3;
    
    // Button commands: km.button_name(state)
    const char* paren_start = strchr(subcmd, '(');
    if (paren_start) {
        const char* paren_end = strchr(paren_start, ')');
        if (paren_end) {
            // Extract button name
            size_t button_name_len = paren_start - subcmd;
            if (button_name_len < 16) {
                char button_name[16];
                strncpy(button_name, subcmd, button_name_len);
                button_name[button_name_len] = '\0';
                
                // Extract state value
                char state_str[8];
                size_t state_len = paren_end - paren_start - 1;
                if (state_len < sizeof(state_str)) {
                    strncpy(state_str, paren_start + 1, state_len);
                    state_str[state_len] = '\0';
                    
                    // Parse button and state
                    uint8_t button = parse_button_name(button_name);
                    if (button < 5) {
                        int state = atoi(state_str);
                        if (state == 0 || state == 1) {
                            if (state == 1) {
                                remapper_km_button_press(button, current_time_ms);
                            } else {
                                remapper_km_button_release(button, current_time_ms);
                            }
                            printf("1\r\n>>> ");
                            return;
                        }
                    }
                }
            }
        }
    }
    
    // Click command: km.click(button_num)
    if (strncmp(subcmd, "click(", 6) == 0) {
        const char* num_start = subcmd + 6;
        char* num_end;
        long button_num = strtol(num_start, &num_end, 10);
        
        if (*num_end == ')' && button_num >= 0 && button_num < 5) {
            remapper_km_button_click((uint8_t)button_num, current_time_ms);
            printf("1\r\n>>> ");
            return;
        }
    }
    
    // Move command: km.move(x, y)
    if (strncmp(subcmd, "move(", 5) == 0) {
        const char* args_start = subcmd + 5;
        const char* comma_pos = strchr(args_start, ',');
        if (comma_pos) {
            const char* paren_end = strchr(comma_pos, ')');
            if (paren_end) {
                // Parse X value
                char x_str[16];
                size_t x_len = comma_pos - args_start;
                if (x_len < sizeof(x_str)) {
                    strncpy(x_str, args_start, x_len);
                    x_str[x_len] = '\0';
                    
                    // Parse Y value
                    const char* y_start = comma_pos + 1;
                    while (*y_start == ' ' || *y_start == '\t') y_start++;
                    
                    char y_str[16];
                    size_t y_len = paren_end - y_start;
                    if (y_len < sizeof(y_str)) {
                        strncpy(y_str, y_start, y_len);
                        y_str[y_len] = '\0';
                        
                        int x_amount = atoi(x_str);
                        int y_amount = atoi(y_str);
                        
                        remapper_km_move((int16_t)x_amount, (int16_t)y_amount);
                        printf(">>> ");
                        return;
                    }
                }
            }
        }
    }
    
    // Wheel command: km.wheel(amount)
    if (strncmp(subcmd, "wheel(", 6) == 0) {
        const char* num_start = subcmd + 6;
        char* num_end;
        long wheel_amount = strtol(num_start, &num_end, 10);
        
        if (*num_end == ')') {
            remapper_km_wheel((int8_t)wheel_amount);
            printf(">>> ");
            return;
        }
    }
    
    // Lock axis commands: km.lock_mx(state), km.lock_my(state)
    if (strncmp(subcmd, "lock_mx(", 8) == 0) {
        const char* arg_start = subcmd + 8;
        const char* paren_end = strchr(arg_start, ')');
        
        if (paren_end) {
            size_t arg_len = paren_end - arg_start;
            if (arg_len == 0) {
                printf("%d\r\n>>> ", g_state.lock_mx ? 1 : 0);
                return;
            }
            
            char state_str[8];
            if (arg_len < sizeof(state_str)) {
                strncpy(state_str, arg_start, arg_len);
                state_str[arg_len] = '\0';
                
                int state = atoi(state_str);
                if (state == 0 || state == 1) {
                    g_state.lock_mx = (state == 1);
                    printf(">>> ");
                    return;
                }
            }
        }
    }
    
    if (strncmp(subcmd, "lock_my(", 8) == 0) {
        const char* arg_start = subcmd + 8;
        const char* paren_end = strchr(arg_start, ')');
        
        if (paren_end) {
            size_t arg_len = paren_end - arg_start;
            if (arg_len == 0) {
                printf("%d\r\n>>> ", g_state.lock_my ? 1 : 0);
                return;
            }
            
            char state_str[8];
            if (arg_len < sizeof(state_str)) {
                strncpy(state_str, arg_start, arg_len);
                state_str[arg_len] = '\0';
                
                int state = atoi(state_str);
                if (state == 0 || state == 1) {
                    g_state.lock_my = (state == 1);
                    printf(">>> ");
                    return;
                }
            }
        }
    }
    
    // Lock button commands: km.lock_ml(state), km.lock_mr(state), etc.
    if (strncmp(subcmd, "lock_", 5) == 0) {
        const char* lock_cmd_start = subcmd + 5;
        const char* paren_start = strchr(lock_cmd_start, '(');
        if (paren_start) {
            const char* paren_end = strchr(paren_start, ')');
            if (paren_end) {
                // Extract button name
                size_t button_name_len = paren_start - lock_cmd_start;
                if (button_name_len < 16) {
                    char button_name[16];
                    strncpy(button_name, lock_cmd_start, button_name_len);
                    button_name[button_name_len] = '\0';
                    
                    uint8_t button = parse_lock_button_name(button_name);
                    if (button < 5) {
                        size_t arg_len = paren_end - paren_start - 1;
                        if (arg_len == 0) {
                            printf("%d\r\n>>> ", g_state.km_buttons[button].is_locked ? 1 : 0);
                            return;
                        }
                        
                        char state_str[8];
                        if (arg_len < sizeof(state_str)) {
                            strncpy(state_str, paren_start + 1, arg_len);
                            state_str[arg_len] = '\0';
                            
                            int state = atoi(state_str);
                            if (state == 0 || state == 1) {
                                g_state.km_buttons[button].is_locked = (state == 1);
                                printf(">>> ");
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Buttons callback command: km.buttons()
    if (strncmp(subcmd, "buttons(", 8) == 0) {
        const char* arg_start = subcmd + 8;
        const char* paren_end = strchr(arg_start, ')');
        
        if (paren_end) {
            size_t arg_len = paren_end - arg_start;
            if (arg_len == 0) {
                // Return current button state
                uint8_t button_state = 0;
                for (int i = 0; i < 5; i++) {
                    if (g_state.km_buttons[i].is_pressed) {
                        button_state |= button_masks[i];
                    }
                }
                printf("km.%c\r\n>>> ", button_state);
                return;
            }
        }
    }
    
    // Default: send prompt for unrecognized commands
    printf(">>> ");
}

static void remapper_process_serial_char(char c, uint32_t current_time_ms)
{
    command_parser_t* parser = &g_state.parser;
    
    // Handle line termination characters
    if (c == '\n' || c == '\r') {
        if (parser->buffer_pos > 0 && !parser->skip_next_terminator) {
            // Store the terminator for this command
            if (c == '\r') {
                parser->command_terminator[0] = '\r';
                parser->terminator_len = 1;
                parser->skip_next_terminator = true;
                parser->last_terminator = '\r';
            } else {
                parser->command_terminator[0] = '\n';
                parser->terminator_len = 1;
            }
            
            // Null terminate and process command
            parser->buffer[parser->buffer_pos] = '\0';
            parse_command(parser->buffer, current_time_ms);
            
            // Reset parser
            parser->buffer_pos = 0;
            parser->in_command = false;
        } else if (parser->skip_next_terminator) {
            // Check if this is part of a \r\n sequence
            if (parser->last_terminator == '\r' && c == '\n') {
                // This is the \n following a \r, update terminator to \r\n
                parser->command_terminator[1] = '\n';
                parser->terminator_len = 2;
                parser->skip_next_terminator = false;
            } else {
                // This is a new line terminator, not part of \r\n
                parser->skip_next_terminator = false;
                if (parser->buffer_pos > 0) {
                    // Store the new terminator
                    if (c == '\r') {
                        parser->command_terminator[0] = '\r';
                        parser->terminator_len = 1;
                        parser->skip_next_terminator = true;
                        parser->last_terminator = '\r';
                    } else {
                        parser->command_terminator[0] = '\n';
                        parser->terminator_len = 1;
                    }
                    
                    parser->buffer[parser->buffer_pos] = '\0';
                    parse_command(parser->buffer, current_time_ms);
                    parser->buffer_pos = 0;
                    parser->in_command = false;
                }
            }
        }
        return;
    }
    
    // Reset skip flag for non-terminator characters
    parser->skip_next_terminator = false;
    
    // Add character to buffer if there's space
    if (parser->buffer_pos < REMAPPER_CMD_BUFFER_SIZE - 1) {
        parser->buffer[parser->buffer_pos++] = c;
        
        // Check if we're starting a command
        if (!parser->in_command && parser->buffer_pos >= 3) {
            if (strncmp(parser->buffer, "km.", 3) == 0) {
                parser->in_command = true;
            }
        }
    } else {
        // Buffer overflow - reset
        parser->buffer_pos = 0;
        parser->in_command = false;
    }
}

//--------------------------------------------------------------------+
// Main Report Processing
//--------------------------------------------------------------------+

static mouse_report_t remapper_process_mouse_report(const mouse_report_t* input_report, uint32_t current_time_ms)
{
    mouse_report_t output_report = *input_report;
    
    // Update physical button state from input
    g_state.physical_buttons = input_report->buttons;
    
    // Check if we have an injected report to send instead
    mouse_report_t injected_report;
    if (queue_pop(&g_state.inject_queue, &injected_report)) {
        output_report = injected_report;
        g_state.reports_injected++;
    } else {
        // Apply KM button states
        uint8_t km_button_state = 0;
        for (int i = 0; i < 5; i++) {
            if (g_state.km_buttons[i].is_pressed) {
                km_button_state |= button_masks[i];
            }
        }
        
        // Use KM button state as the output
        output_report.buttons = km_button_state;
        
        // Add KM movement accumulator to physical movement
        if (g_state.km_mouse_x_accumulator != 0) {
            int16_t new_x = (int16_t)output_report.x + g_state.km_mouse_x_accumulator;
            if (new_x > 127) new_x = 127;
            if (new_x < -128) new_x = -128;
            output_report.x = (int8_t)new_x;
            g_state.km_mouse_x_accumulator = 0;
        }
        
        if (g_state.km_mouse_y_accumulator != 0) {
            int16_t new_y = (int16_t)output_report.y + g_state.km_mouse_y_accumulator;
            if (new_y > 127) new_y = 127;
            if (new_y < -128) new_y = -128;
            output_report.y = (int8_t)new_y;
            g_state.km_mouse_y_accumulator = 0;
        }
        
        if (g_state.km_wheel_accumulator != 0) {
            int16_t new_wheel = (int16_t)output_report.wheel + g_state.km_wheel_accumulator;
            if (new_wheel > 127) new_wheel = 127;
            if (new_wheel < -128) new_wheel = -128;
            output_report.wheel = (int8_t)new_wheel;
            g_state.km_wheel_accumulator = 0;
        }
        
        // Apply axis locks
        if (g_state.lock_mx) {
            output_report.x = 0;
        }
        
        if (g_state.lock_my) {
            output_report.y = 0;
        }
    }
    
    // Update activity tracking
    g_state.reports_processed++;
    g_state.last_report_time = current_time_ms;
    
    // Switch to report active status if we're in rainbow mode
    if (g_state.boot_complete && g_state.current_status == STATUS_RAINBOW) {
        g_state.current_status = STATUS_REPORT_ACTIVE;
        g_state.reports_active = true;
    }
    
    return output_report;
}

static void remapper_update(uint32_t current_time_ms)
{
    // Initialize boot start time on first call
    if (g_state.boot_start_time == 0) {
        g_state.boot_start_time = current_time_ms;
        Serial.println("Remapper update started");
    }
    
    // Handle boot sequence
    if (!g_state.boot_complete) {
        uint32_t elapsed = current_time_ms - g_state.boot_start_time;
        
        if (elapsed < BOOT_PHASE_DURATION_MS && g_state.current_status != STATUS_BOOT_RED) {
            g_state.current_status = STATUS_BOOT_RED;
            Serial.println("Boot phase: RED");
        } else if (elapsed >= BOOT_PHASE_DURATION_MS && elapsed < (BOOT_PHASE_DURATION_MS * 2) && g_state.current_status != STATUS_BOOT_GREEN) {
            g_state.current_status = STATUS_BOOT_GREEN;
            Serial.println("Boot phase: GREEN");
        } else if (elapsed >= (BOOT_PHASE_DURATION_MS * 2)) {
            g_state.current_status = STATUS_RAINBOW;
            g_state.boot_complete = true;
            Serial.println("Boot complete: RAINBOW");
        }
    }
    
    // Check for report activity timeout
    if (g_state.reports_active && g_state.current_status == STATUS_REPORT_ACTIVE) {
        if (current_time_ms - g_state.last_report_time > REPORT_ACTIVITY_TIMEOUT_MS) {
            g_state.reports_active = false;
            g_state.current_status = STATUS_RAINBOW;
        }
    }
    
    // Update KM button states
    km_update_button_states(current_time_ms);
    
    // Update NeoPixel
    update_neopixel_status(current_time_ms);
    
    // Update heartbeat LED
    update_heartbeat(current_time_ms);
}

//--------------------------------------------------------------------+
// Arduino Setup and Loop
//--------------------------------------------------------------------+

void setup() 
{
    // Initialize built-in LED for heartbeat FIRST
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on immediately to show we're alive
    
    // Initialize serial with longer timeout
    Serial.begin(115200);
    delay(100);  // Give serial time to initialize
    
    // Print early debug message
    Serial.println();
    Serial.println("=== MOUSE REMAPPER STARTING ===");
    Serial.println("Initializing components...");
    Serial.flush();
    
    // Initialize NeoPixel early
    Serial.println("Initializing NeoPixel...");
    strip.begin();
    strip.setBrightness(50);
    
    // Set initial red color to show we're starting
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));  // Red
    }
    strip.show();
    Serial.println("NeoPixel initialized (RED)");
    
    // Initialize state with debug output
    Serial.println("Initializing state...");
    memset(&g_state, 0, sizeof(g_state));
    g_state.current_status = STATUS_BOOT_RED;
    g_rand_seed = millis(); // Use current time as seed
    Serial.println("State initialized");
    
    // Initialize USB HID device
    Serial.println("Initializing USB HID device...");
    usb_hid.begin();
    Serial.println("USB HID device initialized");
    
    // Initialize USB host
    Serial.println("Initializing USB host...");
    USBHost.begin(1);
    Serial.println("USB host initialized");
    
    // Print startup messages
    Serial.println();
    Serial.println("=== MOUSE REMAPPER READY ===");
    Serial.println("Mouse Remapper with KM Command Support");
    Serial.println("Commands: km.left(1), km.move(10,5), km.click(0), etc.");
    Serial.println("Boot sequence starting...");
    Serial.println("NeoPixel: RED -> GREEN -> RAINBOW -> ACTIVE");
    Serial.println("Built-in LED: Heartbeat every 500ms");
    Serial.println();
    Serial.println(">>> ");
    Serial.flush();
    
    // Force first heartbeat
    last_heartbeat = millis();
    heartbeat_brightness = 0;
    heartbeat_direction = true;
    analogWrite(LED_BUILTIN, 128);  // Start at medium brightness
    
    Serial.println("Setup complete!");
}

void loop() 
{
    uint32_t current_time = millis();
    
    // Process USB host tasks
    USBHost.task();
    
    // Process serial commands if available
    if (Serial.available()) {
        char c = Serial.read();
        remapper_process_serial_char(c, current_time);
    }
    
    // Update remapper state and NeoPixel (every 10ms)
    static uint32_t last_update = 0;
    if (current_time - last_update >= 10) {
        remapper_update(current_time);
        last_update = current_time;
    }
    
    // Send periodic keep-alive reports if no recent activity (every 16ms for ~60Hz)
    static uint32_t last_report_time = 0;
    if (current_time - last_report_time >= 16) {
        if (usb_hid.ready()) {
            // Send empty report to maintain connection
            mouse_report_t empty_report = {0};
            
            // Check for KM accumulated movement even when no physical mouse input
            if (g_state.km_mouse_x_accumulator != 0 || g_state.km_mouse_y_accumulator != 0 || g_state.km_wheel_accumulator != 0) {
                // Process accumulated KM movements
                mouse_report_t km_report = remapper_process_mouse_report(&empty_report, current_time);
                // Only send if different from last report
                if (memcmp(&km_report, &last_sent_report, sizeof(km_report)) != 0) {
                    usb_hid.sendReport(0, &km_report, sizeof(mouse_report_t));
                    last_sent_report = km_report;
                }
            } else {
                // Send the current KM button state
                uint8_t km_button_state = 0;
                for (int i = 0; i < 5; i++) {
                    if (g_state.km_buttons[i].is_pressed) {
                        km_button_state |= button_masks[i];
                    }
                }
                empty_report.buttons = km_button_state;
                
                // Only send if different from last report
                if (memcmp(&empty_report, &last_sent_report, sizeof(empty_report)) != 0) {
                    usb_hid.sendReport(0, &empty_report, sizeof(mouse_report_t));
                    last_sent_report = empty_report;
                }
            }
            
            last_report_time = current_time;
        }
    }
    
    // Debug output every 5 seconds to show we're alive
    static uint32_t last_debug = 0;
    if (current_time - last_debug >= 5000) {
        Serial.printf("Alive: %lu ms, Status: %d, Reports: %lu\r\n", 
                     current_time, (int)g_state.current_status, g_state.reports_processed);
        last_debug = current_time;
    }
}

//--------------------------------------------------------------------+
// TinyUSB Host Callbacks
//--------------------------------------------------------------------+

extern "C" {

// Invoked when device with hid interface is mounted
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) 
{
    (void) desc_report;
    (void) desc_len;
    
    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    Serial.printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
    Serial.printf("VID = %04x, PID = %04x\r\n", vid, pid);

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
        Serial.printf("HID Mouse detected\r\n");
        if (!tuh_hid_receive_report(dev_addr, instance)) {
            Serial.printf("Error: cannot request to receive report\r\n");
        }
    } else if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
        Serial.printf("HID Keyboard detected\r\n");
        if (!tuh_hid_receive_report(dev_addr, instance)) {
            Serial.printf("Error: cannot request to receive report\r\n");
        }
    } else {
        Serial.printf("HID Generic device detected\r\n");
        if (!tuh_hid_receive_report(dev_addr, instance)) {
            Serial.printf("Error: cannot request to receive report\r\n");
        }
    }
    
    Serial.printf(">>> ");
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) 
{
    Serial.printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
    Serial.printf(">>> ");
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) 
{
    uint32_t current_time = millis();
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE && len >= 3) {
        // Process mouse report
        mouse_report_t input_report;
        input_report.buttons = report[0];
        input_report.x = (len > 1) ? (int8_t)report[1] : 0;
        input_report.y = (len > 2) ? (int8_t)report[2] : 0;
        input_report.wheel = (len > 3) ? (int8_t)report[3] : 0;
        input_report.pan = (len > 4) ? (int8_t)report[4] : 0;
        
        // ALWAYS process and mirror reports regardless of UART connection
        mouse_report_t output_report = remapper_process_mouse_report(&input_report, current_time);
        
        // Send modified report to PC (only if changed)
        if (usb_hid.ready()) {
            // Skip sending if report is identical to last sent
            if (memcmp(&output_report, &last_sent_report, sizeof(output_report)) != 0) {
                usb_hid.sendReport(0, &output_report, sizeof(mouse_report_t));
                last_sent_report = output_report;
            }
        }
        
    } else if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD && len >= 8) {
        // For keyboard reports, just pass through for now
        // Future: could add keyboard remapping functionality
        
    } else {
        // Unknown report format - for debugging
        if (len <= 16) { // Only print short reports to avoid spam
            Serial.printf("Unknown report: dev=%d, inst=%d, len=%d, proto=%d, data=", 
                         dev_addr, instance, len, itf_protocol);
            for (uint16_t i = 0; i < len; i++) {
                Serial.printf("%02x ", report[i]);
            }
            Serial.printf("\r\n>>> ");
        }
    }

    // Continue to request to receive report
    if (!tuh_hid_receive_report(dev_addr, instance)) {
        Serial.printf("Error: cannot request to receive report\r\n>>> ");
    }
}

} // extern "C"

//--------------------------------------------------------------------+
// Usage Examples (printed on startup)
//--------------------------------------------------------------------+

void print_km_usage() 
{
    Serial.println("\nKM Command Examples:");
    Serial.println("km.left(1)               - Press left button");
    Serial.println("km.left(0)               - Release left button");
    Serial.println("km.click(0)              - Click left button (0=left, 1=right, 2=middle)");
    Serial.println("km.move(10,-5)           - Move mouse +10 X, -5 Y");
    Serial.println("km.wheel(1)              - Scroll wheel up");
    Serial.println("km.wheel(-1)             - Scroll wheel down");
    Serial.println("km.lock_mx(1)            - Lock X axis movement");
    Serial.println("km.lock_my(1)            - Lock Y axis movement");
    Serial.println("km.lock_ml(1)            - Lock left button");
    Serial.println("km.lock_mr(0)            - Unlock right button");
    Serial.println("km.buttons()             - Get current button state");
    Serial.println("\nButtons: left, right, middle, side1, side2");
    Serial.println("Locks: ml, mr, mm, ms1, ms2 (mouse lock + button)");
    Serial.println(">>> ");
}
