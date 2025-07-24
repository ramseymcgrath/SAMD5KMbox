/*********************************************************************
 Mouse Remapper with TinyUSB Host/Device and KM Command Compatibility
 
 This firmware acts as a USB mouse remapper that:
 - Receives mouse reports from a connected USB mouse (host mode)
 - Mirrors all reports to PC regardless of UART connection (device mode)
 - Provides km. serial1 command interface for button control and movement
 - Uses NeoPixel for status indication (red->green->rainbow->active)
 
 Serial1 Commands (km. format):
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
#define LED_BUILTIN PIN_LED
#endif

// USBHost configuration
#define CFG_TUH_MAX3421 1
#include "tusb_config.h"
#include "usbh_helper.h"
#include "Adafruit_TinyUSB.h"
#include <Adafruit_NeoPixel.h>  // Use regular NeoPixel library instead of ZeroDMA
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Ensure constrain function is available
#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

//--------------------------------------------------------------------+
// Constants and Configuration
//--------------------------------------------------------------------+

#define REMAPPER_CMD_BUFFER_SIZE 128
#define MAX_QUEUED_REPORTS 16  // Power of 2 for faster masking
#define QMASK (MAX_QUEUED_REPORTS - 1)
#define BOOT_PHASE_DURATION_MS 2000
#define REPORT_ACTIVITY_TIMEOUT_MS 500
#define RAINBOW_SPEED 50
#define REPORT_ACTIVITY_SPEED 25
#define RELEASE_MIN_TIME_MS 125
#define RELEASE_MAX_TIME_MS 175
#define CLICK_PRESS_MIN_TIME_MS 75
#define CLICK_PRESS_MAX_TIME_MS 125
#define USB_DEVICE_ID_RAZER_BASILISK_V3 0x0099
#define USB_DEVICE_NAME "Razer Basilisk V3"
#define USB_DEVICE_KEYBOARD_NAME "Razer Basilisk V3 Keyboard"
#define USB_DEVICE_MOUSE_NAME "Razer Basilisk V3 Mouse"
#define USB_DEVICE_MFR 1
#define USB_DEVICE_PRODUCT 2
#define USB_DEVICE_SERIAL 0
#define USB_DEVICE_POLLING_INTERVAL 1
#define USB_VENDOR_ID_RAZER 0x1532
#define RAZER_USB_REPORT_LEN 0x5A
#define RAZER_MANUFACT "Razer"
#define USB_DEVICE_BCD 0x0200

// Common keyboard HID scancodes
#define HID_KEY_A 0x04
#define HID_KEY_B 0x05
#define HID_KEY_C 0x06
#define HID_KEY_ENTER 0x28
#define HID_KEY_ESCAPE 0x29
#define HID_KEY_SPACE 0x2C
#define HID_KEY_F1 0x3A
#define HID_KEY_F2 0x3B

// Keyboard modifiers
#define HID_MOD_LEFT_CTRL 0x01
#define HID_MOD_LEFT_SHIFT 0x02
#define HID_MOD_LEFT_ALT 0x04
#define HID_MOD_LEFT_GUI 0x08
#define HID_MOD_RIGHT_CTRL 0x10
#define HID_MOD_RIGHT_SHIFT 0x20
#define HID_MOD_RIGHT_ALT 0x40
#define HID_MOD_RIGHT_GUI 0x80



// Debug control - comment out to disable debug output for better performance
// #define DEBUG_VERBOSE

#ifndef DEBUG_VERBOSE
#define VLOG(...)
#else
#define VLOG(...) Serial1.printf(__VA_ARGS__)
#endif

// Debug control - comment out to disable USB debug spam
// #define DEBUG_USB

#ifndef DEBUG_USB
#define DLOG(...)
#else
#define DLOG(...) Serial1.printf(__VA_ARGS__)
#endif

// Debug control - uncomment to enable button state debugging
// #define DEBUG_BUTTONS

#ifndef DEBUG_BUTTONS
#define BLOG(...)
#else
#define BLOG(...) Serial1.printf(__VA_ARGS__)
#endif

//--------------------------------------------------------------------+
// Forward Declarations
//--------------------------------------------------------------------+

static void handle_button_command(const char* subcmd, uint32_t current_time_ms);
static void handle_click_command(const char* subcmd, uint32_t current_time_ms);
static void handle_move_command(const char* subcmd, uint32_t current_time_ms);
static void handle_wheel_command(const char* subcmd, uint32_t current_time_ms);
static void handle_lock_command(const char* subcmd, uint32_t current_time_ms);
static void handle_buttons_command(const char* subcmd, uint32_t current_time_ms);
static void handle_key_command(const char* subcmd, uint32_t current_time_ms);
static void handle_vendor_command(const char* subcmd, uint32_t current_time_ms);

//--------------------------------------------------------------------+
// Data Structures
//--------------------------------------------------------------------+

// Legacy 8-bit mouse report (kept for compatibility if needed)
typedef struct {
  uint8_t buttons;
  int8_t x;
  int8_t y;
  int8_t wheel;
  int8_t pan;
} mouse_report8_t;

// New 16-bit mouse report for full precision pipeline
// This preserves the full resolution from high-end mice like the Razer Basilisk V3
typedef struct {
  uint8_t buttons;
  int16_t x;        // Full 16-bit precision - no premature clamping
  int16_t y;        // Full 16-bit precision - no premature clamping
  int8_t wheel;     // Wheel is still 8-bit (adequate resolution)
  int8_t pan;       // Pan/horizontal wheel is still 8-bit
} mouse_report16_t;

static_assert(sizeof(mouse_report8_t) == 5, "mouse_report8_t must be 5 bytes");
static_assert(sizeof(mouse_report16_t) == 8, "mouse_report16_t must be 8 bytes");

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
  mouse_report16_t reports[MAX_QUEUED_REPORTS];
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
  // KM movement accumulators - use 32-bit for better precision with rapid movements
  int32_t km_mouse_x_accumulator;
  int32_t km_mouse_y_accumulator;
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

static remapper_state_t g_state = { 0 };
static uint32_t g_rand_seed = 0x12345678;

typedef struct __attribute__((packed)) {
  uint8_t  buttons;
  uint8_t  pad;
  int8_t   wheel;
  int8_t   pan;
  int16_t  x;
  int16_t  y;
} razer_mouse_rpt_t;

static_assert(sizeof(razer_mouse_rpt_t) == 8, "Razer mouse rpt must be 8 bytes");


// NeoPixel strip
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Button names and masks
static const char* button_names[5] = { "left", "right", "middle", "side1", "side2" };
static const char* lock_button_names[5] = { "ml", "mr", "mm", "ms1", "ms2" };
static const uint8_t button_masks[5] = { 0x01, 0x02, 0x04, 0x08, 0x10 };

// Heartbeat tracking
static uint32_t last_heartbeat = 0;
static uint8_t heartbeat_brightness = 0;
static bool heartbeat_direction = true;  // true = fading up, false = fading down

// Performance optimization caches
static uint32_t last_neopixel_color = 0;

// usb report states
static volatile bool in_flight = false;  // host has an IN txn outstanding
static razer_mouse_rpt_t last_sent_report = { 0 };

// Physical state validation
static bool     phys_valid = false;
static uint32_t last_phys_ts = 0;
static bool     sent_initial_zero = false;

// tunables
#define PHYS_STALE_MS          40   // after this, ignore cached physical buttons
#define KEEPALIVE_PERIOD_MS    16   // ~60 Hz

// Pending report FIFO to prevent dropped releases
#define PENDING_DEPTH 32  // Increased from 8 for better buffering during high-rate KM moves
typedef struct {
  razer_mouse_rpt_t buf[PENDING_DEPTH];
  volatile uint8_t head;  // write
  volatile uint8_t tail;  // read
} rpt_fifo_t;

static rpt_fifo_t pending_fifo = { 0 };



//--------------------------------------------------------------------+
// Separate HID Descriptors for Multiple Interfaces
//--------------------------------------------------------------------+

// Mouse descriptor
static const uint8_t desc_hid_mouse[] = {
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
  0x09, 0x01, 0xA1, 0x00,
  0x05, 0x09, 0x19, 0x01, 0x29, 0x05,
  0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x05, 0x81, 0x02,
  0x75, 0x01, 0x95, 0x0B, 0x81, 0x03,
  0x05, 0x0C, 0x0A, 0x38, 0x02, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
  0x05, 0x01, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
  0x09, 0x30, 0x09, 0x31, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x02, 0x81, 0x06,
  0xC0, 0xC0
};

// Keyboard descriptor
static const uint8_t desc_hid_keyboard[] = {
  0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00,
  0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x19, 0x00, 0x2A, 0xFF, 0x00, 0x15, 0x00, 0x26,
  0xFF, 0x00, 0x75, 0x08, 0x95, 0x0E, 0x81, 0x00, 0xC0, 0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01,
  0x19, 0x00, 0x2A, 0x3C, 0x02, 0x15, 0x00, 0x26, 0x3C, 0x02, 0x95, 0x01, 0x75, 0x10, 0x81,
  0x00, 0x75, 0x08, 0x95, 0x0D, 0x81, 0x01, 0xC0, 0x05, 0x01, 0x09, 0x80, 0xA1, 0x01,
  0x19, 0x81, 0x29, 0x83, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03, 0x81, 0x02, 0x95, 0x05,
  0x81, 0x01, 0x75, 0x08, 0x95, 0x0E, 0x81, 0x01, 0xC0, 0x05, 0x01, 0x09, 0x00, 0xA1, 0x01,
  0x09, 0x03, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x35, 0x00, 0x46, 0xFF, 0x00, 0x75, 0x08, 0x95,
  0x0F, 0x81, 0x00, 0xC0, 0x05, 0x01, 0x09, 0x00, 0xA1, 0x01, 0x09, 0x03, 0x15, 0x00,
  0x26, 0xFF, 0x00, 0x35, 0x00, 0x46, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x0F, 0x81, 0x00, 0xC0
};

// Vendor descriptor
static const uint8_t desc_hid_vendor[] = {
  0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01,
  0x06, 0x00, 0xFF, 0x09, 0x02, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x08, 0x95, 0x5A, 0xB1, 0x01, 0xC0
};

// Separate HID instances
Adafruit_USBD_HID hid_mouse(desc_hid_mouse, sizeof(desc_hid_mouse),
                            HID_ITF_PROTOCOL_MOUSE, 1, false);
Adafruit_USBD_HID hid_keyboard(desc_hid_keyboard, sizeof(desc_hid_keyboard),
                               HID_ITF_PROTOCOL_KEYBOARD, 1, false);
Adafruit_USBD_HID hid_vendor(desc_hid_vendor, sizeof(desc_hid_vendor),
                             HID_ITF_PROTOCOL_NONE, 1, true);


// -------------------------------------------------------------------+
// Mouse utilities
// -------------------------------------------------------------------+

// Mouse precision architecture:
// 1. PARSE: Device reports are decoded into mouse_report16_t with full precision
//    - 8-byte Razer reports: preserve native 16-bit X/Y coordinates
//    - 3-4 byte boot reports: sign-extend 8-bit to 16-bit coordinates  
// 2. MUTATE: KM commands accumulate in 32-bit precision, applied to 16-bit coords
// 3. EMIT: Final 16-bit coordinates are packed directly into Razer HID report
//    - No precision loss from clamping to 8-bit then re-expanding to 16-bit
//    - Full pixel-perfect movement from device and KM commands

// Helper to parse different mouse report formats
static bool parse_mouse_report(uint8_t const* rpt, uint16_t len, mouse_report16_t& out) {
  memset(&out, 0, sizeof(out));
  if (len == 8) {
    // Razer / 16-bit X-Y format
    // Buttons   Pad   Wheel  Pan    X LSB  X MSB  Y LSB  Y MSB
    // byte 0   byte1 byte2 byte3   byte4  byte5  byte6  byte7
    out.buttons = rpt[0];
    out.wheel   = (int8_t)rpt[2];
    out.pan     = (int8_t)rpt[3];
    out.x = (int16_t)(rpt[4] | (rpt[5] << 8));
    out.y = (int16_t)(rpt[6] | (rpt[7] << 8));
    return true;
  } else if (len >= 4) {
    // Classic 4-byte boot mouse - sign-extend to 16-bit
    out.buttons = rpt[0];
    out.x = (int8_t)rpt[1];  // sign-extend to 16-bit
    out.y = (int8_t)rpt[2];
    out.wheel = (int8_t)rpt[3];
    out.pan = 0;
    return true;
  } else if (len >= 3) {
    // 3-byte fallback - sign-extend to 16-bit
    out.buttons = rpt[0];
    out.x = (int8_t)rpt[1];
    out.y = (int8_t)rpt[2];
    out.wheel = 0;
    out.pan = 0;
    return true;
  }
  return false;  // Unrecognized format
}

static inline razer_mouse_rpt_t pack_razer(const mouse_report16_t* s) {
  razer_mouse_rpt_t r;
  r.buttons = s->buttons;
  r.pad     = 0;
  r.wheel   = s->wheel;
  r.pan     = s->pan;  // Preserve pan instead of forcing to 0 (can be mapped to KM commands later)
  r.x       = s->x;    // Direct 16-bit assignment - no clamping needed
  r.y       = s->y;    // Direct 16-bit assignment - no clamping needed
  return r;
}

// FIFO operations for pending reports
static inline bool fifo_push(rpt_fifo_t* f, const razer_mouse_rpt_t* r) {
  uint8_t next = (f->head + 1) & (PENDING_DEPTH - 1);
  if (next == f->tail) return false;  // full
  memcpy((void*)&f->buf[f->head], r, sizeof(*r));
  f->head = next;
  return true;
}

static inline bool fifo_pop(rpt_fifo_t* f, razer_mouse_rpt_t* r) {
  if (f->tail == f->head) return false;  // empty
  memcpy(r, (void*)&f->buf[f->tail], sizeof(*r));
  f->tail = (f->tail + 1) & (PENDING_DEPTH - 1);
  return true;
}

static inline void queue_pending(const razer_mouse_rpt_t* r) {
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  fifo_push((rpt_fifo_t*)&pending_fifo, r);
  __set_PRIMASK(prim);
}

static inline void send_mouse_report(const razer_mouse_rpt_t* r) {
  if (!hid_mouse.ready()) return;
  if (memcmp(r, &last_sent_report, sizeof(*r)) == 0) return;

  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  bool busy = in_flight;
  __set_PRIMASK(prim);

  if (!busy) {
    if (hid_mouse.sendReport(0, r, sizeof(*r))) {  // Report ID 0 for single interface
      last_sent_report = *r;
      prim = __get_PRIMASK();
      __disable_irq();
      in_flight = true;
      __set_PRIMASK(prim);
      
      // Accumulators are cleared in tud_hid_report_complete_cb() after USB transaction completes
    } else {
      // sendReport failed - queue for retry
      queue_pending(r);
    }
  } else {
    queue_pending(r);
  }
}

// Send keyboard report
static inline void send_keyboard_report(const keyboard_report_t* report) {
  if (!hid_keyboard.ready()) return;
  hid_keyboard.sendReport(0, report, sizeof(*report));
}

// Send vendor report (90 bytes for Razer compatibility)
static inline void send_vendor_report(const uint8_t* data, uint16_t len) {
  if (!hid_vendor.ready()) return;
  if (len > RAZER_USB_REPORT_LEN) len = RAZER_USB_REPORT_LEN;
  hid_vendor.sendReport(0, data, len);
}

// Helper functions for keyboard operations
static inline void send_key_press(uint8_t keycode) {
  keyboard_report_t report = { 0 };
  report.keycode[0] = keycode;
  send_keyboard_report(&report);
}

static inline void send_key_release() {
  keyboard_report_t report = { 0 };
  send_keyboard_report(&report);
}

static inline void send_modifier_press(uint8_t modifier) {
  keyboard_report_t report = { 0 };
  report.modifier = modifier;
  send_keyboard_report(&report);
}

// Helper to clear KM movement accumulators
static inline void clear_km_movement_accumulators(void) {
  g_state.km_mouse_x_accumulator = 0;
  g_state.km_mouse_y_accumulator = 0;
  g_state.km_wheel_accumulator = 0;
}

// ------------------
// Other utilities
//------------------



// -------------------------------------------------------------------+
// KM utilities
// -------------------------------------------------------------------+

// Fast inline random number generation for better performance
static inline uint32_t fast_random(void) {
  g_rand_seed = g_rand_seed * 1664525UL + 1013904223UL;  // Faster LCG constants
  return g_rand_seed;
}

static uint32_t get_random_release_time(void) {
  uint32_t range = RELEASE_MAX_TIME_MS - RELEASE_MIN_TIME_MS + 1;
  uint32_t random_offset = (fast_random() >> 16) % range;
  return RELEASE_MIN_TIME_MS + random_offset;
}

static uint32_t get_random_click_press_time(void) {
  uint32_t range = CLICK_PRESS_MAX_TIME_MS - CLICK_PRESS_MIN_TIME_MS + 1;
  uint32_t random_offset = (fast_random() >> 16) % range;
  return CLICK_PRESS_MIN_TIME_MS + random_offset;
}

// Fast lookup using first character + length for button names
static uint8_t parse_button_name(const char* name) {
  char first_char = name[0];
  size_t len = strlen(name);

  switch (first_char) {
    case 'l':  // "left"
      return (len == 4 && name[1] == 'e' && name[2] == 'f' && name[3] == 't') ? 0 : 255;
    case 'r':  // "right"
      return (len == 5 && memcmp(name, "right", 5) == 0) ? 1 : 255;
    case 'm':  // "middle"
      return (len == 6 && memcmp(name, "middle", 6) == 0) ? 2 : 255;
    case 's':  // "side1", "side2"
      if (len == 5 && memcmp(name, "side", 4) == 0) {
        return (name[4] == '1') ? 3 : (name[4] == '2') ? 4
                                                       : 255;
      }
      return 255;
    default:
      return 255;
  }
}

static uint8_t parse_lock_button_name(const char* name) {
  size_t len = strlen(name);

  switch (len) {
    case 2:  // "ml", "mr", "mm"
      if (name[0] == 'm') {
        switch (name[1]) {
          case 'l': return 0;
          case 'r': return 1;
          case 'm': return 2;
        }
      }
      return 255;
    case 3:  // "ms1", "ms2"
      return (name[0] == 'm' && name[1] == 's') ? ((name[2] == '1') ? 3 : (name[2] == '2') ? 4
                                                                                           : 255)
                                                : 255;
    default:
      return 255;
  }
}

//--------------------------------------------------------------------+
// NeoPixel Functions
//--------------------------------------------------------------------+

// Fast HSV to RGB conversion optimized for rainbow animation
static void hsv_to_rgb_fast(uint8_t h, uint8_t s, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b) {
  if (s == 0) {
    *r = v;
    *g = v;
    *b = v;
    return;
  }

  // Use 6 regions for faster computation
  uint8_t region = h / 43;  // 256/6 ≈ 43
  uint8_t remainder = h % 43;

  // Pre-calculate common values
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  // Use jump table approach for better performance
  switch (region) {
    case 0:
      *r = v;
      *g = t;
      *b = p;
      break;
    case 1:
      *r = q;
      *g = v;
      *b = p;
      break;
    case 2:
      *r = p;
      *g = v;
      *b = t;
      break;
    case 3:
      *r = p;
      *g = q;
      *b = v;
      break;
    case 4:
      *r = t;
      *g = p;
      *b = v;
      break;
    default:
      *r = v;
      *g = p;
      *b = q;
      break;
  }
}

static void neopixel_push(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  
  // Always update during boot phase to ensure visibility
  bool force_update = !g_state.boot_complete;
  
  if (!force_update && color == last_neopixel_color) return;  // Skip if color unchanged

  last_neopixel_color = color;
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

static void update_neopixel_status(uint32_t current_time_ms) {
  // During boot, update more frequently to ensure visibility
  static uint32_t last_neopixel_update = 0;
  uint32_t update_interval = g_state.boot_complete ? 33 : 10;  // 10ms during boot, 33ms after
  
  if (current_time_ms - last_neopixel_update < update_interval) return;

  uint8_t r = 0, g = 0, b = 0;

  switch (g_state.current_status) {
    case STATUS_BOOT_RED:
      r = 255;
      g = 0;
      b = 0;
      break;

    case STATUS_BOOT_GREEN:
      r = 0;
      g = 255;
      b = 0;
      break;

    case STATUS_RAINBOW:
      {
        uint8_t hue = (current_time_ms / RAINBOW_SPEED) & 0xFF;
        hsv_to_rgb_fast(hue, 255, 128, &r, &g, &b);
        break;
      }

    case STATUS_REPORT_ACTIVE:
      {
        uint32_t phase = (current_time_ms / REPORT_ACTIVITY_SPEED) % 512;
        uint8_t intensity;
        if (phase < 256) {
          intensity = phase;
        } else {
          intensity = 511 - phase;
        }
        intensity = intensity / 2;

        r = intensity / 4;
        g = intensity / 4;
        b = intensity;
        break;
      }
  }

  neopixel_push(r, g, b);
  g_state.last_status_update = current_time_ms;
  last_neopixel_update = current_time_ms;
}

static void update_heartbeat(uint32_t current_time_ms) {
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

static bool queue_is_empty(const report_queue_t* queue) {
  return queue->count == 0;
}

static bool queue_is_full(const report_queue_t* queue) {
  return queue->count >= MAX_QUEUED_REPORTS;
}

static bool queue_push(report_queue_t* queue, const mouse_report16_t* report) {
  if (queue->count >= MAX_QUEUED_REPORTS) {
    return false;
  }

  queue->reports[queue->head] = *report;
  queue->head = (queue->head + 1) & QMASK;  // Power of 2 masking
  queue->count++;

  return true;
}

// IRQ-safe version for contexts that can be preempted
static bool queue_push_isr_safe(report_queue_t* queue, const mouse_report16_t* report) {
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  bool ok = queue_push(queue, report);
  __set_PRIMASK(prim);
  return ok;
}

// Keep only the IRQ-safe one, maybe rename it:
static inline bool queue_pop_ts(report_queue_t* q, mouse_report16_t* r) {
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  bool ok = (q->count != 0);
  if (ok) {
    *r = q->reports[q->tail];
    q->tail = (q->tail + 1) & QMASK;
    q->count--;
  }
  __set_PRIMASK(prim);
  return ok;
}


//--------------------------------------------------------------------+
// KM Button Management
//--------------------------------------------------------------------+

static void km_update_button_states(uint32_t current_time_ms) {
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
        BLOG("KM: Click complete btn%d\r\n", i);

        // If not locked, button state will be handled by physical state in report processing
      } else if (current_time_ms >= btn->click_release_start) {
        // In release phase of click
        btn->is_pressed = false;
        BLOG("KM: Click release phase btn%d\r\n", i);
      }
    }
    // Check if a forced release has expired
    else if (btn->is_forced && !btn->is_pressed && btn->release_time > 0) {
      if (current_time_ms >= btn->release_time) {
        btn->is_forced = false;
        btn->release_time = 0;
        BLOG("KM: Unforce btn%d\r\n", i);

        // If not locked, button state will be handled by physical state in report processing
      }
    }
  }
}

static void remapper_km_button_press(uint8_t button_index, uint32_t current_time_ms) {
  if (button_index >= 5) return;

  km_button_state_t* btn = &g_state.km_buttons[button_index];
  btn->is_pressed = true;
  btn->is_forced = true;
  btn->release_time = 0;
  btn->is_clicking = false;
  BLOG("KM: Press btn%d\r\n", button_index);
}

static void remapper_km_button_release(uint8_t button_index, uint32_t current_time_ms) {
  if (button_index >= 5) return;

  km_button_state_t* btn = &g_state.km_buttons[button_index];
  if (btn->is_forced && btn->is_pressed) {
    btn->is_pressed = false;
    btn->release_time = current_time_ms + get_random_release_time();
    btn->is_clicking = false;
    BLOG("KM: Release btn%d (will unforce at %lu)\r\n", button_index, btn->release_time);
  }
}

static void remapper_km_button_click(uint8_t button_index, uint32_t current_time_ms) {
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

static void remapper_km_move(int16_t x, int16_t y) {
  if (!g_state.lock_mx) {
    g_state.km_mouse_x_accumulator += x;
  }

  if (!g_state.lock_my) {
    g_state.km_mouse_y_accumulator += y;
  }
}

static void remapper_km_wheel(int8_t wheel_amount) {
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

static void parse_command(const char* cmd, uint32_t current_time_ms) {
  // Echo the command back
  Serial1.printf("%s%.*s", cmd, g_state.parser.terminator_len, g_state.parser.command_terminator);

  // Quick check: must start with "km." (early return)
  if (cmd[0] != 'k' || cmd[1] != 'm' || cmd[2] != '.') {
    return;  // Silently ignore non-km commands
  }

  const char* subcmd = cmd + 3;
  char first_char = subcmd[0];

  // Fast command routing based on first character
  switch (first_char) {
    case 'l':  // left(), lock_*()
      if (subcmd[1] == 'e') {
        // left() command
        handle_button_command(subcmd, current_time_ms);
      } else if (subcmd[1] == 'o' && subcmd[2] == 'c' && subcmd[3] == 'k' && subcmd[4] == '_') {
        // lock_*() commands
        handle_lock_command(subcmd, current_time_ms);
      } else {
        Serial1.printf(">>> ");
      }
      break;

    case 'r':  // right()
    case 'm':  // middle(), move()
      if (first_char == 'm' && subcmd[1] == 'o' && subcmd[2] == 'v' && subcmd[3] == 'e') {
        handle_move_command(subcmd, current_time_ms);
      } else {
        handle_button_command(subcmd, current_time_ms);
      }
      break;

    case 's':  // side1(), side2()
      handle_button_command(subcmd, current_time_ms);
      break;

    case 'c':  // click()
      if (subcmd[1] == 'l' && subcmd[2] == 'i' && subcmd[3] == 'c' && subcmd[4] == 'k') {
        handle_click_command(subcmd, current_time_ms);
      } else {
        Serial1.printf(">>> ");
      }
      break;

    case 'w':  // wheel()
      if (subcmd[1] == 'h' && subcmd[2] == 'e' && subcmd[3] == 'e' && subcmd[4] == 'l') {
        handle_wheel_command(subcmd, current_time_ms);
      } else {
        Serial1.printf(">>> ");
      }
      break;

    case 'b':  // buttons()
      if (memcmp(subcmd, "buttons(", 8) == 0) {
        handle_buttons_command(subcmd, current_time_ms);
      } else {
        Serial1.printf(">>> ");
      }
      break;

    case 'k':  // key()
      if (subcmd[1] == 'e' && subcmd[2] == 'y') {
        handle_key_command(subcmd, current_time_ms);
      } else {
        Serial1.printf(">>> ");
      }
      break;

    case 'v':  // vendor()
      if (memcmp(subcmd, "vendor(", 7) == 0) {
        handle_vendor_command(subcmd, current_time_ms);
      } else {
        Serial1.printf(">>> ");
      }
      break;

    default:
      // Default: send prompt for unrecognized commands
      Serial1.printf(">>> ");
      break;
  }
}

static void handle_button_command(const char* subcmd, uint32_t current_time_ms) {
  // Button commands: km.button_name(state)
  const char* paren_start = strchr(subcmd, '(');
  if (paren_start) {
    const char* paren_end = strchr(paren_start, ')');
    if (paren_end) {
      // Extract button name (avoid strncpy)
      size_t button_name_len = paren_start - subcmd;
      if (button_name_len < 16) {
        char button_name[16];
        memcpy(button_name, subcmd, button_name_len);
        button_name[button_name_len] = '\0';

        // Extract state value (avoid strncpy)
        size_t state_len = paren_end - paren_start - 1;
        if (state_len < 8) {
          char state_str[8];
          memcpy(state_str, paren_start + 1, state_len);
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
              Serial1.printf("1\r\n>>> ");
              return;
            }
          }
        }
      }
    }
  }
  Serial1.printf(">>> ");
}

static void handle_click_command(const char* subcmd, uint32_t current_time_ms) {
  // Click command: km.click(button_num)
  if (strncmp(subcmd, "click(", 6) == 0) {
    const char* num_start = subcmd + 6;
    char* num_end;
    long button_num = strtol(num_start, &num_end, 10);

    if (*num_end == ')' && button_num >= 0 && button_num < 5) {
      remapper_km_button_click((uint8_t)button_num, current_time_ms);
      Serial1.printf("1\r\n>>> ");
      return;
    }
  }
  Serial1.printf(">>> ");
}

static void handle_move_command(const char* subcmd, uint32_t current_time_ms) {
  // Move command: km.move(x, y)
  if (strncmp(subcmd, "move(", 5) == 0) {
    const char* args_start = subcmd + 5;
    const char* comma_pos = strchr(args_start, ',');
    if (comma_pos) {
      const char* paren_end = strchr(comma_pos, ')');
      if (paren_end) {
        // Parse X value (avoid strncpy)
        size_t x_len = comma_pos - args_start;
        if (x_len < 16) {
          char x_str[16];
          memcpy(x_str, args_start, x_len);
          x_str[x_len] = '\0';

          // Parse Y value (skip whitespace manually)
          const char* y_start = comma_pos + 1;
          while (*y_start == ' ' || *y_start == '\t') y_start++;

          size_t y_len = paren_end - y_start;
          if (y_len < 16) {
            char y_str[16];
            memcpy(y_str, y_start, y_len);
            y_str[y_len] = '\0';

            int x_amount = atoi(x_str);
            int y_amount = atoi(y_str);

            remapper_km_move((int16_t)x_amount, (int16_t)y_amount);
            Serial1.printf(">>> ");
            return;
          }
        }
      }
    }
  }
  Serial1.printf(">>> ");
}

static void handle_wheel_command(const char* subcmd, uint32_t current_time_ms) {
  // Wheel command: km.wheel(amount)
  if (strncmp(subcmd, "wheel(", 6) == 0) {
    const char* num_start = subcmd + 6;
    char* num_end;
    long wheel_amount = strtol(num_start, &num_end, 10);

    if (*num_end == ')') {
      remapper_km_wheel((int8_t)wheel_amount);
      Serial1.printf(">>> ");
      return;
    }
  }
  Serial1.printf(">>> ");
}

static void handle_lock_command(const char* subcmd, uint32_t current_time_ms) {
  // Lock axis commands: km.lock_mx(state), km.lock_my(state)
  if (strncmp(subcmd, "lock_mx(", 8) == 0) {
    const char* arg_start = subcmd + 8;
    const char* paren_end = strchr(arg_start, ')');

    if (paren_end) {
      size_t arg_len = paren_end - arg_start;
      if (arg_len == 0) {
        Serial1.printf("%d\r\n>>> ", g_state.lock_mx ? 1 : 0);
        return;
      }

      if (arg_len < 8) {
        char state_str[8];
        memcpy(state_str, arg_start, arg_len);
        state_str[arg_len] = '\0';

        int state = atoi(state_str);
        if (state == 0 || state == 1) {
          g_state.lock_mx = (state == 1);
          Serial1.printf(">>> ");
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
        Serial1.printf("%d\r\n>>> ", g_state.lock_my ? 1 : 0);
        return;
      }

      if (arg_len < 8) {
        char state_str[8];
        memcpy(state_str, arg_start, arg_len);
        state_str[arg_len] = '\0';

        int state = atoi(state_str);
        if (state == 0 || state == 1) {
          g_state.lock_my = (state == 1);
          Serial1.printf(">>> ");
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
        // Extract button name (avoid strncpy)
        size_t button_name_len = paren_start - lock_cmd_start;
        if (button_name_len < 16) {
          char button_name[16];
          memcpy(button_name, lock_cmd_start, button_name_len);
          button_name[button_name_len] = '\0';

          uint8_t button = parse_lock_button_name(button_name);
          if (button < 5) {
            size_t arg_len = paren_end - paren_start - 1;
            if (arg_len == 0) {
              Serial1.printf("%d\r\n>>> ", g_state.km_buttons[button].is_locked ? 1 : 0);
              return;
            }

            if (arg_len < 8) {
              char state_str[8];
              memcpy(state_str, paren_start + 1, arg_len);
              state_str[arg_len] = '\0';

              int state = atoi(state_str);
              if (state == 0 || state == 1) {
                g_state.km_buttons[button].is_locked = (state == 1);
                Serial1.printf(">>> ");
                return;
              }
            }
          }
        }
      }
    }
  }
  Serial1.printf(">>> ");
}

static void handle_buttons_command(const char* subcmd, uint32_t current_time_ms) {
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
        Serial1.printf("km.%u\r\n>>> ", button_state);
        return;
      }
    }
  }
  Serial1.printf(">>> ");
}

// Handle keyboard commands: km.key(scancode) or km.key(modifier, scancode)
static void handle_key_command(const char* subcmd, uint32_t current_time_ms) {
  (void)current_time_ms;  // Unused parameter

  if (strncmp(subcmd, "key(", 4) == 0) {
    const char* arg_start = subcmd + 4;
    const char* paren_end = strchr(arg_start, ')');

    if (paren_end) {
      // Parse arguments - could be key(scancode) or key(modifier, scancode)
      char args_buffer[32];
      size_t arg_len = paren_end - arg_start;
      if (arg_len > 0 && arg_len < sizeof(args_buffer)) {
        memcpy(args_buffer, arg_start, arg_len);
        args_buffer[arg_len] = '\0';

        // Look for comma to determine format
        char* comma = strchr(args_buffer, ',');
        if (comma) {
          // Two arguments: modifier, scancode
          *comma = '\0';
          uint8_t modifier = (uint8_t)atoi(args_buffer);
          uint8_t scancode = (uint8_t)atoi(comma + 1);

          keyboard_report_t report = { 0 };
          report.modifier = modifier;
          report.keycode[0] = scancode;
          send_keyboard_report(&report);

          Serial1.printf("km.key sent: mod=0x%02X, scan=0x%02X\r\n>>> ", modifier, scancode);
        } else {
          // Single argument: scancode only
          uint8_t scancode = (uint8_t)atoi(args_buffer);

          keyboard_report_t report = { 0 };
          report.keycode[0] = scancode;
          send_keyboard_report(&report);

          Serial1.printf("km.key sent: scan=0x%02X\r\n>>> ", scancode);
        }
        return;
      }
    }
  }
  Serial1.printf(">>> ");
}

// Handle vendor commands: km.vendor(data_bytes...)
static void handle_vendor_command(const char* subcmd, uint32_t current_time_ms) {
  (void)current_time_ms;  // Unused parameter

  if (strncmp(subcmd, "vendor(", 7) == 0) {
    const char* arg_start = subcmd + 7;
    const char* paren_end = strchr(arg_start, ')');

    if (paren_end) {
      // Parse hex bytes separated by commas
      uint8_t vendor_data[RAZER_USB_REPORT_LEN] = { 0 };
      uint16_t data_len = 0;

      char args_buffer[256];
      size_t arg_len = paren_end - arg_start;
      if (arg_len > 0 && arg_len < sizeof(args_buffer)) {
        memcpy(args_buffer, arg_start, arg_len);
        args_buffer[arg_len] = '\0';

        // Parse comma-separated hex values
        char* token = strtok(args_buffer, ",");
        while (token && data_len < RAZER_USB_REPORT_LEN) {
          // Skip whitespace
          while (*token == ' ') token++;

          // Parse hex value (0x prefix optional)
          uint32_t value;
          if (token[0] == '0' && (token[1] == 'x' || token[1] == 'X')) {
            value = strtoul(token + 2, NULL, 16);
          } else {
            value = strtoul(token, NULL, 16);
          }

          vendor_data[data_len++] = (uint8_t)(value & 0xFF);
          token = strtok(NULL, ",");
        }

        if (data_len > 0) {
          send_vendor_report(vendor_data, data_len);
          Serial1.printf("km.vendor sent %u bytes\r\n>>> ", data_len);
          return;
        }
      }
    }
  }
  Serial1.printf(">>> ");
}

static void remapper_process_serial1_char(char c, uint32_t current_time_ms) {
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

static mouse_report16_t remapper_process_mouse_report(const mouse_report16_t* input_report, uint32_t current_time_ms) {
  mouse_report16_t output_report = *input_report;

  // Update physical button state from input
  g_state.physical_buttons = input_report->buttons;
  phys_valid = true;
  last_phys_ts = current_time_ms;

  // Check if we have an injected report to send instead
  mouse_report16_t injected_report;
  if (queue_pop_ts(&g_state.inject_queue, &injected_report)) {
    output_report = injected_report;
    g_state.reports_injected++;
  } else {
    // Combine physical and KM button states properly
    uint8_t final_button_state = 0;
    for (int i = 0; i < 5; i++) {
      bool should_be_pressed = false;

      if (g_state.km_buttons[i].is_locked) {
        should_be_pressed = g_state.km_buttons[i].is_pressed;
      } else if (g_state.km_buttons[i].is_forced) {
        should_be_pressed = g_state.km_buttons[i].is_pressed;
      } else {
        should_be_pressed = (g_state.physical_buttons & button_masks[i]) != 0;
      }

      if (should_be_pressed) {
        final_button_state |= button_masks[i];
      }
    }

    output_report.buttons = final_button_state;

    // Handle movement - ALWAYS add KM movement to physical movement in full 16-bit precision
    int32_t final_x = (int32_t)input_report->x + g_state.km_mouse_x_accumulator;
    int32_t final_y = (int32_t)input_report->y + g_state.km_mouse_y_accumulator;
    int16_t final_wheel = (int16_t)input_report->wheel + g_state.km_wheel_accumulator;

    // Clamp wheel to int8_t range (wheel is still 8-bit)
    if (final_wheel > 127) final_wheel = 127;
    if (final_wheel < -128) final_wheel = -128;

    // Apply axis locks ONLY to the final result
    if (g_state.lock_mx) {
      final_x = input_report->x;  // Keep only physical X, ignore KM
    }
    if (g_state.lock_my) {
      final_y = input_report->y;  // Keep only physical Y, ignore KM  
    }

    // Clamp X/Y to int16_t range for output
    if (final_x > 32767) final_x = 32767;
    if (final_x < -32768) final_x = -32768;
    if (final_y > 32767) final_y = 32767;
    if (final_y < -32768) final_y = -32768;

    output_report.x = (int16_t)final_x;
    output_report.y = (int16_t)final_y;
    output_report.wheel = (int8_t)final_wheel;
    output_report.pan = input_report->pan;  // Preserve pan instead of zeroing
  }

  // DO NOT CLEAR ACCUMULATORS HERE - they need to persist until USB send succeeds

  // Update activity tracking
  g_state.reports_processed++;
  g_state.last_report_time = current_time_ms;

  if (g_state.boot_complete && g_state.current_status == STATUS_RAINBOW) {
    g_state.current_status = STATUS_REPORT_ACTIVE;
    g_state.reports_active = true;
  }

  return output_report;
}

// Helper to clear KM movement accumulators (now inline above)

static void remapper_update(uint32_t current_time_ms) {
  // Don't re-initialize boot start time if already set
  if (g_state.boot_start_time == 0) {
    g_state.boot_start_time = current_time_ms;
    Serial1.println("Remapper update started");
  }

  // Handle boot sequence
  if (!g_state.boot_complete) {
    uint32_t elapsed = current_time_ms - g_state.boot_start_time;
    neopixel_status_t new_status = g_state.current_status;

    if (elapsed < BOOT_PHASE_DURATION_MS) {
      new_status = STATUS_BOOT_RED;
    } else if (elapsed < (BOOT_PHASE_DURATION_MS * 2)) {
      new_status = STATUS_BOOT_GREEN;
    } else {
      new_status = STATUS_RAINBOW;
      g_state.boot_complete = true;
      Serial1.println("Boot complete: RAINBOW");
    }

    // Only print debug message when status actually changes
    if (new_status != g_state.current_status) {
      g_state.current_status = new_status;
      switch (new_status) {
        case STATUS_BOOT_RED:
          Serial1.println("Boot phase: RED");
          break;
        case STATUS_BOOT_GREEN:
          Serial1.println("Boot phase: GREEN");
          break;
        case STATUS_RAINBOW:
          Serial1.println("Boot phase: RAINBOW");
          break;
        default:
          break;
      }
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

void setup() {
  // Initialize built-in LED for heartbeat FIRST
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn on immediately to show we're alive

  // Print early debug message
  Serial1.begin(115200);
  delay(100);

  Serial1.println();
  Serial1.println("=== MOUSE REMAPPER STARTING ===");

  // Initialize NeoPixel early
  strip.begin();
  strip.setBrightness(100);  // Increase brightness to ensure visibility
  strip.clear();
  strip.show();
  delay(50);  // Small delay after clear

  // Force immediate red display for boot indication
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
  }
  strip.show();
  delay(100);  // Hold red for visibility

  // Initialize state with debug output
  memset(&g_state, 0, sizeof(g_state));
  
  // CRITICAL: Explicitly ensure axis locks are disabled
  g_state.lock_mx = false;
  g_state.lock_my = false;
  
  phys_valid = false;
  sent_initial_zero = false;
  last_phys_ts = 0;
  g_state.current_status = STATUS_BOOT_RED;
  g_state.boot_start_time = millis();
  g_rand_seed = millis();

  // Force immediate NeoPixel update with direct call
  neopixel_push(255, 0, 0);  // Bright red to show we're alive

  // Configure USB device identity
  Serial1.println("Configuring USB device...");
  TinyUSBDevice.setID(USB_VENDOR_ID_RAZER, USB_DEVICE_ID_RAZER_BASILISK_V3);
  TinyUSBDevice.setManufacturerDescriptor(RAZER_MANUFACT);
  TinyUSBDevice.setProductDescriptor(USB_DEVICE_NAME);
  TinyUSBDevice.setSerialDescriptor(USB_DEVICE_SERIAL);
  TinyUSBDevice.setVersion(USB_DEVICE_BCD);

  // Initialize multiple HID interfaces
  Serial1.println("Initializing HID interfaces...");

  // Configure and start mouse HID
  hid_mouse.setPollInterval(USB_DEVICE_POLLING_INTERVAL);
  hid_mouse.setStringDescriptor(USB_DEVICE_MOUSE_NAME);
  hid_mouse.begin();
  Serial1.println("Mouse HID interface initialized");

  // Configure and start keyboard HID
  hid_keyboard.setPollInterval(USB_DEVICE_POLLING_INTERVAL);
  hid_keyboard.setStringDescriptor(USB_DEVICE_KEYBOARD_NAME);
  hid_keyboard.begin();
  Serial1.println("Keyboard HID interface initialized");

  // Configure and start vendor HID
  hid_vendor.setPollInterval(USB_DEVICE_POLLING_INTERVAL);
  hid_vendor.setStringDescriptor(USB_DEVICE_NAME);
  hid_vendor.begin();
  Serial1.println("Vendor HID interface initialized");

  // Force re-enum if already mounted
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial1.println("All USB HID devices initialized");

  // Initialize USB host
  Serial1.println("Initializing USB host...");
  USBHost.begin(1);
  Serial1.println("USB host initialized");

  // Print startup messages
  Serial1.println();
  Serial1.println("=== MOUSE REMAPPER READY ===");
  Serial1.println("Mouse Remapper with Multiple HID Interface Support");
  Serial1.println("Commands: km.left(1), km.move(10,5), km.key(0x04), km.vendor(0x00,0x01)");
  Serial1.println("HID Interfaces: Mouse, Keyboard, Vendor (separate USB devices)");
  Serial1.println("Boot sequence starting...");
  Serial1.println("NeoPixel: RED -> GREEN -> RAINBOW -> ACTIVE");
  Serial1.println("Built-in LED: Heartbeat every 500ms");
  Serial1.flush();

  // Force first heartbeat
  last_heartbeat = millis();
  heartbeat_brightness = 0;
  heartbeat_direction = true;
  analogWrite(LED_BUILTIN, 128);  // Start at medium brightness

  Serial1.println("Setup complete!");
}

void loop() {
  uint32_t current_time = millis();

  USBHost.task();

  if (Serial1.available()) {
    char c = Serial1.read();
    remapper_process_serial1_char(c, current_time);
  }

  static uint32_t last_update = 0;
  static uint32_t last_debug = 0;

  // CRITICAL FIX: Immediately send any pending KM movement
  bool has_pending_movement = (g_state.km_mouse_x_accumulator != 0 || 
                              g_state.km_mouse_y_accumulator != 0 || 
                              g_state.km_wheel_accumulator != 0);
  
  // Check in_flight status atomically
  uint32_t prim = __get_PRIMASK();
  __disable_irq();
  bool busy = in_flight;
  __set_PRIMASK(prim);
  
  if (has_pending_movement && hid_mouse.ready() && !busy) {
    // Build movement-only report
    mouse_report16_t km_report = { 0 };
    
    // Use cached physical buttons if available
    if (phys_valid && (current_time - last_phys_ts) <= PHYS_STALE_MS) {
      uint8_t final_button_state = 0;
      for (int i = 0; i < 5; i++) {
        bool should_be_pressed = false;

        if (g_state.km_buttons[i].is_locked) {
          should_be_pressed = g_state.km_buttons[i].is_pressed;
        } else if (g_state.km_buttons[i].is_forced) {
          should_be_pressed = g_state.km_buttons[i].is_pressed;
        } else {
          should_be_pressed = (g_state.physical_buttons & button_masks[i]) != 0;
        }

        if (should_be_pressed) {
          final_button_state |= button_masks[i];
        }
      }
      km_report.buttons = final_button_state;
    }

    // Add accumulated movement - use 32-bit precision for accumulation
    int32_t final_x = g_state.km_mouse_x_accumulator;
    int32_t final_y = g_state.km_mouse_y_accumulator;
    int16_t final_wheel = g_state.km_wheel_accumulator;

    // Only clamp wheel (still 8-bit)
    if (final_wheel > 127) final_wheel = 127;
    if (final_wheel < -128) final_wheel = -128;

    // Apply axis locks
    if (g_state.lock_mx) final_x = 0;
    if (g_state.lock_my) final_y = 0;

    // Clamp X/Y to int16_t range for output
    if (final_x > 32767) final_x = 32767;
    if (final_x < -32768) final_x = -32768;
    if (final_y > 32767) final_y = 32767;
    if (final_y < -32768) final_y = -32768;

    km_report.x = (int16_t)final_x;
    km_report.y = (int16_t)final_y;
    km_report.wheel = (int8_t)final_wheel;
    km_report.pan = 0;

    // Send immediately
    razer_mouse_rpt_t out = pack_razer(&km_report);
    send_mouse_report(&out);
  }

  // REMOVED: All keepalive zero reports that were interfering with movement

  bool should_update = !g_state.boot_complete || (current_time - last_update >= 10);
  if (should_update) {
    remapper_update(current_time);
    last_update = current_time;
  }

  if (current_time - last_debug >= 5000) {
    uint8_t fifo_depth = (pending_fifo.head - pending_fifo.tail) & (PENDING_DEPTH - 1);
    VLOG("Alive: %lu ms, Status: %d, BootComplete: %d, Reports: %lu, Injected: %lu, Pending: %d, InFlight: %d\r\n",
         current_time, (int)g_state.current_status, g_state.boot_complete ? 1 : 0, g_state.reports_processed,
         g_state.reports_injected, fifo_depth, in_flight ? 1 : 0);
    last_debug = current_time;
  }
}

//--------------------------------------------------------------------+
// TinyUSB Host Callbacks
//--------------------------------------------------------------------+

extern "C" {

  // Invoked when device with hid interface is mounted
  void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    (void)desc_report;
    (void)desc_len;

    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
      tuh_hid_receive_report(dev_addr, instance);
    } else if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) {
      tuh_hid_receive_report(dev_addr, instance);
    } else {
      tuh_hid_receive_report(dev_addr, instance);
    }
  }

  // Invoked when device with hid interface is un-mounted
  void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
    Serial1.printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
    Serial1.printf(">>> ");
  }

  // Invoked when received report from device via interrupt endpoint
  void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    uint32_t current_time = millis();
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE && len >= 3) {
      // Process mouse report
      mouse_report16_t input_report;
      if (!parse_mouse_report(report, len, input_report)) {
        // Unrecognized mouse report format, ignore
        return;
      }

      // Mark physical state as valid on first real packet
      phys_valid = true;
      last_phys_ts = current_time;

      // ALWAYS process and mirror reports regardless of UART connection
      mouse_report16_t output_report = remapper_process_mouse_report(&input_report, current_time);

      // Send modified report to PC (only if changed)
      if (hid_mouse.ready()) {
        // Build the packet first, then compare
        razer_mouse_rpt_t out = pack_razer(&output_report);
        if (memcmp(&out, &last_sent_report, sizeof(out)) != 0) {
          send_mouse_report(&out);
        }
      }

    } else if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD && len >= 8) {
      // For keyboard reports, just pass through for now
      // Future: could add keyboard remapping functionality
    }

    // Continue to request to receive report
    tuh_hid_receive_report(dev_addr, instance);
  }

  void tud_hid_report_complete_cb(uint8_t itf,
                                  uint8_t const* report, uint16_t len) {
    (void)itf;
    (void)report;
    (void)len;
    in_flight = false;
    
    // Clear accumulators when USB transaction completes successfully
    clear_km_movement_accumulators();
    
    razer_mouse_rpt_t next;
    if (fifo_pop(&pending_fifo, &next)) {
      if (hid_mouse.sendReport(0, &next, sizeof(next))) {
        last_sent_report = next;
        in_flight = true;
      }
    }
  }
}
// extern "C"

  //--------------------------------------------------------------------+
  // Usage Examples (printed on startup)
  //--------------------------------------------------------------------+

  void print_km_usage() {
    Serial1.println("\nKM Command Examples:");
    Serial1.println("km.left(1)               - Press left button");
    Serial1.println("km.left(0)               - Release left button");
    Serial1.println("km.click(0)              - Click left button (0=left, 1=right, 2=middle)");
    Serial1.println("km.move(10,-5)           - Move mouse +10 X, -5 Y");
    Serial1.println("km.wheel(1)              - Scroll wheel up");
    Serial1.println("km.wheel(-1)             - Scroll wheel down");
    Serial1.println("km.lock_mx(1)            - Lock X axis movement");
    Serial1.println("km.lock_my(1)            - Lock Y axis movement");
    Serial1.println("km.lock_ml(1)            - Lock left button");
    Serial1.println("km.lock_mr(0)            - Unlock right button");
    Serial1.println("km.buttons()             - Get current button state");
    Serial1.println("km.key(0x04)             - Send keyboard scancode (0x04 = 'A')");
    Serial1.println("km.key(0x02, 0x04)      - Send modifier + scancode (Shift+A)");
    Serial1.println("km.vendor(0x00,0x01,0x02) - Send vendor data bytes");
    Serial1.println("\nButtons: left, right, middle, side1, side2");
    Serial1.println("Locks: ml, mr, mm, ms1, ms2 (mouse lock + button)");
    Serial1.println("HID Interfaces: Mouse, Keyboard, Vendor (separate devices)");
    Serial1.println(">>> ");
  }