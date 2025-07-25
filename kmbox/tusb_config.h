#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by compiler flags for flexibility
#ifndef CFG_TUSB_MCU
  #if defined(ARDUINO_ARCH_SAMD)
    #define CFG_TUSB_MCU     OPT_MCU_SAMD51
  #elif defined(ARDUINO_ARCH_RP2040)
    #define CFG_TUSB_MCU     OPT_MCU_RP2040
  #elif defined(ARDUINO_ARCH_ESP32)
    #define CFG_TUSB_MCU     OPT_MCU_ESP32S2
  #else
    #define CFG_TUSB_MCU     OPT_MCU_NONE
  #endif
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS       OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG    0
#endif

// Enable Device stack
#define CFG_TUD_ENABLED       1

// Enable Host stack  
#define CFG_TUH_ENABLED       1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUD_MAX_SPEED     OPT_MODE_DEFAULT_SPEED

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE    64
#endif

//------------- CLASS -------------//
#define CFG_TUD_HID               3  // Multiple HID interfaces: mouse, keyboard, vendor

#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

// HID buffer size Should be sufficient to hold ID (if any) + Data
#define CFG_TUD_HID_EP_BUFSIZE    64

//--------------------------------------------------------------------
// HOST CONFIGURATION
//--------------------------------------------------------------------

// Size of buffer to hold descriptors and other data used for enumeration
#define CFG_TUH_ENUMERATION_BUFSIZE 256

#define CFG_TUH_HUB                 0  // number of supported hubs
#define CFG_TUH_CDC                 0  // CDC ACM
#define CFG_TUH_HID                 4  // typical keyboard + mouse device can have 3-4 HID interfaces
#define CFG_TUH_MSC                 0  // Mass storage
#define CFG_TUH_VENDOR              0  // Vendor (unknown)

// max device support (excluding hub device)
#define CFG_TUH_DEVICE_MAX          (CFG_TUH_HUB ? 4 : 1) // hub typically has 4 ports

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
