#ifndef DEBUG_GLOBALS_H
#define DEBUG_GLOBALS_H

#include "config.h"

//=============================================================================
// RUNTIME DEBUG FLAGS
//=============================================================================
// These global variables can be toggled at runtime using the "debug" command
// in the serial monitor. They start with default values from config.h but
// can be changed during operation without recompiling.
//
// Usage in code:
//   if (g_debug_serial) {
//     Serial.println("Debug message");
//   }
//=============================================================================

// Global debug flags (can be modified at runtime)
extern bool g_debug_serial;      // Enable general debug output
extern bool g_debug_motor;       // Enable detailed motor debug output
extern bool g_debug_comm;        // Enable communication debug output
extern bool g_debug_heartbeat;   // Enable periodic heartbeat messages

// Initialize with default values (call this in setup())
inline void initDebugFlags() {
    extern bool g_debug_serial;
    extern bool g_debug_motor;
    extern bool g_debug_comm;
    extern bool g_debug_heartbeat;

    g_debug_serial = DEBUG_SERIAL_DEFAULT;
    g_debug_motor = DEBUG_MOTOR_DEFAULT;
    g_debug_comm = DEBUG_COMM_DEFAULT;
    g_debug_heartbeat = DEBUG_HEARTBEAT_DEFAULT;
}

#endif // DEBUG_GLOBALS_H
