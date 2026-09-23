#pragma once

#include <cstdint>


// Flight controller running on Core 1, isolated from the WSPR tracker on Core 0.
//
// Ownership:
// - Core 0 (tracker) owns GPS, I2C, radio, USB, flash, logging, clock changes.
// - Core 1 (flight controller) owns the servo PWM on GPIO10.
//
// Core 1 code must not call picoinf (Log, Evm, Timer, Pin, UART, I2C),
// allocate on the heap, or use std::string / std::function.
//
// Data crosses cores only through the plain structs below, copied under a
// hardware-spinlock critical section.  The multicore FIFO is reserved for the
// flash-safety handshake (FreeRTOS on Core 0 also listens on its own FIFO).


// Latest GPS fix, Core 0 -> Core 1.
// picoinf's Fix3DPlus holds std::string/vector and must not cross cores.
struct FcGpsSample
{
    uint32_t seq       = 0;  // increments on each publish, 0 = never published
    uint64_t rxTimeUs  = 0;  // time_us_64() when Core 0 published it
    int32_t  latE6     = 0;  // degrees * 1e6
    int32_t  lngE6     = 0;  // degrees * 1e6
    int32_t  altM      = 0;
    uint16_t courseDeg = 0;
    uint16_t speedKph  = 0;
};

// Flight controller state, Core 1 -> Core 0.
struct FcStatus
{
    uint32_t ticks              = 0;
    uint32_t gpsSeqUsed         = 0;           // seq of the fix used on the last tick
    uint32_t gpsAgeMs           = UINT32_MAX;  // UINT32_MAX if no fix yet
    int32_t  servoAngleCentiDeg = 0;
};


/////////////////////////////////////////////////////////////////
// Core 0 API
/////////////////////////////////////////////////////////////////

// Call once in main() before FcLaunch() and before the app starts.
void FcInit();

// Start (or restart) Core 1.
void FcLaunch();

// Publish a new GPS fix to Core 1.
void FcPublishGps(const FcGpsSample &sample);

// Manual servo command, used by the default control law.
// Returns false (and changes nothing) for non-finite input.
bool FcSetServoAngle(float angleDeg);

FcStatus FcGetStatus();

// Call after every clk_sys change so the servo pulse width stays exact.
// Core 1 also checks each tick as a backstop.
void FcOnClockChange();

// Call periodically (~2 s) from Core 0.  Restarts Core 1 if it has stalled.
void FcSupervise();

uint32_t FcGetRestartCount();
