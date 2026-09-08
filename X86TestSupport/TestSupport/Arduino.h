#pragma once

#include <cstdint>
#include <cstdio>

#include "esp_err.h"

class SystemRestartException {};

#define IRAM_ATTR

// From Arduino.h:

void delay(int ms);

// Get time in microseconds since boot.
int64_t esp_timer_get_time();

// this_thread::yield?
#define NOP()                                                                                                                              \
    do {                                                                                                                                   \
    } while (0);

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// itoa (non-standard, needed by some ESP32 code that was ported)
inline char* itoa(int value, char* str, int base) {
    if (base == 10) { sprintf(str, "%d", value); }
    else if (base == 16) { sprintf(str, "%x", value); }
    else { sprintf(str, "%d", value); }
    return str;
}

// ESP...

#include "Esp.h"
