#ifndef __TIME_KEEPER_H__
#define __TIME_KEEPER_H__
#include <stdbool.h>
#include <stdint.h>

// --- Configuration ---
// Sysclock 8MHz, T16 divider 1, 8-bit overflow = 256 ticks
// 8MHz / 1 / 256 = 31250 Hz => 32 us per interrupt
#define ISR_INTERVAL_US  32

volatile uint32_t _millis_counter = 0; // Countdown timer
volatile uint8_t _accumulator = 0;    // decrements every 32 us

void time_keeper_setup(void) {
  // Setup timer16 (T16) interrupt to tick every 32 uS
  // Note: This assumes the internal IHRC oscillator is running and has been calibrated to the normal 16MHz and not some weird frequency.
  // The system clock can still be set to use a divider, or to use ILRC or EOSC (as long as IHRC is still enabled).
  T16M = (uint8_t)(T16M_CLK_SYSCLK | T16M_CLK_DIV1 | T16M_INTSRC_8BIT);
  T16C = 0;
  INTEN |= INTEN_T16;
}

void time_keeper_set_timeout(uint32_t ms) {
  INTEN &= ~INTEN_T16;                          // Disable T16 (read of 32 bit value _millis is non-atomic)
  _millis_counter = ms;                         // Set the new timeout
  INTEN |= INTEN_T16;                           // Re-enable T16
}

bool time_keeper_has_timed_out(void) {
    bool is_timeout = false;

    // 1. Disable interrupt to ensure atomic read of _millis
    INTEN &= ~INTEN_T16;
    
    // 2. Perform the check (using == for comparison)
    if (_millis_counter == 0) {
        is_timeout = true;
    }

    // 3. RE-ENABLE INTERRUPT (Now reachable!)
    INTEN |= INTEN_T16; 

    // 4. Return the result
    return is_timeout;
}

// Interrupt Service Routine
void time_keeper_irq_handler(void) {
    
    if (_accumulator++ == (uint8_t)(1000/ISR_INTERVAL_US)) {  // 1000/32 is actually 31.25, so we are off a bit here (0.8%)
    _accumulator = 0;
    if (_millis_counter != 0) {
        --_millis_counter;
    }
  }
}

#endif //__TIME_KEEPER_H__
