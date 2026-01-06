/*
  time_keeper_test

  Turns an LED on for one second, then off for one second, repeatedly.
  Uses a timer interrupt for delays.
*/

#include <stdint.h>
#include <stdbool.h>
#include <pdk/device.h>
#include "auto_sysclock.h"
#include "startup.h"
#include "time_keeper.h"
#include "serial.h"

// --- Hardware Configuration ---
// Change this bit to match your hardware (e.g., 0 for PA0, 4 for PA4)
#define BUTTON_BIT          5       
#define BUTTON_PORT_IN      PA      // Read from Port A
#define BUTTON_PORT_C       PAC     // Control Register for Port A
#define BUTTON_PORT_PH      PAP     // Pull-High Register for Port A

// --- Timing Constants (uint16_t safe) ---
#define INTERVAL_NORMAL     250     
#define INTERVAL_SLOW       1000
#define DEBOUNCE_DELAY      50

// --- State Definitions ---
typedef enum {
    STATE_NORMAL,
    STATE_SLOW,
    STATE_POWER_DOWN
} SystemState;

volatile SystemState current_state = STATE_NORMAL;
bool button_handled = false;

// --- Helper Functions ---

// Define thresholds
const uint8_t SOC_THRESHOLDS[] = {50, 102, 152, 203, 255};

uint8_t adc_to_soc(uint8_t adc_value) {
    for (uint8_t i = 0; i < 5; i++) {
        if (adc_value <= SOC_THRESHOLDS[i]) {
            return (i + 1);
        }
    }
    return 6;
}

// --- Interrupt Service Routine ---
void interrupt(void) __interrupt(0) {
  if (INTRQ & INTRQ_T16) {        // T16 interrupt request?
    INTRQ &= ~INTRQ_T16;          // Mark T16 interrupt request processed
    time_keeper_irq_handler();
  }
  if (INTRQ & INTRQ_TM2) {        // TM2 interrupt request?
    INTRQ &= ~INTRQ_TM2;          // Mark TM2 interrupt request processed
    serial_irq_handler();         // Process next Serial Bit
  }
  
}

/*
 * Checks if button is pressed (Active Low: Input is 1).
 * Returns true only on a NEW press (handles holding button down).
 */
bool check_button_trigger(void) {
    // Check if bit is LOW (Button Pressed to GND)
    bool is_pressed = (BUTTON_PORT_IN & (1 << BUTTON_BIT));

    if (is_pressed) { 
        if (!button_handled) {
            button_handled = true;
            return true; // Valid new trigger
        }
    } else {
        button_handled = false; // Reset flag when button is released
    }
    return false;
}


/*
 * Waits for a specific time, but exits immediately if button is pressed.
 * Returns: true if interrupted by button, false if timeout completed.
 * Optimized: Uses uint16_t to save RAM.
 */
bool wait_interruptible(uint16_t duration) {
    time_keeper_set_timeout(duration);
    while (!time_keeper_has_timed_out()) {
        if (check_button_trigger()) {
            return true; 
        }
    }
    return false;
}

/*
 * Advances the state machine to the next state
 */
void transition_state(void) {
    if (current_state == STATE_NORMAL) {
        current_state = STATE_SLOW;
        serial_println("State: SLOW");
    } 
    else if (current_state == STATE_SLOW) {
        current_state = STATE_POWER_DOWN;
        serial_println("State: PWR DOWN");
    } 
    else {
        current_state = STATE_NORMAL;
        serial_println("State: NORMAL");
    }
}

/*
 * Main blinking logic.
 * Returns true if execution was interrupted by a button press.
 */
bool run_blink_sequence(uint16_t interval) {
    // Pulse count placeholder (Swap with ADC logic if needed)
    uint8_t pulse_count = 10; 
    
    // 1. Stabilization Wait
    if (wait_interruptible(interval)) return true;

    serial_println("2"); // Start Marker

    // 2. Pulse Loop
    while (pulse_count > 0) {
        // Toggle Logic (Placeholder text, add LED PIN toggle here)
        serial_println("T"); 
        
        if (wait_interruptible(interval)) return true;
        
        pulse_count--;
        // __asm__("wdreset"); // Optional: Reset Watchdog
    }

    // 3. Rest period (5x interval)
    serial_println("Resting");
    if (wait_interruptible(5 * interval)) return true;
    
    return false; 
}


// --- Main ---
void main(void) {
  // 1. Hardware Setup
  
  // Setup Button Pin: Input Mode
  BUTTON_PORT_C &= ~(1 << BUTTON_BIT); 
  
  // Initialize Systems
  time_keeper_setup();
  serial_setup();                 
  INTRQ = 0;
  __engint();                     // Enable global interrupts
  
  serial_println("Init Complete");

  // 2. Main Loop
  while (1) {
    // A. Check Button State (High Priority)
    if (check_button_trigger()) {
        transition_state();
        // Simple debounce wait
        //time_keeper_set_timeout(DEBOUNCE_DELAY);
        //while(!time_keeper_has_timed_out());
    }

    // B. Run State Logic
    switch (current_state) {
        case STATE_NORMAL:
            // If function returns true (interrupted), 'continue' forces loop to top
            // to handle the button press immediately.
            if (run_blink_sequence(INTERVAL_NORMAL)) continue;
            break;

        case STATE_SLOW:
            if (run_blink_sequence(INTERVAL_SLOW)) continue;
            break;

        case STATE_POWER_DOWN:
            // In power down, we just wait.
            // Ideally, you would use 'stopsys' here to sleep the CPU,
            // but we need the timer running to wake up for button checks 
            // unless you set up a Pin Change Interrupt.
            // For now, we poll slowly (every 100ms) to save "effort".
            if (wait_interruptible(100)) continue;
            break;
    }
  }
}
// Startup code - Setup/calibrate system clock
unsigned char STARTUP_FUNCTION(void) {

  // Initialize the system clock (CLKMD register) with the IHRC, ILRC, or EOSC clock source and correct divider.
  // The AUTO_INIT_SYSCLOCK() macro uses F_CPU (defined in the Makefile) to choose the IHRC or ILRC clock source and divider.
  // Alternatively, replace this with the more specific PDK_SET_SYSCLOCK(...) macro from pdk/sysclock.h
  AUTO_INIT_SYSCLOCK();
  //PDK_SET_SYSCLOCK(SYSCLOCK_IHRC_1MHZ);
  
  // Insert placeholder code to tell EasyPdkProg to calibrate the IHRC or ILRC internal oscillator.
  // The AUTO_CALIBRATE_SYSCLOCK(...) macro uses F_CPU (defined in the Makefile) to choose the IHRC or ILRC oscillator.
  // Alternatively, replace this with the more specific EASY_PDK_CALIBRATE_IHRC(...) or EASY_PDK_CALIBRATE_ILRC(...) macro from easy-pdk/calibrate.h
  AUTO_CALIBRATE_SYSCLOCK(TARGET_VDD_MV);
  //EASY_PDK_CALIBRATE_ILRC(16000000,4900);
  return 0;   // Return 0 to inform SDCC to continue with normal initialization.
}
