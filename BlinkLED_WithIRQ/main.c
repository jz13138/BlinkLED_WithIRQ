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
#include "delay.h"

// --- Hardware Configuration ---
// Button on PA5, high active with external pull down and debounceing
#define BUTTON_BIT          5       
#define BUTTON_PORT         PA      // Data Register of Port A
#define BUTTON_PORT_C       PAC     // Control Register for Port A
#define BUTTON_PORT_DIER    PADIER  // Digital Input Enable Register, default on, will be disabled to save power

// FAN on PA4 high active
#define FAN_BIT          4       
#define FAN_PORT         PA      // Data Register of Port A
#define FAN_PORT_C       PAC     // Control Register for Port A
#define FAN_PORT_DIER    PADIER  // Digital Input Enable Register, default on, will be disabled to save power

#define turnFanOn()      FAN_PORT |= (1 << FAN_BIT)
#define turnFanOff()     FAN_PORT &= ~(1 << FAN_BIT)
#define toggleFan()      FAN_PORT ^= (1 << FAN_BIT)

// LED on PA6 low active
#define LED_BIT          6       
#define LED_PORT         PA      // Data Register of Port A
#define LED_PORT_C       PAC     // Control Register for Port AS
#define LED_PORT_DIER    PADIER  // Digital Input Enable Register, default on, will be disabled to save power

#define turnLedOn()      LED_PORT &= ~(1 << LED_BIT)
#define turnLedOff()     LED_PORT |= (1 << LED_BIT)
#define toggleLed()      LED_PORT ^= (1 << LED_BIT)

// ADC for Battery Voltage on PA3
#define ADC_BIT          3       
#define ADC_PORT         PA      // Data Register of Port A
#define ADC_PORT_C       PAC     // Control Register for Port A
#define ADC_PORT_PH      PAPH    // Digital Input Pull-High Register, default off, optional: will be disabled
#define ADC_PORT_DIER    PADIER  // Digital Input Enable Register, default on, will be disabled to reduce leakage current

// --- Timing Constants (uint16_t safe) ---
#define INTERVAL_NORMAL     100     
#define INTERVAL_SLOW       300

// --- State Definitions ---
typedef enum {
    STATE_NORMAL,
    STATE_SLOW,
    STATE_POWER_DOWN
} SystemState;

volatile SystemState current_state = STATE_NORMAL;
volatile bool button_handled = false;

// --- Global Variables ---

// --- Helper Functions ---

void adc_init(void) {

  // PA3
  ADC_PORT_C &= ~(1 << ADC_BIT);
  ADC_PORT_PH &= ~(1 << ADC_BIT);
  ADC_PORT_DIER &= ~(1 << ADC_BIT);
  ADCC  &= ~ADCC_ADC_ENABLE;

  // Configure the AD conversion clock by adcm register
  // System Clock is 0.9 MHz
  // Target ADC Conversion Clock 2 us
  ADCM |= ADCM_CLK_SYSCLK_DIV8;

  #ifdef ADCRGC
  ADCRGC = ADCRGC_ADC_REF_VDD;     //ADC reference voltage is VDD
  #endif

}

uint8_t adc_read_PA3(void){
  // Clear Channel selection bits 5-2
  ADCC &= 0xc3;
  
  // Select Channel AD8 (PA3) and Enable ADC module
  ADCC |= (ADCC_CH_AD8_PA3 | ADCC_ADC_ENABLE);
  // Short delay for sampling capacitor to charge (Hardware requirement)
  __nop();
  
  // Enable the ADC module by adcc register

  // Start conversion
  ADCC |= ADCC_START_ADC_CONV; 
   
  while( !(ADCC & ADCC_IS_ADC_CONV_READY));
  
  // Disable the ADC module by adcc register
  //ADCC &= ~ADCC_ADC_ENABLE;
  uint8_t res = ADCR;
  
  return res;
}

// Define thresholds
//const uint8_t SOC_THRESHOLDS[] = {160, 184, 207, 231, 254};
__code const uint8_t SOC_THRESHOLDS[] = {160, 184, 207, 231, 254};
uint8_t adc_to_soc(uint8_t adc_value) {
    for (uint8_t i = 0; i < 5; i++) {
        if (adc_value <= SOC_THRESHOLDS[i]) {
            return (i + 1);
        }
    }
    return 5;
}

// --- Interrupt Service Routine ---
void interrupt(void) __interrupt(0) {
  if (INTRQ & INTRQ_T16) {        // T16 interrupt request?
    INTRQ &= ~INTRQ_T16;          // Mark T16 interrupt request processed
    time_keeper_irq_handler();
  } 
}

/*
 * Checks if button is pressed (Active High: Input is 1).
 * Returns true only on a NEW press (handles holding button down).
 */
bool check_button_trigger(void) {
    //bool is_pressed = ((BUTTON_PORT & (1 << BUTTON_BIT)) != 0);    
    bool is_pressed = !!(BUTTON_PORT & (1 << BUTTON_BIT));
    
    if (is_pressed) { 
        if (!button_handled) {
            button_handled = true;
            // Optional: Small delay here for hardware debouncing
            _delay_ms(10); 
            return true; 
        }
    } else {
        // Only reset the flag when the user lets go of the button
        button_handled = false; 
    }
    return false;
}
/*
 * Advances the state machine to the next state
 */
void transition_state(void) {
    turnLedOff();  // Turn LED off first
    SystemState state = current_state; 
    
    switch(state) {
        case STATE_NORMAL:
            current_state = STATE_SLOW;
            break;
        case STATE_SLOW:
            current_state = STATE_POWER_DOWN;
            break;
        default:
            current_state = STATE_NORMAL;
            break;
    }
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
            transition_state();
            return true; // Exit the current blink sequence immediately
        }
    }
    return false;
}
/*
 * Main blinking logic.
 * Returns true if execution was interrupted by a button press.
 */
bool run_blink_sequence(uint16_t interval) {
    // Pulse count placeholder (Swap with ADC logic if needed)
    
    uint8_t pulse_count;
    
    pulse_count = adc_read_PA3();
    pulse_count = 2*adc_to_soc(pulse_count);
    
    // 1. Stabilization Wait
    if (wait_interruptible(interval)) return true;

    // 2. Pulse Loop
    while (pulse_count > 0) {
        // Toggle Logic (Placeholder text, add LED PIN toggle here)
        toggleLed(); 
        toggleFan();
        if (wait_interruptible(interval)) return true;
        
        pulse_count--;
        // __asm__("wdreset"); // Optional: Reset Watchdog
    }

    // 3. Rest period (5x interval)
    
    if (wait_interruptible(5 * interval)) return true;
    
    return false; 
}


// --- Main ---
void main(void) {
  // 1. Hardware Setup
  //MISC |= MISC_LVR_DISABLE; 
  // Setup Button Pin: Input Mode
  BUTTON_PORT_C &= ~(1 << BUTTON_BIT);
  BUTTON_PORT_DIER |= (1 << BUTTON_BIT);
  BUTTON_PORT &= ~(1 << BUTTON_BIT);
  
  // Setup FAN Pin: Output Mode
  FAN_PORT_C |= (1 << FAN_BIT); 
  FAN_PORT_DIER &= ~(1 << FAN_BIT);
  turnFanOff();
  
  // Setup LED Pin: Output Mode
  LED_PORT_C |= (1 << LED_BIT);
  LED_PORT_DIER &= ~(1 << LED_BIT);
  turnLedOff();
  
  // Initialise
  time_keeper_setup();        
  INTRQ = 0;
  __engint();                     // Enable global interrupts
  
  adc_init();
  
  // 2. Main Loop
  while (1) {
   
    // State Logic
    switch (current_state) {
        case STATE_NORMAL:
            // If function returns true (interrupted), 'continue' forces loop to top
            // to handle the button press immediately.
            run_blink_sequence(INTERVAL_NORMAL);
            break;

        case STATE_SLOW:
            run_blink_sequence(INTERVAL_SLOW);
            break;

        case STATE_POWER_DOWN:
            // In power down, we just wait.
            // Ideally, you would use 'stopsys' here to sleep the CPU,
            // but we need the timer running to wake up for button checks 
            // unless you set up a Pin Change Interrupt.
            // For now, we poll slowly (every 100ms) to save "effort".
            wait_interruptible(100);
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
