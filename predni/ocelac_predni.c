#define F_CPU 1000000UL
//----- Include Files ---------------------------------------------------------
#include <avr/io.h> // include I/O definitions (port names, pin names, etc)
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include "common.h"

//----- Begin Code ------------------------------------------------------------

#define LED_PWM_LOW PB3
#define LED_PWM_HIGH PB5
#define USB_PIN PB4
#define BUT_USB PB6
#define SW_LOW_0 PA2
#define SW_LOW_1 PA4
#define SW_HIGH_0 PA0
#define SW_HIGH_1 PA1

#define DATA_IN_0 PB0
#define DATA_IN_1 PB1
#define DATA_OUT_0 PB2
#define DATA_IN_0_BUT (DATA_IN_0+5)
#define DATA_IN_1_BUT (DATA_IN_1+6)



uint16_t pwm[2] = {0, 0};
uint16_t current[2] = {0, 0};
uint16_t zero_pwm[2] = {0, 0};

// current in A * 880

uint16_t currents[2][CURRENT_MODES] = {{0, 88, 220, 792, 44, 44},{0, 88, 220, 792, 44, 44}};
uint8_t cur_tolerances[2][CURRENT_MODES] = {{0, 8, 20, 30, 4, 4},{0, 8, 20, 30, 4, 4}};
uint16_t cur_pwm[2][CURRENT_MODES] = {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};

uint8_t current_mode[2] = {0, 0};

#define CURRENT_MODES_blink 29
uint8_t currents_blink[CURRENT_MODES_blink] = {44,44,45,45,46,47,48,50,52,54,56,58,61,63,66,69,72,75,79,83,87,93,99,104,110,115,121,127,132};
uint16_t cur_blink_pwm[CURRENT_MODES_blink];

uint8_t direction = 1;
uint8_t smooth_count = 0;
uint8_t smooth_state = 0;

uint16_t safe_max_pwm;


uint8_t light_enabled = 0; 
uint8_t LIGHT[2] = {LIGHT_LOW, LIGHT_HIGH};

#ifdef LOGGING
uint16_t log_count = 0;
#endif

// mode 0 
uint8_t mode = 0;

uint8_t disable_count = 0;

uint8_t error = NONE;
uint8_t error_count = 0;

uint8_t vertical_0 = 0;
uint8_t vertical_1 = 0;
uint8_t but_state = (1<<BUT_USB) | (1<<DATA_IN_0_BUT) | (1<<DATA_IN_1_BUT);
uint8_t changes = 0;

uint8_t blink_enable = 0;
uint8_t blink_count = 0;
uint8_t blink_max = 15;

uint8_t i=0;

void debounce(void) {
	uint8_t sample = (PINA & ((1<<SW_LOW_0) | (1<<SW_LOW_1) | (1<<SW_HIGH_0) | (1<<SW_HIGH_1))) |
		(PINB & (1<<BUT_USB)) | ((PINB & (1<<DATA_IN_0))<<5) | ((PINB & (1<<DATA_IN_1))<<6);
	// Set delta to changes from last sample
	uint8_t delta = sample ^ but_state;
	// Increment counters
	vertical_0 ^= vertical_1;
	vertical_1 = ~vertical_1;
	// reset any unchanged bits
    vertical_0 &= delta;
    vertical_1 &= delta;
	// update state & calculate returned change set
	changes = ~(~delta | vertical_0 | vertical_1);
	but_state ^= changes;
}


ISR(WDT_vect) {
	WDTCR |= (1<<WDIE);
}

ISR(ADC_vect) {
}



int main(void) {
	safe_max_pwm = (PWM_TOP - SAFE_TOP_DISTANCE) * PWM_ADDED_VALUES;

	DDRB = (1<<LED_PWM_LOW) | (1<<LED_PWM_HIGH); // LEDs output, other set to input;
	DDRB |= (1<<USB_PIN); // set USB pin as output
	DDRB |= (1<<DATA_OUT_0); // data output
	PORTB = (1<<DATA_OUT_0); // LEDs2 off, other enable pull-ups on, data on
	
	power_all_disable();
	
	but_state |= (PINA & ((1<<SW_LOW_0) | (1<<SW_LOW_1) | (1<<SW_HIGH_0) | (1<<SW_HIGH_1)));
	
	mode = NIGHT;
	
	if (~but_state & (1<<SW_HIGH_0)) {
		i = 1;
	} else if (~but_state & (1<<SW_HIGH_1)) {
		i = 3;
	} else {
		i = 2;
	}
	if (~but_state & (1<<SW_LOW_0)) {
		change_curr_mode(i,0);
		change_curr_mode(0,1);
	} else if (~but_state & (1<<SW_LOW_1)) {
		change_curr_mode(0,0);
		change_curr_mode(i,1);
	} else {
		change_curr_mode(i,0);
		change_curr_mode(i,1);
	}

	set_sleep_mode(SLEEP_MODE_IDLE);
	SREG |= (1<<7); // enable interrupts
	
	
	// setting watchdog
	WDT_Init();
	//pwm[1] = 1300;
	light_set();
	
	while(1) {
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		if (light_enabled & LIGHT_LOW) {
			measure_current(0);
			regulate_pwm(0);
		}
		if (light_enabled & LIGHT_HIGH) {
			error &= ~SHORT_FRONT_HIGH;
			measure_current(1);
			regulate_pwm(1);
			if (error & SHORT_FRONT_HIGH) {
				error_count++;
			} else {
				error_count = 0;
			}
		}
		if (mode == EVENING) {
			uint16_t currentL = 0;
			if (light_enabled & LIGHT_LOW) {
				currentL = current[0]; 
			} else {
				currentL = current[1];
			}
			if ((currentL > currents_blink[smooth_state] / 5 * 4) &&
				(currentL < currents_blink[smooth_state] / 5 * 6)) {
				smooth_count++;
				if (smooth_count >= 2) {
					smooth_count = 0;
					if (light_enabled & LIGHT_LOW) {
						save_current_state(0);
						cur_blink_pwm[smooth_state] = cur_pwm[0][4];
						
					} else {
						save_current_state(1);
						cur_blink_pwm[smooth_state] = cur_pwm[1][4];
					}
					if (direction == 1) {
						smooth_state++;
						if (smooth_state >= CURRENT_MODES_blink) {
							direction = 0;
							smooth_state -= 2;
						}
					} else {
						if (smooth_state > 0) {
							smooth_state--;
						} else {
							direction = 1;
							smooth_state ++;
						}
					}
					currents[0][4] = currents_blink[smooth_state];
					cur_tolerances[0][4] = currents_blink[smooth_state] / 10;
					cur_pwm[0][4] = cur_blink_pwm[smooth_state];
					currents[1][4] = currents_blink[smooth_state];
					cur_tolerances[1][4] = currents_blink[smooth_state] / 10;
					cur_pwm[1][4] = cur_blink_pwm[smooth_state];
					if (light_enabled & LIGHT_LOW) {
						change_curr_mode(4,0);
						
					} else {
						change_curr_mode(4,1);
					}
				}
			}
		}
		if (mode == DAY || mode == LOW_BATTERY) {
			if(error_count < 10 && blink_enable) {
				blink_count++;
				if (blink_count <= 4) {
					if (blink_count == 4) {
						save_current_state(1);
						change_curr_mode(0,1);
					}
				} else if (blink_count >= blink_max) {
					blink_count = 0;
					change_curr_mode(mode==DAY?3:5,1);
				}
			} else if (current[1] <= currents[1][mode==DAY?3:5] * 6/5 &&
					current[1] >= currents[1][mode==DAY?3:5] * 4/5) {
				blink_enable = 1;
				blink_count = 0;
			}
		}
		if (error) {
			PORTB &= ~(1<<DATA_OUT_0);
		}
		debounce();
		if(changes) {
			/*if (changes & ((1<<SW_LOW_0) | (1<<SW_LOW_1))) {
				if (~but_state & (1<<SW_LOW_0)) {
					save_current_state(0);
					change_curr_mode(0,0);
				} else if (~but_state & (1<<SW_LOW_1)) {
					save_current_state(0);
					change_curr_mode(2,0);
				} else {
					save_current_state(0);
					change_curr_mode(1,0);
				}
			}
			if (changes & ((1<<SW_HIGH_0) | (1<<SW_HIGH_1))) {
				if (~but_state & (1<<SW_HIGH_0)) {
					save_current_state(1);
					change_curr_mode(0,1);
				} else if (~but_state & (1<<SW_HIGH_1)) {
					save_current_state(1);
					change_curr_mode(2,1);
				} else {
					save_current_state(1);
					change_curr_mode(1,1);
				}
			}*/
			if (changes & ((1<<SW_LOW_0) | (1<<SW_LOW_1) | (1<<SW_HIGH_0) | (1<<SW_HIGH_1))) {
				switch (mode) {
					case NIGHT: {
						if (~but_state & (1<<SW_HIGH_0)) {
							i = 1;
						} else if (~but_state & (1<<SW_HIGH_1)) {
							i = 3;
						} else {
							i = 2;
						}
						save_current_state(0);
						save_current_state(1);
						if (~but_state & (1<<SW_LOW_0)) {
							change_curr_mode(i,0);
							change_curr_mode(0,1);
						} else if (~but_state & (1<<SW_LOW_1)) {
							change_curr_mode(0,0);
							change_curr_mode(i,1);
						} else {
							change_curr_mode(i,0);
							change_curr_mode(i,1);
						}
						break;
					}
					case EVENING: {
						if (~but_state & (1<<SW_LOW_0)) {
							change_curr_mode(4,0);
							change_curr_mode(0,1);
						} else if (~but_state & (1<<SW_LOW_1)) {
							change_curr_mode(0,0);
							change_curr_mode(4,1);
						}
					}
				}

			}
			if (changes & (1<<BUT_USB) & but_state) {
				if (mode != LOW_BATTERY) {
					PORTB ^= (1<<USB_PIN);
				}
			}
			if (changes & ((1<<DATA_IN_0_BUT) | (1<<DATA_IN_1_BUT))) {
				save_current_state(0);
				save_current_state(1);
				if ((but_state & (1<<DATA_IN_0_BUT)) && (but_state & (1<<DATA_IN_1_BUT))) {
					mode = NIGHT;
					if (~but_state & (1<<SW_HIGH_0)) {
							i = 1;
						} else if (~but_state & (1<<SW_HIGH_1)) {
							i = 3;
						} else {
							i = 2;
						}
						if (~but_state & (1<<SW_LOW_0)) {
							change_curr_mode(i,0);
							change_curr_mode(0,1);
						} else if (~but_state & (1<<SW_LOW_1)) {
							change_curr_mode(0,0);
							change_curr_mode(i,1);
						} else {
							change_curr_mode(i,0);
							change_curr_mode(i,1);
						}
				} else if ((but_state & (1<<DATA_IN_0_BUT))) {
					mode = EVENING;
					if (~but_state & (1<<SW_LOW_0)) {
						change_curr_mode(4,0);
						change_curr_mode(0,1);
					} else {
						change_curr_mode(0,0);
						change_curr_mode(4,1);
					}
				} else if ((but_state & (1<<DATA_IN_1_BUT))) {
					mode = DAY;
					change_curr_mode(0,0);
					change_curr_mode(3,1);
					blink_max = 40;
					blink_enable = 0;	
				} else {
					mode = LOW_BATTERY;
					change_curr_mode(0,0);
					change_curr_mode(5,1);
					blink_max = 30;
					blink_enable = 0;
					PORTB &= ~(1<<USB_PIN);
				}
			}
		}
		light_set();
	}
	wdt_disable();
	light_disable(0);
	light_disable(1);
	timer_disable();
	power_all_disable();
	sleep_enable();
	sleep_cpu();
	return 0;
}

