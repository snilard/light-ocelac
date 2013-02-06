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

#define LED_PWM PB3

#define DATA_OUT_0 PB0
#define DATA_OUT_1 PB1
#define DATA_IN_0 PB2

#define BUTTON_1 PA4
#define BUTTON_2 PA5

#define LED_1_A PA2
#define LED_2_B PB6
#define LED_3_B PB5
#define LED_5_A PA0
#define LED_4_A PA1

#define LED_ON_1 PORTA|=(1<<LED_1_A)
#define LED_ON_2 PORTB|=(1<<LED_2_B)
#define LED_ON_3 PORTB|=(1<<LED_3_B)
#define LED_ON_4 PORTA|=(1<<LED_4_A)
#define LED_ON_5 PORTA|=(1<<LED_5_A)



uint16_t pwm[1] = {0};
uint16_t current[1] = {0};
uint16_t zero_pwm[1] = {0};

// current in A * 880
uint16_t currents[1][CURRENT_MODES] = {{0, 60, 120, 450}};
uint8_t cur_tolerances[1][CURRENT_MODES] = {{0, 10, 15, 40}};
uint16_t cur_pwm[1][CURRENT_MODES] = {{0, 0, 0, 0}};

uint8_t current_mode[1] = {0};

uint16_t safe_max_pwm;

uint8_t light_enabled = 0; 
uint8_t LIGHT[1] = {LIGHT_LOW};

uint8_t blink_enable = 0;
uint8_t blink_count = 0;
uint8_t blink_max = 15;
uint8_t display_count = 1;
#define DISPLAY_MAX 150

uint8_t measure_count = 0;

uint16_t voltage = 0;

uint8_t mode = NIGHT;
uint8_t mode_cur_mode[5] = {1,1,2,3,1};
uint8_t mode_blink_max[5] = {15,15,17,40,20};

//uint16_t counter = 12900;

/*#ifdef LOGGING
uint8_t log_count = 0;
#endif*/

uint8_t vertical_0 = 0;
uint8_t vertical_1 = 0;
uint8_t but_state = (1<<BUTTON_1) | (1<<BUTTON_2);
uint8_t changes;

uint8_t error = NONE;
uint8_t error_count = 0;
uint8_t battery = 3;

void debounce(void) {
	uint8_t sample = (PINA & ((1<<BUTTON_1) | (1<<BUTTON_2))) | (PINB & (1<<DATA_IN_0));
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

void measure_voltage(void) {
	uint16_t voltage_sum = 0;
	uint8_t i;
	//uint16_t offset = 0;
	power_adc_enable();
	// setup A/D convertor
	ADCSRB = (1<< REFS2);
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADATE) | (1<<ADIE);
	
	// PB4 vs GND
	ADMUX =  (1<< REFS1) | (1<< REFS0) | (1<<MUX0) | (1<<MUX1) | (1<<MUX2);

	// enable and start conversion, 1:8 clock to 125 kHz ADC clock
	ADCSRA |= (1<<ADSC);
	// measure
	/*for (i = 0; i < 32; i++) {
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	}*/
	for (i = 0; i < 1; i++) {
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		voltage_sum += ADCL + (ADCH << 8);
	}
	voltage = (voltage_sum >> 0);// -offset;
	ADCSRA &= ~(1<<ADEN);
	power_adc_disable();
}

/*void led_on(uint8_t led) {
	switch (led) {
		case 1: {
			PORTA |= (1<<LED_1_A);
			break;
		}
		case 2: {
			PORTB |= (1<<LED_2_B);
			break;
		}
		case 3: {
			PORTB |= (1<<LED_3_B);
			break;
		}
		case 4: {
			PORTA |= (1<<LED_4_A);
			break;
		}
		case 5: {
			PORTA |= (1<<LED_5_A);
			break;
		}
	}
}*/

void leds_off(void) {
	PORTA &= ~(1<<LED_1_A | 1<<LED_4_A | 1<<LED_5_A);
	PORTB &= ~(1<<LED_2_B | 1<<LED_3_B);
}

void show_voltage(void) {
	leds_off();
	if (battery == 3) {
		if (voltage >= 857) { //10,50V
			LED_ON_1;
		}
		if (voltage >= 930) { //11,39V
			LED_ON_2;
		}
		if (voltage >= 943) { //11,55V
			LED_ON_3;
		}
		if (voltage >= 961) { //11,77V
			LED_ON_4;
		}
		if (voltage >= 1000) { //12,25V
			LED_ON_5;
		}
		if (voltage < 857) {
			if (measure_count <= 10) {
				LED_ON_1;
			}
		}
	} else {
		if (voltage >= 571) {
			LED_ON_1;
		}
		if (voltage >= 620) {
			LED_ON_2;
		}
		if (voltage >= 628) {
			LED_ON_3;
		}
		if (voltage >= 640) {
			LED_ON_4;
		}
		if (voltage >= 666) {
			LED_ON_5;
		}
		if (voltage < 571) {
			if (measure_count <= 10) {
				LED_ON_1;
			}
		}	
	}
}

void show_mode(void) {
	leds_off();
	switch (mode) {
		case NIGHT_STILL: {
			LED_ON_5;
		}
		case NIGHT: {
			LED_ON_4;
			break;
		}
		case EVENING: {
			LED_ON_3;
			LED_ON_2;
			break;
		}
		case DAY: {
			LED_ON_1;
			break;
		}
	}
}

void set_mode(void) {
	change_curr_mode(mode_cur_mode[mode], 0);
	blink_max = mode_blink_max[mode];
	blink_enable = 0;
	PORTB &= ~((1<<DATA_OUT_0) | (1<<DATA_OUT_1));
	switch (mode) {
		case NIGHT:
		case NIGHT_STILL: {
			PORTB |= (1<<DATA_OUT_0) | (1<<DATA_OUT_1);
			break;
		}
		case EVENING: {
			PORTB |= (1<<DATA_OUT_0);
			break;
		}
		case DAY: {
			PORTB |= (1<<DATA_OUT_1);
			break;	
		}
		//LOW BATTERY = both data 0
	}
}

void show_error(void) {
	leds_off();
	if (measure_count <= 12) {
		if (error == LOW_BATT_WARR) {
			LED_ON_2;
			LED_ON_3;
			LED_ON_4;
			LED_ON_5;
		}
		if (error & LOW_BATT) {
			LED_ON_1;
		}
		if (error & SHORT_FRONT) {
			LED_ON_1;
			LED_ON_2;
			LED_ON_3;
		}
		if (error & SHORT_REAR) {
			LED_ON_1;
			LED_ON_4;
			LED_ON_5;
		}
	} else if (error == LOW_BATT_WARR) {
		show_mode();
	}
}

int main(void) {
	safe_max_pwm = (PWM_TOP - SAFE_TOP_DISTANCE) * PWM_ADDED_VALUES;

	DDRB = (1<<LED_PWM) | (1<<DATA_OUT_0) | (1<<DATA_OUT_1) | (1<<LED_2_B) | (1<<LED_3_B);
	DDRA = (1<<LED_1_A) | (1<<LED_4_A) | (1<<LED_5_A);
	
	power_all_disable();
	
	mode = NIGHT;
	set_mode();

	set_sleep_mode(SLEEP_MODE_IDLE);
	SREG |= (1<<7); // enable interrupts
	
	// setting watchdog
	WDT_Init();
	pwm[0]++;
	light_set();
	
	measure_voltage();
	if (voltage < 740) {
		battery = 2;
	}
	show_voltage();
	
	while(1) {
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		if (display_count > 0) {
			show_voltage();
			display_count++;
			if (display_count >= DISPLAY_MAX) {
				display_count = 0;
				show_mode();
			}
		} else {
			if (error) {
				show_error();
			} else {
				show_mode();
			}
		}
		if (light_enabled & LIGHT_LOW) {
			error &= ~SHORT_REAR;
			measure_current(0);
			regulate_pwm(0);
			if (error & SHORT_REAR) {
				error_count++;
			} else {
				error_count = 0;
			}
		}
		if (mode != NIGHT_STILL) {
			if(error_count < 10 && blink_enable) {
				blink_count++;
				if (blink_count <= 4) {
					if (blink_count == 4) {
						save_current_state(0);
						change_curr_mode(0,0);
					}
				} else if (blink_count >= blink_max) {
					blink_count = 0;
					change_curr_mode(mode_cur_mode[mode],0);
				}
			} else if (current[0] <= currents[0][mode_cur_mode[mode]] * 6/5 &&
					current[0] >= currents[0][mode_cur_mode[mode]] * 4/5) {
				blink_enable = 1;
				blink_count = 0;
			}
		}
		light_set();
		debounce();
		if (changes) {
			if (display_count == 0 && mode != LOW_BATTERY) {
				if (changes & (1<<BUTTON_1) & but_state) {
					if (mode < DAY) {
						mode++;
						show_mode();
						set_mode();
					}
				}
				if (changes & (1<<BUTTON_2) & but_state) {
					if (mode > NIGHT_STILL) {
						mode--;
						show_mode();
						set_mode();
					}
				}
			}
			if ((~but_state & (1<<BUTTON_1)) && (~but_state & (1<<BUTTON_2))) {
				display_count = 1;
			}
			if (~but_state & (1<<DATA_IN_0)) {
				error |= SHORT_FRONT;
			}
		}
		measure_count++;
		if (measure_count >= 30) {
			measure_count = 0;
			measure_voltage();
			if (voltage < (battery==3?922:615)) { // 11,3V
				error |= LOW_BATT_WARR;
			} else {
				error &= ~LOW_BATT_WARR;
			}
			if (voltage < (battery==3?857:571)) { // 10,5V
				if (mode != LOW_BATTERY) {
					mode = LOW_BATTERY;
					set_mode();
					error |= LOW_BATT;
				}
			}
		}
		/*counter++;
		if (counter == 13100) {
			counter = 0;
			if (log_count < 256) {
				eeprom_write_byte((uint8_t *)log_count, (uint8_t)voltage);
				log_count++;
			}
		}*/
		/*} else {
			LIGHT_OFF;
			PLLCSR &= ~(1<<PLLE); // PLL start (high speed clock)
			TCCR1B &= ~(1<<CS10);
			power_all_disable();
		}*/
		
	}
	wdt_disable();
	light_disable(0);
	timer_disable();
	power_all_disable();
	sleep_enable();
	sleep_cpu();
	return 0;
}

