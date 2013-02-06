#define F_CPU 1000000UL

#include <avr/io.h> // include I/O definitions (port names, pin names, etc)
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#include "common.h"

extern uint16_t pwm[];
extern uint16_t current[];
extern uint16_t zero_pwm[];


extern uint16_t currents[][CURRENT_MODES];
extern uint8_t cur_tolerances[][CURRENT_MODES];
extern uint16_t cur_pwm[][CURRENT_MODES];

extern uint8_t current_mode[];

extern uint16_t safe_max_pwm;

extern uint8_t light_enabled; 
extern uint8_t LIGHT[];

extern uint8_t error;

//initialize watchdog
void WDT_Init() {
	//disable interrupts
	cli();
	//reset watchdog
	wdt_reset();
	//set up WDT interrupt
	//Start watchdog timer with 16ms delay
	WDTCR = (1<<WDCE)|(1<<WDIE)|(1<<WDE);
	//Enable global interrupts
	sei();
}

void measure_current(uint8_t l) {
	uint16_t current_sum = 0;
	uint8_t i;
	//uint16_t offset = 0;
	power_adc_enable();
	// setup A/D convertor
	ADCSRB = (1<< REFS2);
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADATE) | (1<<ADIE);
	/*#ifdef FRONT
	ADMUX =  (1<< REFS1) | (1<< REFS0) | (1<<MUX4) | (1<<MUX2) | (1<<MUX1);
	ADCSRA |= (1<<ADSC);
	sleep_enable();
	sleep_cpu();
	sleep_disable();
	offset = ADCL + (ADCH << 8);
	#endif*/
	
	if (l == 0) {
		#ifdef FRONT
		// +PA7 -PA6, right aligned, int ref with cap
		ADMUX =  (1<< REFS1) | (1<< REFS0) | (1<<MUX4) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0);
		#endif
		
		#ifdef REAR
		// +PA6 -PA7, right aligned, int ref with cap
		ADCSRB |= (1<<MUX5);
		ADMUX =  (1<< REFS1) | (1<< REFS0) | (1<<MUX4);
		#endif
	} else {
		// +PA5 -PA6, right aligned, int ref with cap
		ADMUX =  (1<< REFS1) | (1<< REFS0) | (1<<MUX4) | (1<<MUX2);
	}
	// enable and start conversion, 1:8 clock to 125 kHz ADC clock
	ADCSRA |= (1<<ADSC);
	// measure
	for (i = 0; i < 32; i++) {
		sleep_enable();
		sleep_cpu();
		sleep_disable();
	}
	for (i = 0; i < 8; i++) {
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		current_sum += ADCL + (ADCH << 8);
	}
	current[l] = (current_sum >> 3);// -offset;
	ADCSRA &= ~(1<<ADEN);
	power_adc_disable();
}

void change_curr_mode(uint8_t mode, uint8_t l) {
	current_mode[l] = mode;
	if (currents[l][mode] != 0) {
		if (! (light_enabled & LIGHT[l])) {
			light_enable(l);
			//#ifdef REAR
			pwm[l] = zero_pwm[l];
			//#endif
		} else if (cur_pwm[l][mode] > 0) {	
			pwm[l] = cur_pwm[l][mode];
		}
	} else {
		light_disable(l);
	}
	//PORTB &= ~(1 << USB_PIN);
}

inline void save_current_state(uint8_t l) {
	cur_pwm[l][current_mode[l]] = pwm[l];
	/*if (currents[l][current_mode[l]] > MAX / 5 * 4) {
		pwm[l] -= 10;
	}*/
}

void light_set() {
	uint16_t pwm_set = pwm[0] >> PWM_ADDED_BITS;
	TC1H = pwm_set >> 8;
	OCR1B = (uint8_t)pwm_set;
	#ifdef FRONT
	pwm_set = pwm[1] >> PWM_ADDED_BITS;
	TC1H = pwm_set >> 8;
	OCR1D = (uint8_t)pwm_set;
	#endif
}

void light_disable(uint8_t l) {
	if (l == 0) {
		TC1H = 0;
		OCR1B = 0;
		TCCR1A &= ~(1<<PWM1B); // pwm low
		light_enabled &= ~LIGHT_LOW;
	}
	#ifdef FRONT
	else {
		TC1H = 0;
		OCR1D = 0;
		TCCR1C &= ~(1<<PWM1D); // pwm high
		light_enabled &= ~LIGHT_HIGH;
	}
	#endif
	pwm[l] = 0;
	if (!light_enabled) {
		timer_disable();
	}
}

void light_enable(uint8_t l) {
	if (!light_enabled) {
		timer_enable();
	}
	if (l == 0) {
		TCCR1A |= (1<<PWM1B); // pwm low
		light_enabled |= LIGHT_LOW;
	}
	#ifdef FRONT
	else {
		TCCR1C |= (1<<PWM1D); // pwm high
		light_enabled |= LIGHT_HIGH;
	}
	#endif
}

void light_fault(uint8_t l) {
	light_disable(l);
	int i;
	for (i = 0; i < CURRENT_MODES; i++) {
		cur_pwm[l][i] = 0;
	}
	//PORTB |= (1 << USB_PIN);
	#ifdef FRONT
	error |= SHORT_FRONT;
	if (l == 0) {
		error |= SHORT_FRONT_LOW;
	} else {
		error |= SHORT_FRONT_HIGH;
	}
	#endif
	#ifdef REAR
	error |= SHORT_REAR;
	#endif
}

void regulate_pwm(uint8_t l) {
	uint16_t current_l = current[l];
	uint16_t pwm_l = pwm[l];
	uint16_t target_current_l = currents[l][current_mode[l]];
	uint8_t target_tolerance_l = cur_tolerances[l][current_mode[l]];
	if (current_l > MAX) {
		light_fault(l);
		return;
	}
	if (current_l > 20 && pwm_l < (40 << PWM_ADDED_BITS)) {
		light_fault(l);
		return;
	}
	if (current_l < 10 && pwm_l > ((PWM_TOP*3/4) <<PWM_ADDED_BITS)) {
		light_fault(l);
		return;
	}
	//#ifdef REAR
	if (pwm_l == zero_pwm[l]) {
		if (current_l > 20) {
			light_fault(l);
			return;
		} else {
			change_curr_mode(current_mode[l],l);
			if (pwm[l] != zero_pwm[l]) {
				return;
			}
		}
		
	}
	//#endif
	if (current_l < target_current_l - target_tolerance_l) {
		pwm_l++;
		if (current_l < target_current_l - 2 * target_tolerance_l) {
			pwm_l += PWM_ADDED_VALUES/2-1;
			if (current_l < target_current_l - 4 * target_tolerance_l) {
				pwm_l += 3*PWM_ADDED_VALUES;
				if (current_l < 10) {
					zero_pwm[l] = pwm[l];
					pwm_l += 6*PWM_ADDED_VALUES;
				}
			}
		}
	}
	if (current_l > target_current_l + target_tolerance_l) {
		pwm_l--;
		if (current_l > target_current_l + 2 * target_tolerance_l) {
			pwm_l -= PWM_ADDED_VALUES/2-1;
			if (current_l > target_current_l + 4 * target_tolerance_l) {
				pwm_l -= 2 * PWM_ADDED_VALUES;
			}
		}
	}
	if (pwm_l > safe_max_pwm) { // allowd max pwm
		pwm_l = safe_max_pwm;
	}
	pwm[l] = pwm_l;
}

void timer_enable() {
	power_timer1_enable();
	PLLCSR |= (1<<PLLE) | (1<<LSM); // PLL start (high speed clock in 32 MHz mode)
	_delay_us(150); // 100 us in datasheet 
	while(! (PLLCSR & (1<<PLOCK)));
	PLLCSR |= (1<<PCKE); // setting PCK as clock source
	//OC1B Connected, clear in Compare Match 
	//Enable PWM based on comparator OCR1B and OCR1D
	TCCR1A |= (1<<COM1B1); // pwm low
	#ifdef FRONT
	TCCR1C |= (1<<COM1D1); //pwm high
	#endif
	TCCR1D &= ~((1<<WGM11) | (1<<WGM10)); // Fast PWM Mode 
	// setting frequency etc.		
	TC1H = 0; // Setting output low to 0
	OCR1B = 0;
	TC1H = 0; // Setting output high to 0
	OCR1D = 0;
	//TC1H = PWM_TOP[freq_state] >> 8; // Setting top
	//OCR1C = (uint8_t)PWM_TOP[freq_state];
	TC1H = PWM_TOP >> 8; // Setting top
	OCR1C = (uint8_t)PWM_TOP;
	TIMSK = 0x00; // no timer interrupt 
	//clock source 64MHz
	TCCR1B |= (1<<CS10);
}

void timer_disable() {
	PLLCSR &= ~(1<<PLLE); // PLL stop (high speed clock)
	TCCR1B &= ~(1<<CS10);
	power_timer1_disable();
}