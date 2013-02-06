void WDT_Init(void);
void measure_current(uint8_t l);
void change_curr_mode(uint8_t mode, uint8_t l);
inline void save_current_state(uint8_t l);
void light_set(void);
void light_disable(uint8_t l);
void light_enable(uint8_t l);
void regulate_pwm(uint8_t l);
void timer_enable(void);
void timer_disable(void);
void light_fault(uint8_t l);

#define LIGHT_LOW 1
#define LIGHT_HIGH 2

#define PWM_ADDED_BITS 3
#define PWM_ADDED_VALUES 8 // PWM_ADDED_BITS ^ 2

#define BUTTON_DEBOUNCE 5

#define PWM_TOP 512

#define SAFE_TOP_DISTANCE 15

#define NONE 0
#define LOW_BATT_WARR 1
#define LOW_BATT 2
#define SHORT_REAR 4
#define SHORT_FRONT 8
#define SHORT_FRONT_LOW 16
#define SHORT_FRONT_HIGH 32

#define NIGHT_STILL 0
#define NIGHT 1
#define EVENING 2
#define DAY 3
#define LOW_BATTERY 4