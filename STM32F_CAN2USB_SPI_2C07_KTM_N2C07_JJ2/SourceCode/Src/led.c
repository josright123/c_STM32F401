#include "main.h"


// 
uint16_t timer_counter = 0; // ?100ms??
uint8_t resetLED = 1;

    uint16_t led1_timer = 0, led2_timer = 0;
    bool led1_on = false, led2_on = false;
    static uint8_t flash_count = 0; // 

    LED_State led1_state = LED_OFF;
    LED_State led2_state = LED_OFF;
// LED??
void control_LED(LED_State state, uint16_t *timer, bool *led_on) {
    switch (state) {
    case LED_ON:
        *led_on = true;
        break;
    case LED_OFF:
        *led_on = false;
        break;
    case LED_SLOW_FLASH:
        if (*timer >= 5) { // 0.5?(5 * 100ms)
            *timer = 0;
            *led_on = !*led_on;
        }
        (*timer)++;
        break;
    case LED_SLOW_FLASH_ONCE_2S_OFF:
        if (flash_count < 1) {
            if (*timer >= 5) { // 0.5?(5 * 100ms)
                *timer = 0;
                *led_on = !*led_on;
                if (!*led_on) flash_count++;
            }
        } else {
            if (*timer >= 20) { // 2?(20 * 100ms)
                *timer = 0;
                flash_count = 0;
            }
        }
        (*timer)++;
        break;
    case LED_SLOW_FLASH_TWICE_2S_OFF:
        if (flash_count < 2) {
            if (*timer >= 5) { // 0.5?(5 * 100ms)
                *timer = 0;
                *led_on = !*led_on;
                if (!*led_on) flash_count++;
            }
        } else {
            if (*timer >= 20) { // 2?(20 * 100ms)
                *timer = 0;
                flash_count = 0;
            }
        }
        (*timer)++;
        break;
    case LED_FAST_FLASH_ONCE_2S_OFF:
        if (flash_count < 1) {
            if (*timer >= 2) { // 0.2?(2 * 100ms)
                *timer = 0;
                *led_on = !*led_on;
                if (!*led_on) flash_count++;
            }
        } else {
            if (*timer >= 20) { // 2?(20 * 100ms)
                *timer = 0;
                flash_count = 0;
            }
        }
        (*timer)++;
        break;
    case LED_FAST_FLASH_TWICE_2S_OFF:
        if (flash_count < 2) {
            if (*timer >= 2) { // 0.2?(2 * 100ms)
                *timer = 0;
                *led_on = !*led_on;
                if (!*led_on) flash_count++;
            }
        } else {
            if (*timer >= 20) { // 2?(20 * 100ms)
                *timer = 0;
                flash_count = 0;
            }
        }
        (*timer)++;
        break;
    case LED_FAST_FLASH:
        if (*timer >= 2) { // 0.2?(2 * 100ms)
            *timer = 0;
            *led_on = !*led_on;
        }
        (*timer)++;
        break;
    default:
        break;
    }
}


void control_LED_all(void){	
        control_LED(led1_state, &led1_timer, &led1_on);
        control_LED(led2_state, &led2_timer, &led2_on);
	if(led1_on == true) LED1_STATUS_H;
		else LED1_STATUS_L;
	if(led2_on == true) LED2_BAT_H;
		else LED2_BAT_L;
}
// LED??
void update_mode(void ) {
    switch (current_mode) {
    case 1: // Mode1
        led1_state = LED_ON;
        led2_state = LED_OFF;
        if(resetLED == 1){
            led1_on = true;
            led2_on = false;
        }
        break;
    case 2: // Mode2
        led1_state = LED_SLOW_FLASH; // LED_ON;
        led2_state = LED_ON; // LED_SLOW_FLASH;
        if(resetLED == 1){
            led1_on = true;
            led2_on = true;
        }
        break;
    case 3: // Mode3
        led1_state = LED_SLOW_FLASH;
        led2_state = LED_OFF;
        if(resetLED == 1){
            led1_on = true;
            led2_on = false;
        }
        break;
    case 4: // Mode4
        led1_state = LED_OFF;
        led2_state = LED_SLOW_FLASH;
        if(resetLED == 1){
            led1_on = false;
            led2_on = true;
        }
        break;
    case 5: // Mode5
        led1_state = LED_OFF;
        led2_state = LED_SLOW_FLASH_ONCE_2S_OFF;
        if(resetLED == 1){
            led1_on = false;
            led2_on = true;
            flash_count = 0;
        }
        break;
    case 6: // Mode6
        led1_state = LED_OFF;
        led2_state = LED_SLOW_FLASH_TWICE_2S_OFF;
        if(resetLED == 1){
            led1_on = false;
            led2_on = true;
            flash_count = 0;
        }
        break;
    case 7: // Mode7
        led1_state = LED_OFF;
        led2_state = LED_FAST_FLASH_ONCE_2S_OFF;
        if(resetLED == 1){
            led1_on = false;
            led2_on = true;
            flash_count = 0;
        }
        break;
    case 8: // Mode8
        led1_state = LED_OFF;
        led2_state = LED_FAST_FLASH_TWICE_2S_OFF;
        if(resetLED == 1){
            led1_on = false;
            led2_on = true;
            flash_count = 0;
        }
        break;
    case 9: // Mode9
        led1_state = LED_FAST_FLASH;
        led2_state = LED_FAST_FLASH;
        if(resetLED == 1){
            led1_on = true;
            led2_on = true;
        }
        break;
    default:
        led1_state = LED_OFF;
        led2_state = LED_OFF;
            led1_on = false;
            led2_on = false;
        break;
    }
            resetLED = 0;
            led1_timer = 0, led2_timer = 0;
}


void set_led_mode(uint8_t mode) {
	led1_on = false, led2_on = false;
	old_LED_MODE = mode; 
	resetLED = 1;	
}