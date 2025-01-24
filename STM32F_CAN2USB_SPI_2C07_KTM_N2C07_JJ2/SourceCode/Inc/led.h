typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_SLOW_FLASH, // 0.5s??
    LED_SLOW_FLASH_ONCE_2S_OFF,
    LED_SLOW_FLASH_TWICE_2S_OFF,
    LED_FAST_FLASH_ONCE_2S_OFF,
    LED_FAST_FLASH_TWICE_2S_OFF,
    LED_FAST_FLASH // 0.2s??
} LED_State;

typedef enum {
Standby_LED = 1,
Charging_LED,  // 2
Charging_finished_LED,  // 3
Discharge_LED,  // 4
Battery_low_LED,  // 5
Battery_critical_low_LED,  // 6
Error_Solar_panel_LED,  // 7
Error_Battery_LED,  // 8
Update_LED,  // 9
} LED_mode;