// Author: David Sin
// LED Breathing effect
#include <stdint.h>
#include "main.h"

#define LED_STEP rand() % 20
// TIM_CHANNEL_2 LED_R
// TIM_CHANNEL_3 LED_G
// TIM_CHANNEL_4 LED_B

// Prescaler: 72 - 1
// Period: 1000

extern TIM_HandleTypeDef htim3;
extern uint8_t LED_R_value;
extern uint8_t LED_G_value;
extern uint8_t LED_B_value;
extern uint8_t LED_R_up;
extern uint8_t LED_G_up;
extern uint8_t LED_B_up;

void ledInit() {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    // Set random duty cycle
    LED_R_value = rand() % 200;
    LED_G_value = rand() % 200;
    LED_B_value = rand() % 200;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LED_R_value);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, LED_G_value);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, LED_B_value);
}

void ledBreath() {
    // RGB breathing effect
    if (LED_R_up == 1) {
        LED_R_value = LED_R_value + LED_STEP;
        if (LED_R_value >= 200) {
            LED_R_up = 0;
            LED_R_value = 200;
        }
    } else {
        LED_R_value = LED_R_value - LED_STEP;
        if (LED_R_value <= 0) {
            LED_R_up = 1;
            LED_R_value = 0;
        }
    }

    if (LED_G_up) {
        LED_G_value = LED_G_value + LED_STEP;
        if (LED_G_value >= 200) {
            LED_G_up = 0;
            LED_G_value = 200;
        }
    } else {
        LED_G_value = LED_G_value - LED_STEP;
        if (LED_G_value <= 0) {
            LED_G_up = 1;
            LED_G_value = 0;
        }
    }

    if (LED_B_up) {
        LED_B_value = LED_B_value + LED_STEP;
        if (LED_B_value >= 200) {
            LED_B_up = 0;
            LED_B_value = 200;
        }
    } else {
        LED_B_value = LED_B_value - LED_STEP;
        if (LED_B_value <= 0) {
            LED_B_up = 1;
            LED_B_value = 0;
        }
    }

    htim3.Instance -> CCR2 = LED_R_value;
    htim3.Instance -> CCR3 = LED_G_value;
    htim3.Instance -> CCR4 = LED_B_value;
}