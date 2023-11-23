// Author: David Sin
#include <stdint.h>
#include "main.h"

// Target PWM: 1000 Hz
// Prescaler: 72 - 1
// Period: 1000 - 1

extern TIM_HandleTypeDef htim1;
// TIM_CHANNEL_1

// Start up beep
void buzzerBeep() {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_Delay(250);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
}

// Tape loading beep
void buzzerTone(uint8_t tone) {
    // Tone range: 0x00 - 0xFF
    // Lowest CCR: 231
    // Highest CCR: 999
    // 256 tones be 3 Hz starting from 231
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 231 + tone * 3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_Delay(1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
}