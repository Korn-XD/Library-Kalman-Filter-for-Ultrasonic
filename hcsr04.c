#include "hcsr04.h"
#include "tim.h"
#include "gpio.h"

uint32_t raw_ticks = 0;
float raw_distance_m = 0;
uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint8_t Is_First_Captured = 0;

extern TIM_HandleTypeDef htim2;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (Is_First_Captured == 0) {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); 
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); 
            __HAL_TIM_SET_COUNTER(htim, 0);
            if (IC_Val2 > IC_Val1) raw_ticks = IC_Val2 - IC_Val1;
            else raw_ticks = (0xFFFFFFFF - IC_Val1) + IC_Val2;
            
            raw_distance_m = (float)raw_ticks * 0.0003432f / 2.0f;
            Is_First_Captured = 0; 
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
        }
    }
}

void HCSR04_Trigger(void) {
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    for(volatile int i=0; i<500; i++); 
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
}