#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "main.h"

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA

extern float raw_distance_m;

void HCSR04_Trigger(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* INC_HCSR04_H_ */