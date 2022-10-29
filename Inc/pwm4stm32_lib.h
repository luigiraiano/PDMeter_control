//
//  pwm4stm32_lib.h
//  
//
//  Created by Luigi Raiano on 24/06/2020.
//

#ifndef __PWM4STM32_LIB_H
#define __PWM4STM32_LIB_H

#include "main.h"

//uint32_t duty_min = 0; // 0%
//extern uint32_t duty_max; // 100%

/**** Function prototypes *****/
void pwm_init(TIM_HandleTypeDef *timer, uint32_t CHANNEL);
void change_pwm_duty(uint32_t duty_value, TIM_HandleTypeDef *timer, uint32_t CHANNEL);

#endif /* __PWM4STM32_LIB_H */
