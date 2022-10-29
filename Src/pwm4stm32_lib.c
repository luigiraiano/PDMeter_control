//
//  pwm4stm32_lib.c
//  
//
//  Created by Luigi Raiano on 24/06/2020.
//

#include "pwm4stm32_lib.h"

/* INFO */
// The following variables must be defined (as follows, without typing extern) and initialized (={0}) in the main function.
// PWM related variables
//extern uint32_t duty;
//extern uint32_t value;
extern uint32_t duty_min;
extern uint32_t duty_max;


/**
 * @brief PWM change duty
 * @param duty value, timer and channel out
 * @retval None
 */
void change_pwm_duty(uint32_t duty_value, TIM_HandleTypeDef *timer, uint32_t CHANNEL){
    /*-------CHANGE PWM DUTY------*/
    if(duty_value >= duty_max){
        duty_value = duty_max;
    }
    else if(duty_value <= duty_min){
        duty_value = duty_min;
    }

    __HAL_TIM_SET_COMPARE(timer, CHANNEL, duty_value);
}

/**
 * @brief PWM initialization
 * @param timer and channel out
 * @retval None
 */
void pwm_init(TIM_HandleTypeDef *timer, uint32_t CHANNEL){
    /*-------INITIALIZE AND START PWM------*/
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Initialize and start pwm
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = duty_min;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(timer, CHANNEL) != HAL_OK){
        Error_Handler();
    }   // start pwm generation
}
