/*
 * Luigi Raiano, v11, 10-07-2020
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "MAX1187_Library_STM32.h"
#include "pwm4stm32_lib.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADC_EOC_Pin GPIO_PIN_4
#define ADC_EOC_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define MOT_ENABLE_Pin GPIO_PIN_7
#define MOT_ENABLE_GPIO_Port GPIOC
#define MOT_DIRECTION_Pin GPIO_PIN_8
#define MOT_DIRECTION_GPIO_Port GPIOC
#define MOT_DIN4_Pin GPIO_PIN_9
#define MOT_DIN4_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* Defines ---------------------------------------------------------*/
#define TWOS_COMPL_ARRAY_SIZE 8
#define NUMEBER_OF_DOUBLE 3 // modified in version 6
#define BYTES_OF_DOUBLE_TO_SEND (TWOS_COMPL_ARRAY_SIZE*NUMEBER_OF_DOUBLE)

//#define n_bytes_uart_RX (8+1+1) // Rx_buffer_size [tau_d; n_perturbations; state_flag]
#define n_bytes_uart_RX (8+8+1+1) // Rx_buffer_size [tau_d/theta_d; delta_t_in; n_perturbations; state_flag]
#define Tx_buffer_size_FS 2 // byte of force sensor to send
#define Tx_buffer_size_curr 2
#define Tx_buffer_size_ENCODER 2
#define CHECK_data_size 1
#define LAST_BYTES_SIZE 1
#define SIZE_UINT32 4 // 4 bytes for a 32 bit variable

#define BAUDRATE 921600

/**********SPI MULTIPLE SCAN*********/
//#define Tx_total_data_size (4 + Tx_buffer_size_FS + Tx_buffer_size_curr + BYTES_OF_DOUBLE_TO_SEND  + LAST_BYTES_SIZE) // PRE+DEV_ID+Time_L+Time_H+Data_H(force sensor)+data_L(force sensor)+Data_H(speed)+data_L(speed)+torque_double
#define Tx_total_data_size (2 + SIZE_UINT32 + Tx_buffer_size_FS + Tx_buffer_size_curr + BYTES_OF_DOUBLE_TO_SEND  + LAST_BYTES_SIZE) // PRE+DEV_ID+Time_L+Time_H+Data_H(force sensor)+data_L(force sensor)+Data_H(speed)+data_L(speed)+torque_double
/**********SPI MULTIPLE SCAN*********/

/***** TIMERS Configuration *****/
/* Tim2 500 Hz timers interrupt*/
#define SRATE_T2 500
#define PERIOD_T2 39999
#define PRE_T2 3
#define T2_FREQ 80000000 // [Hz]
/* Tim2 1000 Hz timers interrupt*/
//#define SRATE_T2 1000
//#define PERIOD_T2 39999
//#define PRE_T2 1
//#define T2_FREQ 80000000 // [Hz]

/* Tim3 500 Hz timers interrupt*/
#define SRATE_T3 500
#define PERIOD_T3 39999
#define PRE_T3 3
#define T3_FREQ 80000000 // [Hz]

/* Tim4 100 Hz timers interrupt*/
//#define SRATE_T4 100
//#define PERIOD_T4 79999
//#define PRE_T4 9
//#define T4_FREQ 80000000 // [Hz]
/* Tim4 99.9998 Hz timers interrupt*/
//#define SRATE_T4 99.9998
//#define PERIOD_T4 57142
//#define PRE_T4 13
//#define T4_FREQ 80000000 // [Hz]
/* Tim4 125 Hz timers interrupt*/
#define SRATE_T4 125
#define PERIOD_T4 63999
#define PRE_T4 9
#define T4_FREQ 80000000 // [Hz]

// Maxon Actuator + Encoder Specifications
#define TORQUE_CONSTANT 14.6 // mNm/A
#define K_t 0.0146 // Nm/A the as above but expressed in Nm/A onsted of mNm/A
#define current_max 5 // A
#define GEARHEAD_EFFICIENCY 0.81 // efficiency of the selected gearhed
#define MOTOR_EFFICIENCY 0.9 // efficiency of the selected motor

#define CPT 500 // encoder counts per turn
#define REDUCTION 28 // reduction ration of the gearhead
#define ENCODER_MAX_COUNT CPT*4*REDUCTION // The timer register overflows every complete turn of the motor shaft
#define ENCODER_RES 0.0064 // [deg/counts] -> RES = 360[deg]/(CPT[counts]*4*GEARHEAD_REDUCTION)
#define TIM1_COUNTER_THRESHOLD 10000

// Load Cell Specifications
#define LC_V_FS 5 // [V] -> Load cell voltage full scale
#define LC_noLoad LC_V_FS/2 // [V] -> no load voltage output
#define LC_FS 250 // [N] -> Load cell force full scale
#define LC_sensitivity 0.0081 // [V/N] -> load cell sensitivity
#define d 0.019 // [m] distance between axis of rotation of the actuator and the axis of the load cell

// Escon current sensor specifications
#define CS_V_FS 5 // [V] -> current sensor voltage full scale
#define CS_V_noLoad CS_V_FS/2 // [V] -> current sensor no load output

/**** TYPEDEF ****/
typedef union
{
	uint32_t data_uint32;
	uint8_t data_uint8[SIZE_UINT32];
}uint32Touint8;

// typedefs
typedef enum{
	positive_trapeziod = 0,
	negative_trapeziod = 1,
	positive_triangle = 2,
	negative_triangle = 3,
	no_torque_wave = 4,
}wave_status;

typedef union{
	double data_double;
	uint8_t data_uint[TWOS_COMPL_ARRAY_SIZE];
}dataDoubleUint;

typedef enum{
	up = 0,
	steady = 1,
	down = 2,
	ready_for_next = 3,
	none = 4,
}state;

typedef enum{
	stop = 0,
	MM1 = 1,
	MM2 = 2,
	PC1 = 3,
	MM3 = 4,
}mode_selection;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
