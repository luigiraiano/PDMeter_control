/*
 * Luigi Raiano, v11, 10-07-2020
 *
 * SW: PDMeter_UI_v9
 *
 * EDIT 10-07-2020
 * 	- occhio sempre ai cavi e corretto collegamento
 * 	- v11 funziona in modo stabile con singolo timer (tim 2 a 500 Hz).
 * 	- v11 funziona in modo stabile con doppio timer (tim2 a 500 Hz per controllo e tim4 a 125 Hz per invio dati)
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

TIM_OC_InitTypeDef TimConfig;
TIM_OC_InitTypeDef sConfigOC;

uint8_t RxBuffer[n_bytes_uart_RX];
uint8_t TxBuffer_force[Tx_buffer_size_FS] = {1, 2};
uint8_t TxPRE = 85;
uint8_t TxDevID = 106; // uint8(j);
uint8_t TxData[Tx_total_data_size];
uint8_t TxLastByte = 1;

uint32_t duty = 0;
uint32_t value = 0;
double torque_steady = 0;
uint32_t duty_min = round(((PERIOD_T3+1)*10)/100 - 1); // 10%
uint32_t duty_max = round(((PERIOD_T3+1)*90)/100 - 1); // 90%

double current_provided = 0;

uint32Touint8 time_stmp;

// SPI
uint8_t RxSPIData[4] = {0, 0, 0, 0};
uint8_t ADC1167_confByte = 0b00101111; // scan mode enabled from channel 0 to channel 1. Reference AVDD, internal clock mode.

// STATE FLAGS
bool Start = 0;
// inizializzo i due flags
state torque_state_flag = ready_for_next;
state next_torque_state_flag = up;
state angle_state_flag = ready_for_next;
state next_angle_state_flag = up;

wave_status wave_status_flag;

double delta_t_steady = 0.5; // [s]

mode_selection mode_selection_flag = stop;

double freq_IT_timer = (T2_FREQ)/((PRE_T2+1)*(PERIOD_T2+1)); // [Hz]
double period_IT_timer = 0;

uint16_t count_torque_generator = 0;
uint16_t count_angle_generator = 0;

bool flag_up = false;
bool flag_steady = false;
bool flag_down = false;

bool positive_wave_torque = false;
bool negative_wave_torque = false;

bool positive_wave_angle = false;
bool negative_wave_angle = false;

bool first_wave_ended = false;

uint16_t encoder_count = 0, encoder_count_zero = 0;
uint16_t encoder_count_pre = 0, encoder_count_pre_zero = 0;
uint16_t encoder_count_offset = 0, encoder_count_offset_zero = 0;
uint16_t timer_counts = 0;
uint16_t timer_counts_pre = 0;

int16_t encoder_direction;

int16_t count_encoder_overflow = 0, count_encoder_overflow_zero = 0;
uint16_t count_encoder_underflow = 0;

double tau_tbp = 0; // torque to be provided to the actuator in order to compensate the load

double theta_d_max = 0;
double theta_zero = 0;

double theta_current;
double theta_sum = 0;
double theta_old = 0;

double delta_t_steady_angle = 0.5; // [s]
double delta_t_angle = 5; // [s]

uint8_t n_perturbations = 1; // number of load perturbations to apply to the wrist
uint8_t count_perturbations_applied = 0;

// pid based pistion controller parameters
double K_p_PC1 = 0.2;
double K_d_PC1  = 0.0005;
double K_i_PC1 = 0.005;

double theta_err = 0;
double theta_e_d = 0, theta_e_i = 0;
double theta_e_old = 0;

double torque_max  = 50; // mNm (al motore)

uint8_t flag_mode = 0;

double delta_t_input;

/**INPUT PARAMETER**/
dataDoubleUint input_parameter_1,input_parameter_2;

/**OUTPUT PARAMETER**/
dataDoubleUint theta,theta_d,theta_e;
dataDoubleUint toruqe_tbp;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_Delay(__IO uint32_t Delay);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void trapezoid_torque_generator(); // Positive wave and negative wave
void inverse_trapezoid_torque_generator(); // Negative wave and positive wave
void triangular_torque_generator();
void trapezoid_position_controller();

void ReadEncoderData(TIM_HandleTypeDef *timer);
void ReadEncoderDataZero(TIM_HandleTypeDef *timer);
void Init_Encoder();

void Send_Data_UART(UART_HandleTypeDef* huart);

void MM1_Init(void);
void MM2_Init(void);
void MM3_Init(void);
void PC1_Init(void);
void Stop_Init();

void MM1_Controller(void);
void MM2_Controller(void);
void MM3_Controller(void);
void PC1_Controller(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  // HAL_GPIO_WritePin(PORT, PIN, STATE);
  HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET); // enable pin -> DISABLED
  HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_SET); // DIRECdiretionTION pin -> CCW
  HAL_GPIO_WritePin(MOT_DIN4_GPIO_Port, MOT_DIN4_Pin, GPIO_PIN_RESET); // escon speed out of range ((0 to 150*29) rpm)

  // Set SPI disabled
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // set CS pin high to disable SPI

  mode_selection_flag = stop;

  period_IT_timer = 1/freq_IT_timer;

  delta_t_input = delta_t_angle;

  TxData[0] = TxPRE;
  TxData[1] = TxDevID;
  TxData[Tx_total_data_size-1] = TxLastByte;

  time_stmp.data_uint32 = 0;

  Init_Encoder();

  // PWM INITIALIZATION -> set duty=0;
  pwm_init(&htim3, TIM_CHANNEL_1);
  HAL_Delay(100);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxBuffer, n_bytes_uart_RX);

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = ENCODER_MAX_COUNT;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = PRE_T2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PERIOD_T2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PRE_T3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PERIOD_T3;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = PRE_T4;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PERIOD_T4;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = BAUDRATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOT_ENABLE_Pin|MOT_DIRECTION_Pin|MOT_DIN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_EOC_Pin */
  GPIO_InitStruct.Pin = ADC_EOC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_EOC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_ENABLE_Pin MOT_DIRECTION_Pin MOT_DIN4_Pin */
  GPIO_InitStruct.Pin = MOT_ENABLE_Pin|MOT_DIRECTION_Pin|MOT_DIN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if(huart->Instance==USART2){
		HAL_TIM_Base_Stop_IT(&htim2); // DISABLE TIM2 interrupt
		HAL_TIM_Base_Stop_IT(&htim4); // DISABLE TIM2 interrupt

		HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxBuffer, n_bytes_uart_RX);

		// Init Encoder
		Init_Encoder();

		n_perturbations = RxBuffer[n_bytes_uart_RX-2];

		flag_mode = RxBuffer[n_bytes_uart_RX-1];
		switch(flag_mode)
		{

		case 115: // start MM1
		{
			MM1_Init();
			// Enable encoder mode
			HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
			// ENABLE TIM2 interrupt
			HAL_TIM_Base_Start_IT(&htim2);
			// ENABLE 4 interrupt
			HAL_TIM_Base_Start_IT(&htim4);
		}break;

		case 110: // start MM2
		{
			MM2_Init();
			// Enable encoder mode
			HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
			// ENABLE TIM2 interrupt
			HAL_TIM_Base_Start_IT(&htim2);
			// ENABLE 4 interrupt
			HAL_TIM_Base_Start_IT(&htim4);
		}break;

		case 114: // start MM3
		{
			MM3_Init();
			// Enable encoder mode
			HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
			// ENABLE TIM2 interrupt
			HAL_TIM_Base_Start_IT(&htim2);
			// ENABLE 4 interrupt
			HAL_TIM_Base_Start_IT(&htim4);
		}break;

		case 112: // start PC1
		{
			PC1_Init();
			// Enable encoder mode
			HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
			// ENABLE TIM2 interrupt
			HAL_TIM_Base_Start_IT(&htim2);
			// ENABLE 4 interrupt
			HAL_TIM_Base_Start_IT(&htim4);
		}break;

		case 116: // stop
		{
			Stop_Init();
		}break;

		} // end switch(flag_mode)

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	HAL_TIM_Base_Stop_IT(&htim2); // DISABLE TIM2 interrupt
//	HAL_TIM_Base_Stop_IT(&htim4); // DISABLE TIM2 interrupt

	if(htim->Instance==TIM2) // timer for executing the motor controller
	{
		if(Start)
		{
			time_stmp.data_uint32++;

			/**********SPI MULTIPLE SCAN*********/
			// read two data and store them in RxSPIData: 1- Force Sensor [0,1]; 2- Current Sensor [2,3]
			MAX1187_SPI_Scan(&hspi2, ADC1167_confByte, sizeof(RxSPIData), RxSPIData);

			// Read data from encoder
			ReadEncoderData(&htim1);

			// Set the command torque to the actuator
			switch(mode_selection_flag)
			{
			case MM1:
			{
				MM1_Controller();
			}break;

			case MM2:
			{
				MM2_Controller();
			}break;

			case PC1:
			{
				PC1_Controller();
			} break;

			case MM3:
			{
				MM3_Controller();
			} break;

			} // end switch mode_selection flag

			/****** Single TIMER config *****/
//			Send_Data_UART(&huart2);
			/****** Single TIMER config *****/
		}
		else if(!Start)
		{
			// Non invio pi� dati uart e disabilito escon
			HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET); // disable motor

			// disable TIM2 IT
			HAL_TIM_Base_Stop_IT(&htim2); // DISABLE TIM2 interrupt
		}
	} // end if

	if(htim->Instance==TIM4) // timer for sending data
		{
			if(Start)
			{
				Send_Data_UART(&huart2);
			}
			else if(!Start)
			{
				// Non invio piu' dati uart e disabilito escon
				HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET); // disable motor

				// disable TIM2 IT
				HAL_TIM_Base_Stop_IT(&htim4); // DISABLE TIM2 interrupt
			}
		} // end if

//	HAL_TIM_Base_Start_IT(&htim2); // ENABLE TIM2 interrupt
//	HAL_TIM_Base_Start_IT(&htim4); // ENABLE TIM2 interrupt
}

void Send_Data_UART(UART_HandleTypeDef* huart)
{
	/********UART COMMUNICATION*********/
	int k = 2; // if send 4-Byte timestamp

	// 1- time stamp
	for(int i = 0; i<SIZE_UINT32; i++)
	{
		TxData[k] = time_stmp.data_uint8[i];
		k++;
	}

	/**********SPI MULTIPLE SCAN*********/
	for(int i=0; i<(Tx_buffer_size_FS+Tx_buffer_size_curr); i++){
		TxData[k] = RxSPIData[i];
		//TxCheck = TxCheck + TxBuffer[i];
		k++;
	}
	/**********SPI MULTIPLE SCAN*********/

	// torque provided to the ESCON (double) (control torque)
	for(int i=0; i<TWOS_COMPL_ARRAY_SIZE; i++){
		TxData[k] = toruqe_tbp.data_uint[i];
		k++;
	}

	// Angle read by the encoder
	//			k = first_ThetaTx_datum_index;
	for(int i=0; i<TWOS_COMPL_ARRAY_SIZE; i++){
		TxData[k] = theta.data_uint[i];
		k++;
	}

	/*****NEW*****/
	// Theta_e -> error between theta_d and theta measured ([deg]). This value is different to zero only when PC1 is selected
	for(int i=0; i<TWOS_COMPL_ARRAY_SIZE; i++)
	{
		TxData[k] = theta_d.data_uint[i];
		k++;
	}
	/*****NEW*****/

	// Set Last byte equal to TxLastByte for parsing purposes
	TxData[Tx_total_data_size-1] = TxLastByte;

	// transmit data over uart2
	HAL_UART_Transmit(huart, (uint8_t *)&TxData, Tx_total_data_size, 0xFF);
	/********UART COMMUNICATION*********/
}

void MM1_Init(void)
{
	//			int i;
	//			for(i=0; i<((n_bytes_uart_RX-2)/sizeof(double)); ++i) {
	//				torque_steady = ((double*)RxBuffer)[i]; // mNm
	//			}

	int k = 0;
	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_1.data_uint[i] = RxBuffer[k];
		k++;
	}

	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_2.data_uint[i] = RxBuffer[k];
		k++;
	}

	torque_steady = input_parameter_1.data_double;
	delta_t_input = input_parameter_2.data_double;

	HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET); // enable motor

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // turn on led

	// Set SPI disabled
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // set CS pin high to disable SPI

	torque_state_flag = ready_for_next;
	next_torque_state_flag = up;
	wave_status_flag = positive_trapeziod;
	positive_wave_torque = true;
	negative_wave_torque = false;

	angle_state_flag = none;
	next_angle_state_flag = none;
	positive_wave_angle = false;
	negative_wave_angle = false;

	mode_selection_flag = MM1;

	tau_tbp = 0;

	time_stmp.data_uint32 = 0;

	Start = 1;

	Init_Encoder();

	count_perturbations_applied = 0;
}

void MM2_Init(void)
{
	// devo assegnare questo valore come valore di regime

	// in alternativa potrei usare la variabile union
	//			n_perturbations = RxBuffer[n_bytes_uart_RX-2];

	//			int i;
	//			for(i=0; i<((n_bytes_uart_RX-2)/sizeof(double)); ++i) {
	//				torque_steady = ((double*)RxBuffer)[i]; // mNm
	//			}

	int k = 0;
	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_1.data_uint[i] = RxBuffer[k];
		k++;
	}

	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_2.data_uint[i] = RxBuffer[k];
		k++;
	}

	torque_steady = input_parameter_1.data_double;
	delta_t_input = input_parameter_2.data_double;

	HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET); // enable motor

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	// Set SPI disabled
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // set CS pin high to disable SPI

	torque_state_flag = ready_for_next;
	next_torque_state_flag = down;
	wave_status_flag = negative_trapeziod;
	positive_wave_torque = false;
	negative_wave_torque = true;

	angle_state_flag = none;
	next_angle_state_flag = none;
	positive_wave_angle = false;
	negative_wave_angle = false;

	mode_selection_flag = MM2;

	tau_tbp = 0;

	time_stmp.data_uint32 = 0;

	Start = 1;

	Init_Encoder();

	count_perturbations_applied = 0;
}

void MM3_Init(void)
{
	//			int i;
	//			for(i=0; i<((n_bytes_uart_RX-2)/sizeof(double)); ++i)
	//			{
	//				torque_steady = ((double*)RxBuffer)[i]; // mNm
	//			}

	int k = 0;
	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_1.data_uint[i] = RxBuffer[k];
		k++;
	}

	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_2.data_uint[i] = RxBuffer[k];
		k++;
	}

	torque_steady = input_parameter_1.data_double;
	delta_t_input = input_parameter_2.data_double;

	HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET); // enable motor

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	// Set SPI disabled
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // set CS pin high to disable SPI

	torque_state_flag = ready_for_next;
	next_torque_state_flag = up;
	wave_status_flag = positive_triangle;
	positive_wave_torque = true;
	negative_wave_torque = false;

	angle_state_flag = none;
	next_angle_state_flag = none;
	positive_wave_angle = false;
	negative_wave_angle = false;

	mode_selection_flag = MM3;

	tau_tbp = 0;

	time_stmp.data_uint32 = 0;

	Start = 1;

	count_perturbations_applied = 0;

	Init_Encoder();
}

void PC1_Init(void)
{
	//			// new in version 6
	//			int i;
	//			for(i=0; i<((n_bytes_uart_RX-2)/sizeof(double)); ++i) {
	//				theta_d_max = ((double*)RxBuffer)[i]; // mNm
	//			}
	int k = 0;
	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_1.data_uint[i] = RxBuffer[k];
		k++;
	}

	for(int i=0; i<sizeof(double); i++)
	{
		input_parameter_2.data_uint[i] = RxBuffer[k];
		k++;
	}

	theta_d_max = input_parameter_1.data_double;
	delta_t_input = input_parameter_2.data_double;

	toruqe_tbp.data_double = 0;
	tau_tbp = 0;

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	change_pwm_duty(duty_min, &htim3, TIM_CHANNEL_1);

	HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET); // enable motor
	HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_SET); // set ccw

	// Set SPI disabled
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // set CS pin high to disable SPI

	time_stmp.data_uint32 = 0;

	torque_state_flag = none;
	next_torque_state_flag = none;
	wave_status_flag = no_torque_wave;
	positive_wave_torque = false;
	negative_wave_torque = false;

	angle_state_flag = ready_for_next;
	next_angle_state_flag = up;
	positive_wave_angle = true;
	negative_wave_angle = false;

	mode_selection_flag = PC1;

	Start = 1;

	Init_Encoder();

	count_perturbations_applied = 0;
}

void Stop_Init()
{
	//value = duty_min;
	//			change_pwm_duty(duty_min, &htim2, TIM_CHANNEL_1);
	change_pwm_duty(duty_min, &htim3, TIM_CHANNEL_1);

	HAL_TIM_Base_Stop_IT(&htim2); // enable TIM2 interrupt
	HAL_TIM_Base_Stop_IT(&htim4); // ENABLE 4 interrupt

	HAL_TIM_Encoder_Stop(&htim1,TIM_CHANNEL_ALL);

	HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET); // disable motor

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	toruqe_tbp.data_double = 0;
	HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_SET); // set ccw

	// Set SPI disabled
	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET); // set CS pin high to disable SPI


	mode_selection_flag = stop;

	Start = 0;

	torque_state_flag = none;
	next_torque_state_flag = none;

	wave_status_flag = positive_trapeziod;

	positive_wave_torque = false;
	negative_wave_torque = false;

	count_encoder_overflow = 0;
	count_encoder_underflow = 0;
}

void Init_Encoder()
{
	// Encoder mode init
	htim1.Instance->CNT = 0;
	encoder_count_offset = __HAL_TIM_DIRECTION_STATUS(&htim1);
	timer_counts = __HAL_TIM_GetCounter(&htim1);
	encoder_count = 0;
	encoder_count_pre = 0;
	count_encoder_overflow = 0;
	count_encoder_underflow = 0;

	// Define the initial position
	theta_zero = 0;

	theta.data_double = 0;
	theta_d.data_double = theta_zero;
	theta_e_old = 0;

	// Initialize theta error, its derivative and its integral
	theta_e.data_double= 0;
	theta_err = 0;
	theta_e_d = 0;
	theta_e_i = 0;
}

void ReadEncoderDataZero(TIM_HandleTypeDef *timer)
{

	encoder_count_zero = timer->Instance->CNT;

	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(timer)==0) // upward -> timer counter is encreasing -> CCW
	{
		if(encoder_count_zero - encoder_count_pre_zero < -TIM1_COUNTER_THRESHOLD){
			count_encoder_overflow_zero++;
		}
	}
	else // downward -> timer counter is decreasing -> CW
	{
		if(encoder_count_zero - encoder_count_pre_zero > TIM1_COUNTER_THRESHOLD){
			count_encoder_overflow_zero--;
		}
	}

	theta_zero = (double) -((encoder_count_zero*ENCODER_RES) + count_encoder_overflow_zero*ENCODER_MAX_COUNT*ENCODER_RES); // differential evaluation with respect the previous angle

	encoder_count_pre_zero = encoder_count_zero;
}

void ReadEncoderData(TIM_HandleTypeDef *timer)
{
	encoder_count = timer->Instance->CNT;

	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(timer)==0) // upward -> timer counter is increasing -> CCW
	{
		if(encoder_count - encoder_count_pre < -TIM1_COUNTER_THRESHOLD){
			count_encoder_overflow++;
		}
	}
	else // downward -> timer counter is decreasing -> CW
	{
		if(encoder_count - encoder_count_pre > TIM1_COUNTER_THRESHOLD){
			count_encoder_overflow--;
		}
	}

	theta.data_double = (double) -((encoder_count*ENCODER_RES) + count_encoder_overflow*ENCODER_MAX_COUNT*ENCODER_RES); // differential evaluation with respect the previous angle

	encoder_count_pre = encoder_count;
}

void trapezoid_position_controller()
{
	if(angle_state_flag == ready_for_next)
	{
		if(next_angle_state_flag == steady)
		{
			angle_state_flag = steady;
		}
		else if(next_angle_state_flag == down)
		{
			angle_state_flag = down;
		}
		else if(next_angle_state_flag == up)
		{
			angle_state_flag = up;
		}
	}

	// Extension
	if(positive_wave_angle == true && negative_wave_angle == false)
	{
		switch(angle_state_flag)
		{
		case up:
		{
			if(theta_d.data_double < theta_d_max)
			{
				theta_d.data_double = theta_d.data_double + (theta_d_max/delta_t_input)*period_IT_timer;
			}
			else
			{
				theta_d.data_double = theta_d_max;

				angle_state_flag = ready_for_next;
				next_angle_state_flag = steady;

				count_angle_generator = 0;
			}
		} break;

		case steady:
		{
			if((count_angle_generator < (freq_IT_timer*delta_t_steady_angle)))
			{
				theta_d.data_double = theta_d_max;
				count_angle_generator = count_angle_generator + 1;
			}
			else
			{
				theta_d.data_double = theta_d_max;

				angle_state_flag = ready_for_next;
				next_angle_state_flag = down;

				count_angle_generator = 0;
			}
		} break;

		case down:
		{
			if(theta_d.data_double > theta_zero)
			{
				theta_d.data_double = theta_d.data_double - (theta_d_max/delta_t_input)*period_IT_timer;
			}
			else
			{
				theta_d.data_double = theta_zero;

				angle_state_flag = ready_for_next;
				next_angle_state_flag = down;

				count_angle_generator = 0;

				positive_wave_angle = false;
				negative_wave_angle = true;

				//timer->stop();
			}
		} break;
		}
	}
	else if(positive_wave_angle == false && negative_wave_angle == true) // flexion
	{
		switch(angle_state_flag)
		{
		case down:
		{
			if(theta_d.data_double > (-theta_d_max))
			{
				theta_d.data_double = theta_d.data_double - (theta_d_max/delta_t_input)*period_IT_timer;
			}
			else
			{
				theta_d.data_double = -theta_d_max;

				angle_state_flag = ready_for_next;
				next_angle_state_flag = steady;

				count_angle_generator = 0;
			}
		} break;

		case steady:
		{
			if((count_angle_generator < (freq_IT_timer*delta_t_steady_angle)))
			{
				theta_d.data_double = -theta_d_max;
				count_angle_generator = count_angle_generator + 1;
			}
			else
			{
				theta_d.data_double = -theta_d_max;

				angle_state_flag = ready_for_next;
				next_angle_state_flag = up;

				count_angle_generator = 0;
			}
		} break;

		case up:
		{
			if(theta_d.data_double < theta_zero)
			{
				theta_d.data_double = theta_d.data_double + (theta_d_max/delta_t_input)*period_IT_timer;
				//count_torque_generator = count_torque_generator + 1;
			}
			else if(theta_d.data_double >= theta_zero)
			{
				theta_d.data_double = theta_zero;

				angle_state_flag = ready_for_next;
				next_angle_state_flag = up;

				count_torque_generator = 0;

				/************NUOVO***********/
				count_perturbations_applied = count_perturbations_applied + 1;
				if(count_perturbations_applied >= n_perturbations)
				{
					positive_wave_angle = false;
					negative_wave_angle = false;

					count_perturbations_applied = 0;

					Start = 0;
				}
				else if(count_perturbations_applied < n_perturbations)
				{
					positive_wave_angle = true;
					negative_wave_angle = false;

					Start = 1;
				}
			}
		} break;

		}
	}
}

void trapezoid_torque_generator()
{
	if(torque_state_flag == ready_for_next)
	{
		if(next_torque_state_flag == steady)
		{
			torque_state_flag = steady;
		}
		else if(next_torque_state_flag == down)
		{
			torque_state_flag = down;
		}
		else if(next_torque_state_flag == up)
		{
			torque_state_flag = up;
		}
	}

	if(positive_wave_torque==true && negative_wave_torque==false)
	{

		// Behavior of the wave: up->steady->down
		switch (torque_state_flag)
		{
		case up:
		{
			//if((count_torque_generator < freq_IT_timer*delta_t_input) || (u.double_torque < steady_torque))
			if(toruqe_tbp.data_double < torque_steady)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double + (torque_steady/delta_t_input)*period_IT_timer;
				//count_torque_generator = count_torque_generator + 1;
			}
			else
			{
				toruqe_tbp.data_double = torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = steady;

				count_torque_generator = 0;
			}
		} break;
		case steady:
		{
			if((count_torque_generator < (freq_IT_timer*delta_t_steady)))
			{
				toruqe_tbp.data_double = torque_steady;
				count_torque_generator = count_torque_generator + 1;
			}
			else
			{
				toruqe_tbp.data_double = torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = down;

				count_torque_generator = 0;
			}
		} break;
		case down:
		{
			if(toruqe_tbp.data_double > 0)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double - (torque_steady/delta_t_input)*period_IT_timer;
			}
			else
			{
				toruqe_tbp.data_double = 0;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = down;

				count_torque_generator = 0;

				positive_wave_torque = false;
				negative_wave_torque = true;
			}
		} break;
		case none:
		{
			positive_wave_torque = false;
		} break;

		}
	}
	else if(positive_wave_torque==false && negative_wave_torque==true)
	{
		// torque wave behavior: down->(-steady)->up
		switch (torque_state_flag)
		{
		case down:
		{
			if(toruqe_tbp.data_double > (-torque_steady))
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double - (torque_steady/delta_t_input)*period_IT_timer;
			}
			else
			{
				toruqe_tbp.data_double = -torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = steady;

				count_torque_generator = 0;
			}
		} break;

		case steady:
		{
			if((count_torque_generator < (freq_IT_timer*delta_t_steady)))
			{
				toruqe_tbp.data_double = -torque_steady;
				count_torque_generator = count_torque_generator + 1;
			}
			else
			{
				toruqe_tbp.data_double = -torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = up;

				count_torque_generator = 0;
			}
		} break;

		case up:
		{
			if(toruqe_tbp.data_double < 0)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double + (torque_steady/delta_t_input)*period_IT_timer;
			}
			else if(toruqe_tbp.data_double >= 0)
			{
				toruqe_tbp.data_double = 0;

				torque_state_flag = ready_for_next;

				count_torque_generator = 0;

				wave_status_flag = positive_trapeziod;
				next_torque_state_flag = up;

				/************NUOVO***********/
				count_perturbations_applied = count_perturbations_applied + 1;
				if(count_perturbations_applied >= n_perturbations)
				{
					positive_wave_torque = false;
					negative_wave_torque = false;

					count_perturbations_applied = 0;

					Start = 0;
				}
				else if(count_perturbations_applied < n_perturbations)
				{
					positive_wave_torque = true;
					negative_wave_torque = false;

					Start = 1;
				}
				/************NUOVO***********/

			}
		} break;

		}
	}
}

void inverse_trapezoid_torque_generator()
{
	if(torque_state_flag == ready_for_next)
	{
		if(next_torque_state_flag == steady){
			torque_state_flag = steady;
		}
		else if(next_torque_state_flag == down){
			torque_state_flag = down;
		}
		else if(next_torque_state_flag == up){
			torque_state_flag = up;
		}
	}

	if(negative_wave_torque==true && positive_wave_torque==false)
	{
		// torque wave behavior: down->(-steady)->up
		switch (torque_state_flag)
		{
		case down:
		{
			if(toruqe_tbp.data_double > (-torque_steady))
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double - (torque_steady/delta_t_input)*period_IT_timer;
			}
			else{
				toruqe_tbp.data_double = -torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = steady;

				count_torque_generator = 0;
			}
		} break;

		case steady:
		{
			if((count_torque_generator < (freq_IT_timer*delta_t_steady)))
			{
				toruqe_tbp.data_double = -torque_steady;
				count_torque_generator = count_torque_generator + 1;
			}
			else
			{
				toruqe_tbp.data_double = -torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = up;

				count_torque_generator = 0;
			}
		} break;

		case up:
		{
			if(toruqe_tbp.data_double < 0)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double + (torque_steady/delta_t_input)*period_IT_timer;
			}
			else if(toruqe_tbp.data_double >= 0)
			{
				toruqe_tbp.data_double = 0;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = up;

				count_torque_generator = 0;

				positive_wave_torque = true; // if false -> the torque generator stops
				negative_wave_torque = false;
			}
		} break;

		case none:
		{
			negative_wave_torque = false;
		} break;

		}
	}
	else if(positive_wave_torque==true && negative_wave_torque==false)
	{

		// Behavior of the wave: up->steady->down
		switch (torque_state_flag)
		{
		case up:
		{
			if(toruqe_tbp.data_double < torque_steady)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double + (torque_steady/delta_t_input)*period_IT_timer;
			}
			else
			{
				toruqe_tbp.data_double = torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = steady;

				count_torque_generator = 0;
			}
		} break;
		case steady:
		{
			if((count_torque_generator < (freq_IT_timer*delta_t_steady))){
				toruqe_tbp.data_double = torque_steady;
				count_torque_generator = count_torque_generator + 1;
			}
			else{
				toruqe_tbp.data_double = torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = down;

				count_torque_generator = 0;
			}
		} break;
		case down:
		{
			if(toruqe_tbp.data_double > 0)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double - (torque_steady/delta_t_input)*period_IT_timer;
			}
			else{
				toruqe_tbp.data_double = 0;

				torque_state_flag = ready_for_next;

				count_torque_generator = 0;

				wave_status_flag = negative_trapeziod;
				next_torque_state_flag = down;


				/************NUOVO***********/
				count_perturbations_applied = count_perturbations_applied + 1;
				if(count_perturbations_applied >= n_perturbations)
				{
					negative_wave_torque = false;
					positive_wave_torque = false;

					count_perturbations_applied = 0;

					Start = 0;
				}
				else if(count_perturbations_applied < n_perturbations)
				{
					negative_wave_torque = true;
					positive_wave_torque = false;

					Start = 1;
				}
				/************NUOVO***********/
			}
		} break;
		case none:
		{
			positive_wave_torque = false;
		} break;

		}
	}
}

void triangular_torque_generator()
{
	if(next_torque_state_flag == down)
	{
		torque_state_flag = down;
	}
	else if(next_torque_state_flag == up)
	{
		torque_state_flag = up;
	}

	if(positive_wave_torque==true && negative_wave_torque==false)
	{
		switch (torque_state_flag)
		{

		case up:
		{
			if(toruqe_tbp.data_double < torque_steady)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double + (torque_steady/delta_t_input)*period_IT_timer;
			}
			else
			{
				toruqe_tbp.data_double = torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = down;

				count_torque_generator = 0;
			}
		} break;

		case down:
		{
			if(toruqe_tbp.data_double > 0)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double - (torque_steady/delta_t_input)*period_IT_timer;
			}
			else
			{
				toruqe_tbp.data_double = 0;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = down;

				count_torque_generator = 0;

				positive_wave_torque = false;
				negative_wave_torque = true;
				// ending of the first half wave (positive half-wave)
			}
		} break;
		} // end switch torque_state_flag
	} // end if
	else if(positive_wave_torque==false && negative_wave_torque==true)
	{
		switch (torque_state_flag)
		{
		case down:
		{
			if(toruqe_tbp.data_double > (-torque_steady))
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double - (torque_steady/delta_t_input)*period_IT_timer;
				//count_torque_generator = count_torque_generator + 1;
			}
			else
			{
				toruqe_tbp.data_double = -torque_steady;

				torque_state_flag = ready_for_next;
				next_torque_state_flag = up;

				count_torque_generator = 0;
			}
		} break;
		case up:
		{
			if(toruqe_tbp.data_double < 0)
			{
				toruqe_tbp.data_double = toruqe_tbp.data_double + (torque_steady/delta_t_input)*period_IT_timer;
			}
			else if(toruqe_tbp.data_double >= 0)
			{
				toruqe_tbp.data_double = 0;

				torque_state_flag = ready_for_next;

				count_torque_generator = 0;

				wave_status_flag = positive_triangle;
				next_torque_state_flag = up;

				count_perturbations_applied = count_perturbations_applied + 1;
				if(count_perturbations_applied >= n_perturbations)
				{
					positive_wave_torque = false;
					negative_wave_torque = false;

					count_perturbations_applied = 0;

					Start = 0;
				}
				else if(count_perturbations_applied < n_perturbations)
				{
					positive_wave_torque = true;
					negative_wave_torque = false;

					Start = 1;
				}
			}
		} break;
		} // end switch torque_state_flag
	} // end else if

}

void MM1_Controller(void)
{
	trapezoid_torque_generator();

	// define the current to provide to the escon
	if(toruqe_tbp.data_double >= 0)
	{
		current_provided = (toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}
	else
	{
		current_provided = (-toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}

	// Transform the current into pwm signal to control the ESCON
	value = (uint32_t) ( ( (duty_max - duty_min)/current_max)*current_provided +  duty_min );

	// Asses the sign of the torque to control the ESCON
	if(toruqe_tbp.data_double>=0)
	{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_SET); // set ccw
	}
	else{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_RESET); // set cw
	}
	// Set PWM's duty
	change_pwm_duty(value, &htim3, TIM_CHANNEL_1);
}

void MM2_Controller(void)
{
	inverse_trapezoid_torque_generator();

	// define the current to provide to the escon
	if(toruqe_tbp.data_double >= 0)
	{
		current_provided = (toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}
	else
	{
		current_provided = (-toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}

	// Transform the current into pwm signal to control the ESCON
	value = (uint32_t) ( ( (duty_max - duty_min)/current_max)*current_provided +  duty_min );

	// Asses the sign of the torque to control the ESCON
	if(toruqe_tbp.data_double>=0)
	{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_SET); // set ccw
	}
	else{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_RESET); // set cw
	}
	// Set PWM's duty
	change_pwm_duty(value, &htim3, TIM_CHANNEL_1);
}

void MM3_Controller(void)
{
	triangular_torque_generator();

	// define the current to provide to the escon
	if(toruqe_tbp.data_double >= 0)
	{
		current_provided = (toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}
	else
	{
		current_provided = (-toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}

	// Transform the current into pwm signal to control the ESCON
	value = (uint32_t) ( ( (duty_max - duty_min)/current_max)*current_provided +  duty_min );

	// Asses the sign of the torque to control the ESCON
	if(toruqe_tbp.data_double>=0)
	{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_SET); // set ccw
	}
	else{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_RESET); // set cw
	}
	// Set PWM's duty
	change_pwm_duty(value, &htim3, TIM_CHANNEL_1);
}

void PC1_Controller(void)
{
	// upgrade \theta_d(t)
	trapezoid_position_controller();

	theta_err = (double) theta_d.data_double - theta.data_double;
//	theta_e_d = (double) (theta_e.data_double - theta_e_old)/period_IT_timer;
	theta_e_d = (double) (theta_err - theta_e_old)/period_IT_timer;
	theta_e_i = theta_e_i + (theta_e.data_double)*period_IT_timer;


//	tau_tbp = K_p_PC1*theta_err; // P controller


//	tau_tbp = K_p_PC1*theta_err + K_d_PC1*theta_e_d; // PD controller
	//				tau_tbp = K_p_PC1*theta_err + K_i_PC1*theta_e_i; // PI controller

	tau_tbp = K_p_PC1*theta_err + K_d_PC1*theta_e_d + K_i_PC1*theta_e_i; // PID controller

	// metto una condizione che valuta il valore di coppia fornito
	if(tau_tbp >= torque_max*REDUCTION)
	{
		toruqe_tbp.data_double = torque_max*REDUCTION; //[mNm]
	}
	else if(tau_tbp <= -torque_max*REDUCTION)
	{
		toruqe_tbp.data_double = -torque_max*REDUCTION; //[mNm]
	}
	else
	{
		toruqe_tbp.data_double = (double) tau_tbp*1000; //[mNm]
	}

	if(toruqe_tbp.data_double >= 0)
	{
		current_provided = (toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}
	else
	{
		current_provided = (-toruqe_tbp.data_double)/(GEARHEAD_EFFICIENCY*MOTOR_EFFICIENCY*REDUCTION*TORQUE_CONSTANT); // [A]
	}

	// Transform the current into pwm signal to control the ESCON
	value = (uint32_t) ( ( (duty_max - duty_min)/current_max)*current_provided +  duty_min );

	// Asses the sign of the torque to control the ESCON
	if(toruqe_tbp.data_double>=0)
	{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_SET); // set ccw
	}
	else
	{
		HAL_GPIO_WritePin(MOT_DIRECTION_GPIO_Port, MOT_DIRECTION_Pin, GPIO_PIN_RESET); // set cw
	}

	// Set PWM's duty
	change_pwm_duty(value, &htim3, TIM_CHANNEL_1);

	theta_e_old = theta_err;
}



void HAL_Delay(__IO uint32_t Delay)
{
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = Delay;

	/* Add a period to guarantee minimum wait */
	if (wait < HAL_MAX_DELAY)
	{
		wait++;
	}

	while((HAL_GetTick() - tickstart) < wait)
	{
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	while(1)
	{
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
