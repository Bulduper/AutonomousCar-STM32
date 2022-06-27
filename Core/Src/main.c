/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stdio.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_A_PIN GPIO_PIN_12
#define TRIG_A_PORT GPIOB
#define TRIG_B_PIN GPIO_PIN_13
#define TRIG_B_PORT GPIOB
#define SW_PIN GPIO_PIN_15
#define SW_PORT GPIOB
#define ECHO_FR_PIN GPIO_PIN_0
#define ECHO_FR_PORT GPIOB
#define ECHO_MR_PIN GPIO_PIN_7
#define ECHO_MR_PORT GPIOA
#define ECHO_RR_PIN GPIO_PIN_6
#define ECHO_RR_PORT GPIOA
#define ECHO_FL_PIN GPIO_PIN_1
#define ECHO_FL_PORT GPIOB
#define ECHO_ML_PIN GPIO_PIN_8
#define ECHO_ML_PORT GPIOA
#define ECHO_RL_PIN GPIO_PIN_11
#define ECHO_RL_PORT GPIOA
#define SERVO_PIN GPIO_PIN_6
#define SERVO_PORT GPIOB
#define MOTOR_PWM_PIN GPIO_PIN_9
#define MOTOR_PWM_PORT GPIOB
#define DIR_A_PIN GPIO_PIN_8
#define DIR_A_PORT GPIOB
#define DIR_B_PIN GPIO_PIN_7
#define DIR_B_PORT GPIOB
#define V_BAT_PIN GPIO_PIN_5
#define V_BAT_PORT GPIOA
#define LED_1_PIN GPIO_PIN_14
#define LED_1_PORT GPIOC
#define LED_2_PIN GPIO_PIN_15
#define LED_2_PORT GPIOC
#define LED_3_PIN GPIO_PIN_13
#define LED_3_PORT GPIOC

#define UART_BUFFER_SIZE 255

#define SERVO_MIN 800
#define SERVO_MAX 2000

//according to motor datasheet 7260 = 4*1815 = 4*11*165 (ratio)
//empirically ~7470
//#define STEPS_PER_REV 1815*4
#define STEPS_PER_REV 7470

//empirically obtained
#define KP 0.15f
#define KI 0.0006f
#define KD 2.5f

#define MAX_SPEED 160.0f //[mmps]

#define TWO_PI 6.2831853f
#define R_WHEEL 32.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define constrain(x, a, b) (((x) > (b)) ? (b) : ( (x) < (a) ? (a) : (x)))
//#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
typedef struct HC_SR04{
	char placement [3];
	TIM_HandleTypeDef *timer;
	//uint32_t channel;
	HAL_TIM_ActiveChannel active_ch;
	uint8_t tim_ch;
	uint8_t it_ch;
	uint16_t trig_pin;
	uint8_t now_responding;
	uint32_t rising_time;
	uint16_t distance;
	uint8_t done;
}HC_SR04;

HC_SR04 sensor[6];
HC_SR04* cur_sensor;
char str[100];

uint8_t UART1_rxBuffer[UART_BUFFER_SIZE] = {0};

int16_t curPWM = 0;
float target_speed = 0.0f;
float speed_limit = MAX_SPEED;
float target_pos = 0.0f;
uint8_t await_distance = 0;
char distance_measurement[200];
int prevEnc = 0;
float curSpeed = 0.0f;
float curRps = 0.0f;
float curPos = 0.0f;

//speed regulator vars
float kP = KP;
float kI = KI;
float kD = KD;
float err_integral = 0.0f;
float prev_err= 0.0f;
float integral_limit = 2.0f;
float err_thr = 0.0f;
int minPWM = 20;
//position regulator vars
uint8_t posRegulationEnabled = 0;
float kP_pos = 15.0f;
float err_thr_pos = 5.0f;


//quadratic func coefficients
float a = 0.003f;
float b = 0.82f;
float c = -0.6767f;

float x;

uint16_t loop_cnt = 0;

uint16_t servoPWM;

uint8_t xD=0;

//battery voltage vars
float bat_voltage = -1.0f;
//empirically obtained
float voltage_coef = 3.99f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delayUs(uint16_t time);
void sendString(char* s);
void HCSR04_Read (uint8_t sensorID);
void readDistance();
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void sensorSetup();
void oneTurn();
void setDirection(int8_t dir);
float getCurrentSpeed(uint32_t dt_us);
void speedRegulator(uint32_t dt_us);
void positionRegulator();
void servoSetPWM(uint16_t value);
void servoSetAngle(int16_t angle);
float servoGetAngle();
void setTurnAngle(float wheel_angle);
void USER_UART_IRQHandler(UART_HandleTypeDef* huart);
void telemetryResponse();
void sensorResponse();
void interpretMessage(char*);
void parseBuffer(uint8_t);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);
void loop100Hz();
void loop1Hz();
void getBatVoltage();
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);

int _write(int file, char *ptr, int len);
int32_t MAP(int32_t au32_IN, int32_t au32_INmin, int32_t au32_INmax, int32_t au32_OUTmin, int32_t au32_OUTmax);


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	sensorSetup();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //ITM_Port32(31) = 1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //ITM_Port32(31) = 2;

  for(int i=0 ; i<6; i++)
  {
	  HAL_TIM_IC_Start_IT(sensor[i].timer, sensor[i].tim_ch);
  }

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, UART_BUFFER_SIZE);
  HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //readDistance();
	  //oneTurn();
	  //sprintf(str,"%d\t%d\t%d\t%d\t%d\t%d\n",sensor[0].distance,sensor[1].distance,sensor[2].distance,sensor[3].distance,sensor[4].distance,sensor[5].distance);
	  //sprintf(str,"target mm/s: %f\tcurrent mm/s: %f\t current rps: %f\t current PWM: %d\n",target_speed,curSpeed,curRps,curPWM);
	  //sendString(str);

	  //TIME CONSUMING FUNCTIONS
		sensorResponse();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1000000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1000-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 255;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void delayUs(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)< time);
}

void sendString(char* s)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 1000);
}

void HCSR04_Read (uint8_t sensorID)
{
	//set which sensor is current
	cur_sensor = &sensor[sensorID];
	cur_sensor->done = 0;

	HAL_GPIO_WritePin(TRIG_A_PORT, cur_sensor->trig_pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delayUs(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_A_PORT, cur_sensor->trig_pin, GPIO_PIN_RESET);  // pull the TRIG pin low


	__HAL_TIM_ENABLE_IT(cur_sensor->timer, cur_sensor->it_ch);
	uint32_t start_time = HAL_GetTick();
	while(!(cur_sensor->done) && (HAL_GetTick()-start_time)<100){};
	//HAL_Delay(100);
	__HAL_TIM_DISABLE_IT(cur_sensor->timer, cur_sensor->it_ch);
	__HAL_TIM_SET_CAPTUREPOLARITY(cur_sensor->timer,cur_sensor->tim_ch,TIM_INPUTCHANNELPOLARITY_RISING);
	cur_sensor->now_responding = 0;
	cur_sensor->rising_time = 0;
}

void readDistance()
{
	//start input capture for every sensor -it does not make it enabled yet, it's for reset purposes
	for(int i=0 ; i<4; i++)
	{
	  HAL_TIM_IC_Start_IT(sensor[i].timer, sensor[i].tim_ch);

	}

	for(int i = 0; i<6; i++)
	{
		HCSR04_Read(i);
		HAL_Delay(10);
	}

	for(int i=0 ; i<4; i++)
	{
	  HAL_TIM_IC_Stop_IT(sensor[i].timer, sensor[i].tim_ch);
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//this solution is ok, it works
	//but i would prefer to make it work dynamically, depending on tim&channel it could obtain sensor id and calculate dist
	//and not depending on which sensor we are now expecting
	//it would be faster then

	if(!(htim == cur_sensor->timer && htim->Channel ==cur_sensor->active_ch)) return;
	//if Rising edge
	if(!cur_sensor->now_responding)
	{
		cur_sensor->rising_time = HAL_TIM_ReadCapturedValue(cur_sensor->timer, cur_sensor->tim_ch);
		__HAL_TIM_SET_CAPTUREPOLARITY(cur_sensor->timer, cur_sensor->tim_ch,TIM_INPUTCHANNELPOLARITY_FALLING);
		cur_sensor->now_responding = 1;
	}
	else
	{
		uint32_t fallingTime = HAL_TIM_ReadCapturedValue(cur_sensor->timer, cur_sensor->tim_ch);
		if(cur_sensor->rising_time<fallingTime)
		{
			cur_sensor->distance = (fallingTime - cur_sensor->rising_time)/58;
		}
		else{
			cur_sensor->distance = (cur_sensor->timer->Init.Period - cur_sensor->rising_time + fallingTime)/58;
		}

//		__HAL_TIM_SET_CAPTUREPOLARITY(cur_sensor->timer,cur_sensor->tim_ch,TIM_INPUTCHANNELPOLARITY_RISING);
//		cur_sensor->now_responding = 0;
//		__HAL_TIM_DISABLE_IT(cur_sensor->timer, cur_sensor->it_ch);
		cur_sensor->done = 1;
	}

}


void sensorSetup()
{
	strcpy(sensor[0].placement,"FR");
	sensor[0].timer = &htim3;
	sensor[0].active_ch = HAL_TIM_ACTIVE_CHANNEL_3;
	sensor[0].tim_ch = TIM_CHANNEL_3;
	sensor[0].it_ch = TIM_IT_CC3;
	sensor[0].trig_pin = TRIG_A_PIN;

	strcpy(sensor[1].placement,"MR");
	sensor[1].timer = &htim3;
	sensor[1].active_ch = HAL_TIM_ACTIVE_CHANNEL_2;
	sensor[1].tim_ch = TIM_CHANNEL_2;
	sensor[1].it_ch = TIM_IT_CC2;
	sensor[1].trig_pin = TRIG_A_PIN;

	strcpy(sensor[2].placement,"RR");
	sensor[2].timer = &htim3;
	sensor[2].active_ch = HAL_TIM_ACTIVE_CHANNEL_1;
	sensor[2].tim_ch = TIM_CHANNEL_1;
	sensor[2].it_ch = TIM_IT_CC1;
	sensor[2].trig_pin = TRIG_A_PIN;

	strcpy(sensor[3].placement,"FL");
	sensor[3].timer = &htim3;
	sensor[3].active_ch = HAL_TIM_ACTIVE_CHANNEL_4;
	sensor[3].tim_ch = TIM_CHANNEL_4;
	sensor[3].it_ch = TIM_IT_CC4;
	sensor[3].trig_pin = TRIG_B_PIN;

	strcpy(sensor[4].placement,"ML");
	sensor[4].timer = &htim1;
	sensor[4].active_ch = HAL_TIM_ACTIVE_CHANNEL_1;
	sensor[4].tim_ch = TIM_CHANNEL_1;
	sensor[4].it_ch = TIM_IT_CC1;
	sensor[4].trig_pin = TRIG_B_PIN;

	strcpy(sensor[5].placement,"RL");
	sensor[5].timer = &htim1;
	sensor[5].active_ch = HAL_TIM_ACTIVE_CHANNEL_4;
	sensor[5].tim_ch = TIM_CHANNEL_4;
	sensor[5].it_ch = TIM_IT_CC4;
	sensor[5].trig_pin = TRIG_B_PIN;
}

//int target = STEPS_PER_REV;

//int thr = 100;
//float P = 1.0f;
//void oneTurn()
//{
//	uint32_t start = TIM2->CNT;
//
//	int error = target - TIM2->CNT;
//
//	if(error>thr)
//	{
//		setDirection(1);
//	}
//	else if(error<-thr){
//		setDirection(-1);
//	}
//	else{
//		setDirection(0);
//	}
//	error = abs(error);
//	int PWM = constrain(P*error,0,255);
//	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,PWM);
////	while(abs(TIM2->CNT-start)<=STEPS_PER_REV)
////	{
////		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,50);
////	}
////	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,0);
//
//}


void setDirection(int8_t dir)
{
	if(dir>0)
	{
		//forward
		HAL_GPIO_WritePin(DIR_A_PORT, DIR_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_B_PORT, DIR_B_PIN, GPIO_PIN_RESET);
	}
	else if(dir<0)
	{
		//backward
		HAL_GPIO_WritePin(DIR_A_PORT, DIR_A_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR_B_PORT, DIR_B_PIN, GPIO_PIN_SET);
	}
	else
	{
		//stop
		HAL_GPIO_WritePin(DIR_A_PORT, DIR_A_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR_B_PORT, DIR_B_PIN, GPIO_PIN_RESET);
	}
}

float getCurrentSpeed(uint32_t dt_us)
{
	int currentEnc = TIM2->CNT;
	float rotpos = (int)(currentEnc)/(float)STEPS_PER_REV;
	float rps = ((int)(currentEnc - prevEnc))/((float)dt_us*0.000001f)/(float)STEPS_PER_REV;
	curRps = rps;
	prevEnc = currentEnc;
	//omega * 2*PI*r [mm/s]
	curSpeed = rps*TWO_PI*R_WHEEL;
	curPos = rotpos*TWO_PI*R_WHEEL;
	return rps*TWO_PI*R_WHEEL;
}

void speedRegulator(uint32_t dt_us)
{
	float current_speed = getCurrentSpeed(dt_us);
	float error = target_speed - current_speed;
	if(fabs(error)<err_thr)error = 0.0f;
	err_integral = constrain(err_integral+error,-integral_limit/kI,integral_limit/kI);
	float change = kP*error + kI*err_integral + kD * (error-prev_err);
	prev_err = error;

	int newPWM = curPWM + change;
	if(abs(newPWM)<minPWM)newPWM = 0.0f;
	curPWM = constrain(newPWM,-255,255);
	newPWM = constrain(newPWM,-255,255);
	if(newPWM>0.0f) setDirection(1);
	else if(newPWM<0.0f) setDirection(-1);
	newPWM = abs(newPWM);


	//set motor pwm 0-255
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,newPWM);


}

void positionRegulator()
{
	float error = target_pos - curPos;
	if(fabs(error)<err_thr_pos){
		error = 0.0f;
		posRegulationEnabled = 0;
	}
	target_speed = constrain(error * kP_pos,-speed_limit, speed_limit);
}


void servoSetPWM(uint16_t value)
{
	value = constrain(value,SERVO_MIN,SERVO_MAX);
	servoPWM = value;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,value);
}

void servoSetAngle(int16_t angle)
{
	int pwm = MAP(angle, -90, 90, 585, 2415);
	servoSetPWM(pwm);
}

float servoGetAngle()
{
	return MAP(htim4.Instance->CCR1, 585, 2415, -90, 90);
}

void setTurnAngle(float wheel_angle)
{
	if(wheel_angle<(-b*b/(4*a)+c))wheel_angle = (-b*b/(4*a)+c);
	//if(wh>=(b/2*a))return sqrt((wheel_angle-c)/a + b*b/(4*a*a)) - b/(2*a);
	x = sqrt((wheel_angle-c)/a + b*b/(4*a*a)) - b/(2*a);
	servoSetAngle((int16_t)x);
}

float getTurnAngle()
{
	float x = servoGetAngle();
	return a*x*x + b*x + c;
}




//this is an UART interrupt
void USER_UART_IRQHandler(UART_HandleTypeDef* huart)
{
	//we check if it is from uart1
	if(huart1.Instance == USART1)
	{
		//we check if it is idle state interrupt (the transmission is no longer continued)
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
		{
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			//UART1 Idle IQR Detected
			//trigger callback function
			USER_UART_IDLECallback(huart);
		}
	}
}

void telemetryResponse()
{
	sprintf(str,"{\"speed\":%.1f, \"angle\":%d, \"speed_limit\":%.1f,\"bat_vol\":%.1f, \"pos_reg\":%d}\n",curSpeed,(int16_t)getTurnAngle(),speed_limit,bat_voltage,posRegulationEnabled);
	sendString(str);
}

void sensorResponse()
{
	if(!await_distance)return;
	uint8_t samples = await_distance;
	uint16_t dist_buf [6] = {0};
	while(await_distance)
	{
		readDistance();
		for(int i=0 ;i<6; i++)
		{
			dist_buf[i]+=sensor[i].distance;
		}
		await_distance--;
	}
	sprintf(str,"{\"sensors\":[%d,%d,%d,%d,%d,%d]}\n",dist_buf[0]/samples,dist_buf[1]/samples,dist_buf[2]/samples,dist_buf[3]/samples,dist_buf[4]/samples,dist_buf[5]/samples);
	sendString(str);
}

void interpretMessage(char* message)
{
	char command = message[0];
	float value = atof(message+1);
	switch(command)
	{
	//turn
	case 'T':
	{
		setTurnAngle(value);
		break;
	}
	//speed
	case 'S':
	{
		posRegulationEnabled = 0;
		target_speed = constrain(value,-speed_limit, speed_limit);
		break;
	}
	//position
	case 'P':
	{
		posRegulationEnabled = 1;
		target_pos = curPos + value;
		break;
	}
	//speed limit
	case 's':
	{
		if(value<=0.1f)speed_limit=MAX_SPEED;
		else
		{
			speed_limit = value;
			target_speed = constrain(target_speed,-speed_limit, speed_limit);
		}
		break;
	}
	//environment
	case 'E':
	{
		await_distance = (uint8_t)value ? (uint8_t)value : 1;
		break;
	}
	case '?':
	{
		telemetryResponse();
		break;
	}
	//speed PID rates
	case 'p':
	{
		kP = value;
		break;
	}
	case 'i':
	{
		kI = value;
		break;
	}
	case 'd':
	{
		kD = value;
		break;
	}
	case 'L':
	{
		//three bits - three leds
		HAL_GPIO_WritePin(LED_1_PORT, LED_1_PIN, ((uint8_t)value & 0b001));
		HAL_GPIO_WritePin(LED_2_PORT, LED_2_PIN, ((uint8_t)value & 0b010));
		HAL_GPIO_WritePin(LED_3_PORT, LED_3_PIN, !((uint8_t)value & 0b100));
		break;
	}
	default:
		break;
	}
}

//Parses message by message, splits on newline character
void parseBuffer(uint8_t length)
{
	uint8_t remaining_length = length;
	uint8_t* p = UART1_rxBuffer;
	while(remaining_length)
	{
		int i = 0;
		char message[100];
		memset(message,0,sizeof message);
		while( i < remaining_length && *p !='\n' )
		{
			message[i] = (char)*p;
			i++;
			p++;
		}
		remaining_length -= (i+1);
		message[i] = '\0';
		if(i>0)interpretMessage(message);
//        HAL_UART_Transmit(&huart2,(uint8_t*)message,i+1,0x200);
		p++;
	}
}

//uart idle state callback function
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	//Stop this DMA transmission
    HAL_UART_DMAStop(huart);

    //Calculate the length of the received data
    uint8_t data_length  = UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

	//Test function: Print out the received data
//    HAL_UART_Transmit(&huart2,UART1_rxBuffer,data_length,0x200);
    parseBuffer(data_length);

	//Zero Receiving Buffer
    memset(UART1_rxBuffer,0,data_length);
    data_length = 0;

    //Restart to start DMA transmission of 255 bytes of data at a time
    HAL_UART_Receive_DMA(huart, (uint8_t*)UART1_rxBuffer, UART_BUFFER_SIZE);
}

void loop100Hz()
{
//	  sprintf(str,"%d\n",TIM5->CNT);
//	  sendString(str);
	if(posRegulationEnabled == 1)
	{
		positionRegulator();
	}

	speedRegulator(10000);

	 //sprintf(str,"target:%f\tcurrent: %f\terror:%f\n",target_speed,curSpeed,target_speed-curSpeed);
	 // sendString(str);
	//oneTurn();
	loop_cnt++;
	if(loop_cnt>=100)
	{
		loop1Hz();
		loop_cnt=0;
	}
}

void loop1Hz()
{
	getBatVoltage();
}

void getBatVoltage()
{
	uint16_t adc_res;
	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	{
		adc_res = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start(&hadc1);
		//3.79 is a voltage divider multiplication
		bat_voltage = (float)adc_res * (3.3f)/4095.0f * voltage_coef;
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	//1Hz loop
	if(htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,TIM5->CNT+10000-1);
		loop100Hz();
	}
}


int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for(DataIdx=0; DataIdx<len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}


int32_t MAP(int32_t au32_IN, int32_t au32_INmin, int32_t au32_INmax, int32_t au32_OUTmin, int32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
