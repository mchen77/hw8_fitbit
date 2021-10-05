/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "retarget.h"
#include "hw8_heartRate.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

osThreadId_t defaultTaskHandle;
/* USER CODE BEGIN PV */
#define MPU6500_ADDR 0x68<<1				// ACCELEROMETER //
#define MPU6500_RA_WHO_AM_I 0x75			// who am i???
int accel_cnt = 0;							// sample count
int ac_avg_cnt = 0;							// count for averaging accelerations
int32_t ax_avg, ay_avg, az_avg;				// averages for accel
int32_t avg;
int time_accel_raw;							// acceleration time
int time_accel;								// acceleration time
int32_t ax, ay, az, xangle, yangle;			// acceleration X,Y,Z and X angle and Y angle
uint8_t bud[6];								// buffer to read raw accel data
mpu_sample_t sample_accel;					// accel struct to r/w to
mpu_sample_t accel_o;						// accel struct to r/w to

osThreadId_t TID1, TID2, TID3;				// THREADS //
osMutexId_t I2C1_MUTEX_ID;					// mutex
osMessageQueueId_t QUEUE_ID, A_QUEUE_ID;	// queues
char printbuf[1000];						// print buffer
#define MSGQUEUE_OBJECTS 16					// something defined in example so why not

#define MAX30102_ADDR 0xae					// HEARTRATE MONITOR //
#define MAX30102_PARTID 0xFF				// HR ID
#define Mode_Config 0x09					// was going to #define each ADDR for MAX30102 but got lazy
uint8_t buf[6];								// buffer for HR monitor set up and read
uint8_t bul[6];								// buffer to read hr from
uint32_t heart_sample, another_hr;			// heartrate samples
uint32_t heartrate[4], hr[10];				// arrays to average HR
int count1 = 0, count2 = 0, count3 = 0;		// counters for hr sample processing
int hr1, hr2, hr_read1 = 0, hr_read2 = 0;	// current & old count of HR samples
const int num_beats = 10;					// for smoothing/averaging 10 beats
int hr_cnt = 0;								// current count of hr samples read (debugging)
int hr_10cnt = 0;							// counter to not start displaying bpm
int hr_time_raw = 0, hr_time = 0;			// time it takes to read num of hr_cnt (debugging)
int hr_total = 0;							// smoothing var
int hr_FINAL = 0;							// final HR to print
int hr_current = 0;							// smoothing var
int old_hr_reading = 0;						// 1 if old reading, 0 otherwise
typedef struct{								// struct for HR QUEUE
	uint32_t hr_buf;						// heart rate buffer
	uint8_t Idx;							// id bc it was in the example so why not
} MSGQUEUE_OBJ_t;							// name of struct
MSGQUEUE_OBJ_t msg;							// declare a struct for hr
int total_time = 0;							// total time passed of program
uint8_t last_beat = 1;						// instance var for measuring time since last beat
int last_beat_time = 0;					// variable to store time count of first beat

uint8_t var1;								// DOUBLE TAP //
uint32_t prev_z = 0;						// Store previous z reading for comparison
uint32_t thresh = 3000;  					// CHANGE depending on force for tap
uint32_t lat = 40;							// CHANGE depending on how fast Z is read
uint8_t start_up_tap = 0;					// tap count for initial start up
uint8_t start_up = 0;						// instance for triple tap condition
int tap = 0;								// current tap count
uint8_t mode = 1;							// current mode count
uint8_t mode2_delay1 = 0, mode2_delay2 = 0;	// checkForBeat returns twice per beat (for most part), reduce to once per beat
uint8_t display_mode = 0;					// a random counter
uint8_t idle_mode = 0, init_hr = 1;			// 1 if idle mode is entered. 1 if need to initialize hr monitor

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void ThreadFunc1(void *argument);
void ThreadFunc2(void *argument);
void ThreadFunc3(void *argument);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c);
void HAL_I2C_ErrorCallBack(I2C_HandleTypeDef *hi2c);
void UART2_SendString(char* printbuf);
void initialize_HR();

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);	//double tap timer
  HAL_TIM_Base_Start(&htim7);	//heart rate timer
  HAL_TIM_Base_Start(&htim6);	//interrupt timer
  HAL_TIM_Base_Start(&htim14);	//acceleration timer
  HAL_TIM_Base_Start(&htim15);	//longer timer to test
  HAL_TIM_Base_Start(&htim16);	//acceleration samples per second timer

  HAL_I2C_Init(&hi2c1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_MspInit(&htim6);

  RetargetInit(&huart2);

  // Print some nice info //
  printf("\r\n\r\nhello world!\r\n");					// the usual
  HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, MPU6500_RA_WHO_AM_I,I2C_MEMADD_SIZE_8BIT, bud, 1, 100);
  printf("Accelerometer ID = 0x%x \r\n", bud[0]);		// accelerometer ID = 0x70

  //heart rate monitor
  HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, MAX30102_PARTID, I2C_MEMADD_SIZE_8BIT, bul, 1, 100);
  printf("Heartrate monitor ID = 0x%x \r\n", bul[0]);   // HR ID = 0x15

  printf("\r\nRest finger on HR monitor for 10 heart beats to calibrate properly. . .\r\n"
		  "Keep finger as still as possible for best reading. Sometimes thumb gets better reading . . .\r\n"
		  "In HEARTBEAT MODE, press finger hard on sensor for best beat detections (otherwise may double read),\r\n"
		  "		though pressing too hard may give inaccurate BPMs. . .\r\n");
  printf("Have the serial monitor wide enough so it is wide enough to read this entireeeeeeeeeeeeeeeee line of text.\r\n\r\n");

  printf("Double tap to toggle between THREE MODES:\r\n\t-[1] SUMMARY MODE\r\n\t"
		  "-[2] HEARTBEAT MODE\r\n\t-[3] IDLE MODE\r\n\r\n");

  printf("--------------- TRIPLE TAP TO BEGIN ---------------\r\n\r\n");

  /* USER CODE END 2 */

  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  I2C1_MUTEX_ID = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  //osMessageQueueId_t osMessageQueueNew (uint32_t msg_count, uint32_t msg_size, const osMessageQueueAttr_t *attr);
  QUEUE_ID =  osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(MSGQUEUE_OBJ_t), NULL);	//HR queue
  A_QUEUE_ID =  osMessageQueueNew(16, 8, NULL);										//accel queue
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 4
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

//  const osThreadAttr_t lowdefaultTask_attributes = {
//      .name = "lowdefaultTask",
//      .priority = (osPriority_t) osPriorityBelowNormal,
//      .stack_size = 128 * 4
//    };

  TID1 = osThreadNew(ThreadFunc1, NULL, NULL);		// HR
  TID2 = osThreadNew(ThreadFunc2, NULL, NULL);		// accel
  TID3 = osThreadNew(ThreadFunc3, NULL, NULL);		// printing

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000000;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47900;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 479;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 20000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 479;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 9999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 47900;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 9999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47900;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 20000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

}

/* USER CODE BEGIN 4 */

//Thread for reading heartrate monitor at 50 samples per second, 			uses htim7
void ThreadFunc1(void *argument){
	(&htim7)->Instance->CNT = 0;
	(&htim6)->Instance->CNT = 0;
	while(1){
		// DON'T aquire Mutex unless timing is correct
		if ((&htim7)->Instance->CNT > 900 && mode != 3 &&osMutexAcquire(I2C1_MUTEX_ID, 0) == osOK){	// og is 1000, played with timing
			if(init_hr == 1) {
				initialize_HR();
				init_hr = 0;
			}
			HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,  		//REG ADDR: 0x00
					(uint8_t *) buf, 1, 100);		//Poll Interrupt Status 1 register until PPG_RDY bit goes high
			while((buf[0] >> 6 & 0x1) != 1){
				HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,  	//REG ADDR: 0x00
					(uint8_t *) buf, 1, 100);		//Poll Interrupt Status 1 register until PPG_RDY bit goes high
			}
			HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, (uint8_t *) buf, 3, 100);
			msg.hr_buf = (buf[0] << 16) + (buf[1] << 8) + buf[2];

			osMessageQueuePut(QUEUE_ID, &msg, 0 , 0);		//returns osOK

			hr_cnt++;
			if(hr_cnt == 50){		//calibrate/check 50 samples/second
				hr_time_raw = (&htim6)->Instance->CNT;
				hr_cnt = 0;
				(&htim6)->Instance->CNT = 0;	//use to check time to run for samples/second
			}
			(&htim7)->Instance->CNT = 0;
			osMutexRelease(I2C1_MUTEX_ID);
			osThreadYield();
		}
	}
}

// Thread for Accelerometer at 200 samples per second, 					   	uses htim14 & htim16
void ThreadFunc2(void *argument){
//	UART2_SendString("Thread 2 entered\r\n");
	accel_cnt = 0;
	(&htim14)->Instance->CNT = 0;										// 200Hz timer
	(&htim16)->Instance->CNT = 0;										// accel samples per second timer
	while(1){				// DON'T aquire Mutex unless timing is correct
		if ((&htim14)->Instance->CNT > 140 && osMutexAcquire(I2C1_MUTEX_ID, 0) == osOK){ //og is 200, played with timing

			HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, MPU6500_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, bud, 6, 100);
			sample_accel.accelX = (bud[0] << 8) + bud[1]; // X axis
			sample_accel.accelY = (bud[2] << 8) + bud[3]; // Y axis
			sample_accel.accelZ = (bud[4] << 8) + bud[5]; // Z axis

			osMessageQueuePut(A_QUEUE_ID, &sample_accel, 1 , 0);		// returns osOK

			accel_cnt++;												// add counter for num samples read
			if(accel_cnt == 200){										// Once 200 samples are read
				time_accel_raw = (&htim16)->Instance->CNT;				// get time it takes to read 200 samples
				accel_cnt = 0;											// reset counter for samples read
				(&htim16)->Instance->CNT = 0;							// reset timer for accel samples/sec
			}

			(&htim14)->Instance->CNT = 0;
			osMutexRelease(I2C1_MUTEX_ID);
			osThreadYield();
		}
	}
}

// Thread to Print data													// uses htim15
void ThreadFunc3(void *argument){						// TA said don't need to explicitly make lower priority
	//printf("thread 3 reached\r\b");
	hr1 = 0, hr2 = 0;					// set heart rate counts to 0
	count1 = 0,	count2 = 0;				// counters for heart rate

	ax = 0,	ay = 0, az = 0;				// acceleration vars
	ax_avg = 0, ay_avg = 0, az_avg = 0;	// acceleration average vars
	xangle = 0, yangle = 0;				// acceleration angles
	time_accel = 0;
	tap = 0;

	(&htim15)->Instance->CNT = 0;
	(&htim2)->Instance->CNT = 0;
	while(1){
		if (osMutexAcquire(I2C1_MUTEX_ID, 0) == osOK){
//			UART2_SendString("Thread 3 aquired\r\n");

//			// Get from QUEUE //
			osMessageQueueGet(QUEUE_ID, &msg, NULL, 0);					// returns osOK for HR
			osMessageQueueGet(A_QUEUE_ID, &sample_accel, NULL, 0);		// returns osOK for Accel

//			// DOUBLE TAP BEGIN //
			// -single tap- //
			if(abs(sample_accel.accelZ - prev_z) > thresh && tap < 1){
				tap = 1;
				(&htim2)->Instance->CNT = 0;
			  }
			// -double tap- //
			if(tap >= lat){  								  // condition of lat is to wait for excess noise to pass
				if(abs(sample_accel.accelZ - prev_z) > thresh && (&htim2)->Instance->CNT < 250000){
					if(start_up_tap){
						mode++;
						if (mode >= 4) mode = 1;
						sprintf(printbuf,"Double tap! Switching to mode [%d]. . .\r\n", mode);
						HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
						display_mode = 1;
						tap = 0;
					}
					(&htim2)->Instance->CNT = 0;
					start_up = 1;
				} else if((&htim2)->Instance->CNT >250000){
					tap = 0;
					(&htim2)->Instance->CNT = 0;
				}
			}
			// -triple tap cuz why not- //
			if(!start_up_tap && start_up && tap >= 2 * lat){  // condition of lat is to wait for excess noise to pass
				if(abs(sample_accel.accelZ - prev_z) > thresh && (&htim2)->Instance->CNT < 250000){
					sprintf(printbuf,"---------- Triple tap detected! Starting MODE [1] ----------\r\n");
					display_mode = 1;
					HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
					start_up_tap = 1;
					tap = 0;
				} else if((&htim2)->Instance->CNT > 250000){
					tap = 0;
					(&htim2)->Instance->CNT = 0;
				}
			}
			if(tap >= 1) tap++;
			prev_z = sample_accel.accelZ;
//			// DOUBLE TAP END //


//			// MODE DISPLAY BEGIN //
			if(display_mode){										// only print MODE if double tap detected
				switch(mode){
				case 1:
					UART2_SendString("[1] SUMMARY MODE\r\n");		// print current mode
					UART2_SendString("(total    )|    X |     Y |     Z |(Xdeg,Ydeg)| Accel.,  HR | Heartrate"
						"\r\n(time     )|\t  |\t  |\t  |\t      | samples per | BPM\r\n(elapsed  )|");
					UART2_SendString("\t  |\t  |\t  |\t      | second      |\r\n\r");
					display_mode = 0;								// don't print again until next double tap
					init_hr = 1;									// need to initialize HR after SHDN in MODE [3]
					break;
				case 2:
					UART2_SendString("[2] HEARTBEAT MODE - Prints when a heartbeat is detected!\r\n");
					UART2_SendString("[t elasped]| Pulse detected? |Time since|  Heartrate (BPM)\r\n             last beat\r\n");
					display_mode = 0;
					break;
				case 3:
					UART2_SendString("[3] IDLE MODE - double tap to wake up\r\n");
					display_mode = 0;
					idle_mode = 1;
					break;
				default:
					UART2_SendString("no mode selected somehow\r\n");//should never run
					display_mode = 0;
				}
			}
//			// MODE DISPLAY END //


//  		// ACCELEROMETER BEGIN //
			if(mode == 1){
				time_accel = (200 * 1000) / time_accel_raw;			// accel samples per second
				hr_time    = (50 * 1000) / hr_time_raw;				// heartrate samples per second

				ax = sample_accel.accelX;							// add up samples to average (wasn't as clean so taken out)
				ay = sample_accel.accelY;
				az = sample_accel.accelZ;
				if(accel_cnt == 199){ 				// avg and third degree polynomial regression models,
//					ax_avg = ax/time_accel;			// don't calculate too often to prevent wasted resources
//					ay_avg = ay/time_accel;
//					az_avg = az/time_accel;
					xangle = 3.87+(ax*-2.65*pow(10,-3))+(1.34*pow(10,-8)*pow(ax,2))+(-1.03*pow(10,-11)*pow(ax,3));
					yangle = 3.52+(ay*-2.72*pow(10,-3))+(-8.56*pow(10,-9)*pow(ay,2))+(-9.83*pow(10,-12)*pow(ay,3));
				}
			}
// 			// ACCELEROMETER END //


//			// HEART RATE BEGIN //
			if(mode == 1 || mode == 2){			// only run HR processing if [1] HR MODE or [3] SUMMARY MODE
				heart_sample = msg.hr_buf;									// get sample from buffer
//				heartrate[count3] = heart_sample;							// average the last four heartrate readings (doesn't seem as accurate)
//				another_hr = (heartrate[0]+heartrate[1]+heartrate[2]+heartrate[3])/4; //average last 4 readings using infinite loop array
				count1++;													// update sample count
				if(checkForBeat(heart_sample) == 1 && heart_sample > 6000){	// checkForBeat, true if beat detected
					hr2 = hr1;												// save current sample count
					hr1 = count1;											// save previous sample count
					hr_10cnt++;
					if(hr_read1 < 1) hr_read1 = 1;							// pulse is identified
				}
			    if(hr2 != hr1) {									// calculate heartrate
					old_hr_reading = 0;
					hr_current = (50 * 60) * 2 / (hr1-hr2);	// samples/sec to bpm  =  (50 * 60) / (hr1-hr2);
					if(!(hr_current < 15)){							// get rid of bad apple readings
						hr_total = hr_total - hr[count2];			// smoothing algorithm using last ten samples (hr[10])
						hr[count2] = hr_current;
						hr_total = hr_total + hr[count2];
						count2++;
						if(count2 >= num_beats) count2 = 0;
						hr_FINAL = hr_total / num_beats;			// FINAL HR reading after smoothin'
						hr2 = hr1;
					}
				} else {
					old_hr_reading = 1;								// indicate it's an old reading
				}
			    last_beat_time = 60 * 1000 / hr_FINAL;				// time since last beat, mult by 60, x1000 for visibility
//				count3++;											// add to infinite loop
//				if(count3 >= 4) count3 = 0;							// reset infinite loop
			}
//			// HEART RATE END //

//			// IDLE MODE BEGIN //
			if (mode == 3 && idle_mode == 1){
				idle_mode = 0;
				var1 = 0x80;
				if(HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, Mode_Config, I2C_MEMADD_SIZE_8BIT,	//Bit 6 in "Mode Config," ADDR:0x09
					(uint8_t *) &var1, 1, 100) == HAL_OK){			//SHDN shut down pin set high
					var1 = 0x00;
					UART2_SendString("HR Monitor Shutting Down . . .\r\n");
				}
			}
//			// IDLE MODE END //

//			// PRINT BEGIN //
		if((&htim15)->Instance->CNT > 1000 && start_up_tap == 1){		// timer to print every second, don't start until triple tap
			if(mode == 1){				// 3 [1] SUMMARY MODE
				total_time++;																// add every second
				sprintf(printbuf, "(%2dm%2ds) | %6ld |%6ld |%6ld |", total_time/60,total_time%60, ax, ay, az);	//print time elapsed in min and sec
				HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);		// print accelerations in x y z
				sprintf(printbuf, " (%3ld,%3ld)||", xangle, yangle);	// print X and Y angles
				HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
				sprintf(printbuf, "%5d %5d || ", time_accel, hr_time);						// print acceleration samples per sec.
				HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
				if(heart_sample < 5000 && hr_FINAL < 10){									// HR printing, first read
					sprintf(printbuf, "Please place finger.\r\n");
				} else if(heart_sample < 5000){												// HR printing, finger removed
					sprintf(printbuf, "%3d is last reading. Please place finger.\r\n", hr_FINAL);
					hr_10cnt = 0;
				} else if(hr_10cnt < 10){													// HR calibrating until reads 10 beats
					sprintf(printbuf, "%3d calibrating . . .\r\n", hr_FINAL);
				} else {																	// HR printing, actively reading
					sprintf(printbuf, "%3d\r\n", hr_FINAL);
				}
				HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
				(&htim15)->Instance->CNT = 0;												// reset one sec timer
			} else {
				total_time++;
				(&htim15)->Instance->CNT = 0;
			}
		}

		if(mode == 2 && hr_read1 == 1){		// 1 [2] HEART RATE MODE, print any time when pulse detected (only detect first pulse)
			sprintf(printbuf,"(%2dm%2ds)   | Pulse detected! |", total_time/60,total_time%60);			// run time
			HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
			sprintf(printbuf," (%2d.%03d) | ", last_beat_time/1000, last_beat_time%1000);	// time since last beat, in sec.
			HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
			if(heart_sample < 5000 && hr_FINAL < 10){									// HR printing, first read
				sprintf(printbuf, "Please place finger.\r\n");
			} else if(heart_sample < 5000){												// HR printing, finger removed
				sprintf(printbuf, "%3d is last reading. Please place finger.\r\n", hr_FINAL);
				hr_10cnt = 0;
			} else if(hr_10cnt < 10){													// HR calibrating until reads 10 beats
				sprintf(printbuf, "%3d calibrating . . .\r\n", hr_FINAL);
			} else {																	// HR printing, actively reading
				sprintf(printbuf, "%3d\r\n", hr_FINAL);
			}
			HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
			hr_read1 = 2;
		}
		if(hr_read1 == 2) hr_read1 = 0;
//		if(hr_read1 == 1){
//			hr_read1 = 2;				// reset read pulse
//		} else if (hr_read1 == 2){
//			hr_read1 = 0;				// reset read pulse delay one beat
//		}
//			// PRINT END //

			osMutexRelease(I2C1_MUTEX_ID);
			osThreadYield();
		}
	}
}

// Function to print
void UART2_SendString(char* printbuf){
	HAL_UART_Transmit(&huart2, (uint8_t *)printbuf, strlen(printbuf), 100);
}

// When called, initializes heart rate monitor MAX30102
void initialize_HR(){
	if(init_hr == 1){		// initialize HR monitor
		init_hr = 0;
		var1 = 0x40;
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, Mode_Config, I2C_MEMADD_SIZE_8BIT,	//Bit 6 in "Mode Config," ADDR:0x09
							(uint8_t *) &var1, 1, 100);								//RESET, gives HAL_OK
		HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, Mode_Config, I2C_MEMADD_SIZE_8BIT,
				(uint8_t *) buf, 1, 100);
		while(buf[0] == 0x40){														//While reset pin reads 1, wait until it reads 0.
			HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, Mode_Config, I2C_MEMADD_SIZE_8BIT,
					(uint8_t *) buf, 1, 100);										//check current reset pin reading
		}
		var1 = 0b010;
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, Mode_Config, I2C_MEMADD_SIZE_8BIT,
					(uint8_t *) &var1, 1, 100);										//Write to Heart Rate Mode: MODE[2:0]=0b010
		var1 = 0x40;
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, 0x02, I2C_MEMADD_SIZE_8BIT,		//REG ADDR: 0x02
					(uint8_t *) &var1, 1, 100);										//Enable Interrupt Enable 1: PPG_READY interrupt (bit 6)
		HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, 0x02, I2C_MEMADD_SIZE_8BIT,
					(uint8_t *) buf, 1, 100);
		var1 = 0x63;
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, 0x0A, I2C_MEMADD_SIZE_8BIT,		//REG ADDR: 0x0A
					(uint8_t *) &var1, 1, 100);										//Write to SpO2 Config Register
		var1 = 0x1F;
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, 0x0C, I2C_MEMADD_SIZE_8BIT,		//REG ADDR: 0x1F
					(uint8_t *) &var1, 1, 100);										//Write to LED 1 Pulse Amplitude Register
		var1 = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, 0x04, I2C_MEMADD_SIZE_8BIT,		//REG ADDR: 0x04
					(uint8_t *) &var1, 1, 100);										//Write 0x00 to FIFO Write pointer
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, 0x05, I2C_MEMADD_SIZE_8BIT,		//REG ADDR: 0x05
					(uint8_t *) &var1, 1, 100);										//Write 0x00 to Overflow counter
		HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT,		//REG ADDR: 0x06
					(uint8_t *) &var1, 1, 100);										//Write 0x00 to FIFO Read pointer
//		UART2_SendString("HR Monitor Initialized\r\n");			// print to indicate HR monitor properly initialized
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
//	osThreadNew(ThreadFunc3, NULL, NULL);
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

//  count1++;
//  	printf("htim15: %d\r\n", (&htim15)->Instance->CNT);
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
