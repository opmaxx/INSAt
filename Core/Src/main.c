/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm9ds1_driver.h"
#include "rf_driver.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef struct {
  void   *hbus;
  uint8_t i2c_address;
} sensbus_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BOOT_TIME  20
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart_rx;

SPI_HandleTypeDef hspi1;

/* Definitions for consoleTask */
osThreadId_t consoleTaskHandle;
uint32_t consoleTaskBuffer[ 128 ];
osStaticThreadDef_t consoleTaskControlBlock;
const osThreadAttr_t consoleTask_attributes = {
  .name = "consoleTask",
  .cb_mem = &consoleTaskControlBlock,
  .cb_size = sizeof(consoleTaskControlBlock),
  .stack_mem = &consoleTaskBuffer[0],
  .stack_size = sizeof(consoleTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
uint32_t imuTaskBuffer[ 128 ];
osStaticThreadDef_t imuTaskControlBlock;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .cb_mem = &imuTaskControlBlock,
  .cb_size = sizeof(imuTaskControlBlock),
  .stack_mem = &imuTaskBuffer[0],
  .stack_size = sizeof(imuTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rfTask */
osThreadId_t rfTaskHandle;
uint32_t rfTaskBuffer[ 128 ];
osStaticThreadDef_t rfTaskControlBlock;
const osThreadAttr_t rfTask_attributes = {
  .name = "rfTask",
  .cb_mem = &rfTaskControlBlock,
  .cb_size = sizeof(rfTaskControlBlock),
  .stack_mem = &rfTaskBuffer[0],
  .stack_size = sizeof(rfTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for consoleQueue */
osMessageQueueId_t consoleQueueHandle;
const osMessageQueueAttr_t consoleQueue_attributes = {
  .name = "consoleQueue"
};
/* Definitions for consoleMutex */
osMutexId_t consoleMutexHandle;
const osMutexAttr_t consoleMutex_attributes = {
  .name = "consoleMutex"
};
/* USER CODE BEGIN PV */
static sensbus_t mag_bus = {&SENSOR_BUS,
							LSM9DS1_MAG_I2C_ADD_H
                           };
static sensbus_t imu_bus = {&SENSOR_BUS,
							LSM9DS1_IMU_I2C_ADD_H
                           };
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
//static uint8_t rst;
static uint8_t tx_buffer[1000];
static lsm9ds1_status_t reg;

static lsm9ds1_id_t whoamI;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartConsoleTask(void *argument);
void StartIMUTask(void *argument);
void StartRFTask(void *argument);

/* USER CODE BEGIN PFP */
void sendUART(char *data);
static int32_t stm32l4_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t stm32l4_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t stm32l4_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t stm32l4_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);

void transmit_demo(uint16_t size);
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of consoleMutex */
  consoleMutexHandle = osMutexNew(&consoleMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of consoleQueue */
  consoleQueueHandle = osMessageQueueNew (10, sizeof(char), &consoleQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of consoleTask */
  consoleTaskHandle = osThreadNew(StartConsoleTask, NULL, &consoleTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTask_attributes);

  /* creation of rfTask */
  rfTaskHandle = osThreadNew(StartRFTask, NULL, &rfTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00505B89;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GDO0_Pin */
  GPIO_InitStruct.Pin = GDO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GDO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartConsoleTask */
/**
  * @brief  Function implementing the consoleTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartConsoleTask */
void StartConsoleTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	osStatus_t status;
	char data[128]; // Taille maximale des données à envoyer

	consoleQueueHandle = osMessageQueueNew(10, sizeof(data), NULL);
	consoleMutexHandle = osMutexNew(NULL);
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		status = osMessageQueueGet(consoleQueueHandle, &data, NULL, 0); // vérification si données dans fifo
		if (status == osOK)
		{
		  status = osMutexAcquire(consoleMutexHandle, osWaitForever);
		  if (status == osOK)
		  {
			  HAL_UART_Transmit(&hlpuart1, (uint8_t*)&data, strlen(data), 0xFFFF); // envoyer données à UART
			  osMutexRelease(consoleMutexHandle);
		  }
		}
		osDelay(1000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void* argument)
{
	/* USER CODE BEGIN 6 */
  stmdev_ctx_t dev_ctx_imu;
  stmdev_ctx_t dev_ctx_mag;

  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = stm32l4_write_imu;
  dev_ctx_imu.read_reg = stm32l4_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = stm32l4_write_mag;
  dev_ctx_mag.read_reg = stm32l4_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;

  osDelay(BOOT_TIME);
  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID)
  {
	    sendUART("whoami not passed\r\n");
  }
  else
  {
    	sendUART("whoami passed\r\n");
    	osDelay(100);
  }
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                PROPERTY_ENABLE);
  /* Set full scale */
   lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
   lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
   lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
   lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
   lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu,
                                      LSM9DS1_LP_ODR_DIV_50);
   lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
   /* Gyroscope filtering chain */
   lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu,
                                      LSM9DS1_LP_ULTRA_LIGHT);
   lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
   lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu,
                                  LSM9DS1_LPF1_HPF_LPF2_OUT);
   /* Set Output Data Rate / Power mode */
   lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
   lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);
  /* Infinite loop */
  for(;;)
  {
    /* Read device status register */
	lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
	if ( reg.status_imu.xlda && reg.status_imu.gda )
	{
	  /* Read imu data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	  lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
	                               data_raw_acceleration);
	  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
	                               data_raw_angular_rate);
	  acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[0]);
	  acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[1]);
	  acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[2]);
	  angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[0]);
	  angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[1]);
	  angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[2]);
	  //sprintf((char *)tx_buffer,
	  //        "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
	  //        acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
	  //        angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
	  //sprintf((char *)tx_buffer, "blabla\r\n");
	  osDelay(100);
	}

	if ( reg.status_mag.zyxda )
	{
	  /* Read magnetometer data */
	  memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
	  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
	  magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[0]);
	  magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[1]);
	  magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[2]);
	  //sprintf((char *)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
	  //        magnetic_field_mgauss[0], magnetic_field_mgauss[1],
	  //        magnetic_field_mgauss[2]);
	  osDelay(100);
	}
	sendUART("Mesures faites\r\n");
    osDelay(2000);
  }
  /* USER CODE END 6 */
}

static int32_t stm32l4_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                 I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t stm32l4_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                 I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

static int32_t stm32l4_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  reg |= 0x80;

  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                  I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t stm32l4_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  reg |= 0x80;

  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                 I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

void sendUART(char *data)
{
	osMessageQueuePut(consoleQueueHandle, (char *)data, 0, 0);
}

/* USER CODE BEGIN Header_StartRFTask */
/**
* @brief Function implementing the rfTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRFTask */
void StartRFTask(void *argument)
{
  /* USER CODE BEGIN StartRFTask */
	rf_begin(&hspi1, GFSK_1_2_kb, MHz434, CS_GPIO_Port, CS_Pin, GDO0_Pin);
	rf_sidle();
	uint8_t result = rf_read_register(MARCSTATE);
	rf_set_carrier_frequency(433.98);
	rf_set_carrier_offset(50);
	uint16_t i = 2;
	while(i<33){
			  transmit_demo(i);
			  osDelay(1000);
			  i++;
	}
  /* Infinite loop */
  for(;;)
  {
		transmit_demo(i);
		i +=100;
		osDelay(1000);
  }
  /* USER CODE END StartRFTask */
}

void transmit_demo(uint16_t size){
	uint8_t data[size];
	int i;
	static int counter = 0;
	data[0] = size & 0xFF;
	data[1] = (size>>8) & 0xFF;

	for(i=2; i<size; i++){
	  data[i] = (i)%256;
	}

	if(send_frame(data, size)!=FRAME_OK) sendUART("ERROR\n\r");
	else sendUART("Niquel!\r\n");
	data[size-1] = counter++;

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
