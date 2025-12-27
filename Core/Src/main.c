/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "debug.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ServoH 50 // 阈值
#define ServoL 2  

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define ADC_CHANNELS 6
uint32_t dmaBuffer[ADC_CHANNELS];
  
uint32_t lastServoMove;
uint8_t tickFlag=0,level;
int EmgEnvelop,lastEmgEnvelop, filterEmgEnvelop;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

float Filter(float input);
int getEnvelop(int abs_emg);
int emgLevel(int emgval) ;  
void SetServoAngle(int16_t angle);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	printf("HelloCheez\r\n");
	HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, dmaBuffer, ADC_CHANNELS);
  HAL_TIM_Base_Start_IT(&htim3); 
	HAL_TIM_Base_Start_IT(&htim4);
 
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
		 if(tickFlag)
		 {
			tickFlag=0;
			 
			uint16_t A0 = dmaBuffer[0];
			float EMGSignal =	Filter((float)A0);
			EmgEnvelop = getEnvelop(fabs(EMGSignal));
	  
			char dat[60];
			uint32_t	len = sprintf(dat, "%d,%d,%d\r\n",A0,(int16_t)EMGSignal,EmgEnvelop);
			CDC_Transmit_FS((uint8_t*)dat,len);												// 使用USB虚拟串口
			HAL_UART_Transmit(&huart1, (uint8_t*)dat,len, 1000);			// 使用CH340串口

		 }
		 
		  if (HAL_GetTick() - lastServoMove > 20) 
			{
				lastServoMove = HAL_GetTick();
				if(HAL_GPIO_ReadPin(EMG1_LEAD_GPIO_Port,EMG1_LEAD_Pin))
				{
					filterEmgEnvelop = 0.8 * lastEmgEnvelop + 0.2 * EmgEnvelop;
					lastEmgEnvelop = filterEmgEnvelop;
					SetServoAngle(filterEmgEnvelop);
				}
			}
			 
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)tickFlag =1; 
}

int getEnvelop(int abs_emg) {
	// 窗口
	#define BUFFER_SIZE 128           			 // 窗口大小
	static int circular_buffer[BUFFER_SIZE]; // 环形数组
	static int data_index, sum;              // 数据索引
	
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum / BUFFER_SIZE) * 2;
}
 

void SetServoAngle(int16_t angle) {  
    if (angle<0||angle > 180) return;   
 
    uint32_t compareValue = 50 + (angle * 200 / 180);   
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,compareValue); 
}


/**************************** 滤波 由软件生成 ************************************/
// >>> Butterworth IIR Digital Filter: bandpass
// 	Sampling Rate:500.0 Hz ,Frequency:[70.0, 110.0] Hz
// 	Order: 4.0 ,implemented as second-order sections (biquads)
float Filter(float input) {
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - (-0.55195385 * z1) - (0.60461714 * z2);
    output = 0.00223489 * x + (0.00446978 * z1) + (0.00223489 * z2);
    z2 = z1;
    z1 = x;
  }

  {
    static float z1, z2; // filter section state
    float x = output - (-0.86036562 * z1) - (0.63511954 * z2);
    output = 1.00000000 * x + (2.00000000 * z1) + (1.00000000 * z2);
    z2 = z1;
    z1 = x;
  }

  {
    static float z1, z2; // filter section state
    float x = output - (-0.37367240 * z1) - (0.81248708 * z2);
    output = 1.00000000 * x + (-2.00000000 * z1) + (1.00000000 * z2);
    z2 = z1;
    z1 = x;
  }

  {
    static float z1, z2; // filter section state
    float x = output - (-1.15601175 * z1) - (0.84761589 * z2);
    output = 1.00000000 * x + (-2.00000000 * z1) + (1.00000000 * z2);
    z2 = z1;
    z1 = x;
  }

  return output;
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
