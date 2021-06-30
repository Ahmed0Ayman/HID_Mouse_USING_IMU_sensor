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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include <stdio.h>
#include "retarget.h"
#include <math.h>
#include "usbd_customhid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_Flag_BIT			0x01u
#define JOY_Flag_BIT			0x02u

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int8_t 	 USBbuffer[4] ; /* the HID endpoint descriptor only define as 4 byte boot mode  */
int32_t JOYRes ;

/* volatile force the compiler to read every time because this variable is modified through ISR */
volatile union {
	struct {
	uint8_t LeftButton     : 1 ;
	uint8_t RightButton    : 1 ;
	uint8_t MiddleButton   : 1 ;
	uint8_t VendorSpecific : 5 ; /* left for vendor specification */
	}Buttons;
	uint8_t AllBits  ;
}MouseButtons;


uint8_t arr_cong[8] = {MPU6050_SMPLRT_DIV_R,0x07, MPU6050_PWR_MGMT_1_R,0x01, MPU6050_GYRO_CONFIG_R,0,MPU6050_ACCEL_CONFIG_R,0x18};

int8_t  X_Axis , Y_Axis ,MouseWheel;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

osSemaphoreId_t	IMUSemaHandle;
const osSemaphoreAttr_t IMUSema_attributes = {
		/* set name for debugging information */
  .name = "IMUSemaphore"
};

osSemaphoreId_t	JOYSemaHandle;
const osSemaphoreAttr_t JOYSema_attributes = {
		/* set name for debugging information */
  .name = "JOYSemaphore"
};


osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTasK_Attr ;


osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTasK_Attr ;


osThreadId_t JOYTaskHandle;
const osThreadAttr_t JOYTasK_Attr ;


osEventFlagsId_t Main_TaskEvents ;
const osEventFlagsAttr_t Main_TaskEventAttr ={.name =" Main_TaskEventAttr "} ;



void MainTask_Fun(void * arg);
void IMUTask_Fun(void * arg);
void JOYTask_Fun(void * arg);



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 osThreadAttr_t * TasK_Attr ;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

   TasK_Attr =  &MainTasK_Attr; /* two way to initialize constant var but i used the pointer method */


    TasK_Attr->name = "MainTask";
    TasK_Attr->stack_size = 256 * 4;
  	TasK_Attr->priority = (osPriority_t) osPriorityHigh; /* only the other task run when enter block state */


  	TasK_Attr =  &IMUTasK_Attr;

  	TasK_Attr->name = "IMUTask";
  	TasK_Attr->stack_size = 256 * 4;
	TasK_Attr->priority = (osPriority_t) osPriorityAboveNormal1;



  	TasK_Attr =  &JOYTasK_Attr ;

  	TasK_Attr->name = "JOYTask";
  	TasK_Attr->stack_size = 256 * 4;
	TasK_Attr->priority = (osPriority_t) osPriorityAboveNormal;

	MainTaskHandle = osThreadNew(MainTask_Fun, NULL, &MainTasK_Attr);

	IMUTaskHandle = osThreadNew(IMUTask_Fun, NULL, &IMUTasK_Attr);

	JOYTaskHandle = osThreadNew(JOYTask_Fun, NULL, &JOYTasK_Attr);

	IMUSemaHandle = osSemaphoreNew(1, 0, &IMUSema_attributes);

	JOYSemaHandle = osSemaphoreNew(1, 0, &JOYSema_attributes);

	Main_TaskEvents = osEventFlagsNew(&Main_TaskEventAttr);


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* start timer 8 to work as trigger for ADC conversion */
  HAL_TIM_Base_Start(&htim8);

  /* start ADC and initialize it to work with DMA mode */
  HAL_ADC_Start_DMA(&hadc1,&JOYRes, 1);

  /* register timer handler to work with stdio system calls */
  RetargetInit(&huart1 );

  /* set configurations for MPU6050 */
  MPU6050_SET_Config(arr_cong ,sizeof(arr_cong));
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	  if (GPIO_Pin == GPIO_PIN_1) /* if GPIOA pin1 left button */
	  {
		  MouseButtons.Buttons.LeftButton = 1;
	  }
	  if(GPIO_Pin == GPIO_PIN_0)/* if GPIOA pin0 right button */
	  {
		  MouseButtons.Buttons.RightButton = 1;
	  }


}


void MainTask_Fun(void * arg)
{
	const TickType_t xDelayTicks = 1 / portTICK_PERIOD_MS; /* calculate the number of ticks that generate 1Ms delay*/

	while(1)
	{
		USBbuffer[0] = MouseButtons.AllBits ;
		USBbuffer[1] = X_Axis;
		USBbuffer[2] = Y_Axis;
		USBbuffer[3] = MouseWheel;

		  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, USBbuffer, 4);


		  MouseButtons.AllBits = 0 ;
		  X_Axis = 0 ;
		  Y_Axis= 0 ;
		  MouseWheel= 0 ;

			osDelay(xDelayTicks);  /* make some delay */

			osSemaphoreRelease(JOYSemaHandle);
			osSemaphoreRelease(IMUSemaHandle);


			osEventFlagsWait(Main_TaskEvents, IMU_Flag_BIT | JOY_Flag_BIT , osFlagsWaitAll, osWaitForever);

	}

}




/* this task used to update X and Y axis */
void IMUTask_Fun(void * arg)
{
	MPU6050_ANGELS_t CurrentMeasure ={0} ;


	float TempX = 0,TempY = 0 ;	/* perform the operation on local var before access the global one */
	while(1)
	{

		  osSemaphoreAcquire(IMUSemaHandle , osWaitForever);

		  /* read IMU angles */
		Read_Accurate_Angles_ComplemantrayFilter(&CurrentMeasure);

		/*IMU angles from =-90 t0 90 but axis -128 to 127 but we need it to be more stable */
		TempX = (CurrentMeasure.GYRO_ANGLE_ROLL * 0.1)  ;
		TempY = (CurrentMeasure.GYRO_ANGLE_PITCH * 0.1) ;

			/* no need to protect these global variables because all tasks is synchronized so there is no concurrency access right now*/
			X_Axis  = TempX ;

			Y_Axis  = TempY ;




			/* print the measured angle through uart */
		printf("pitch = %0.2f  ,  Roll = %0.2f  , Yaw = %0.2f  \n",CurrentMeasure.GYRO_ANGLE_PITCH
				  	  	  	  	  	  	  	  	  	  	  ,CurrentMeasure.GYRO_ANGLE_ROLL
														  ,CurrentMeasure.GYRO_ANGLE_YAW);

		/* send notification to the main function my data is done */
		osEventFlagsSet(Main_TaskEvents, IMU_Flag_BIT);
	}

}


/* ADC work with DMA so data is update periodically */
void JOYTask_Fun(void * arg)
{
	while(1)
	{
		osSemaphoreAcquire(JOYSemaHandle , osWaitForever);
		MouseWheel = (JOYRes - 2048)/1000;
		osEventFlagsSet(Main_TaskEvents, JOY_Flag_BIT);


	}

}


/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
