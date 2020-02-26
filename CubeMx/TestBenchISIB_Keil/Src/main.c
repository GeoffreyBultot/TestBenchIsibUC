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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "types.h"
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

/* USER CODE BEGIN PV */
uint16_t ADC_buffer[6], adc[6], nmoyenne;
uint32_t moyenne[6];
T_SPI_FRAME MessageToSend;
union Measures_telemetries Measures_TM;

uint8_t buffer_SPI_TX[16] = {0x35,2,0x80,0x24,0x1,0x2,0x1,0x2,0x1,0x2,0x1,0x2,0x1,0x2,0x1,0x2};
uint8_t buffer_SPI_RX[16];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_PWM 65535	//Vitesse max robot
float C = 134.0;//136		//Consigne
#define H 75		//Fréquence ech en mS
float K = 1000;		//Gain proportionnel


void P_Loop_Motor()
{
	float M = 0.0;
	float E = 0.0;	//Erreur
	float u = 0.0;	
	
	M = Measures_TM.strct.Umotor;//Mesure
	
	E=C-M;			//Calcul de l'erreur
	u = E*K;	//Calcul de la commande
	
	if(u>MAX_PWM)		//commande est trop grande 
	{//PWM max
		htim3.Instance->CCR1 = MAX_PWM;
	}
	else if(u<0)
	{
		htim3.Instance->CCR1 = 0;
	}
	else
	{//Si on calcule une plage de vitesse acceptable, on donne
	 //Cette vitesse au robot.
		htim3.Instance->CCR1 = u;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	int i;
	for (i =0; i<6; i++)
	{
		 moyenne[i] += ADC_buffer[i];  // store the values in adc[]
	}
	++nmoyenne;
	
	if(nmoyenne==10)
	{
		for (i =0; i<6; i++)
		{
			adc[i] = moyenne[i]/10;  // store the values in adc[]
			moyenne[i] = 0;
		}
		nmoyenne = 0;
		Measures_TM.strct.ULoadCell = adc[0] - adc[1];
		Measures_TM.strct.Umotor = adc[2];
		Measures_TM.strct.Imotor = adc[3];
		Measures_TM.strct.Ubrake = adc[4];
		Measures_TM.strct.Ibrake = adc[5];
		Measures_TM.strct.Speed = adc[5];
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	buffer_SPI_TX[0] = MessageToSend.fr.SOF;
	buffer_SPI_TX[1] = MessageToSend.fr.Length;
	buffer_SPI_TX[2] = MessageToSend.fr.OD_INDEX;
	for(int i = 0;i<15;i++)
	{
		buffer_SPI_TX[i+3] = ((uint8_t*)(Measures_TM.measures))[i];
	}
	buffer_SPI_TX[15] = MessageToSend.fr.EOF;
	HAL_SPI_Transmit_IT(&hspi2, buffer_SPI_TX, 16);
	//HAL_SPI_Receive(&hspi2,receiveData,16,100);

	//HAL_SPI_Transmit(&hspi2,array,2,100);	
	HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 16);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	buffer_SPI_TX[0] = MessageToSend.fr.SOF;
	buffer_SPI_TX[1] = MessageToSend.fr.Length;
	buffer_SPI_TX[2] = MessageToSend.fr.OD_INDEX;
	for(int i = 0;i<15;i++)
	{
		buffer_SPI_TX[i+3] = ((uint8_t*)(Measures_TM.measures))[i];
	}
	buffer_SPI_TX[15] = MessageToSend.fr.EOF;
	HAL_SPI_Transmit_IT(&hspi2, buffer_SPI_TX, 16);
}

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
  MX_I2C1_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_USART1_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	int N;
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //Start the PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //Start the PWM
	htim4.Instance->CCR3 = 0; //50% duty cycle
	htim4.Instance->CCR4 = 0; //25% duty cycle
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Start the PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Start the PWM
	htim3.Instance->CCR1 = 0; //50% duty cycle
	htim3.Instance->CCR2 = 0; //25% duty cycle
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_buffer, 6);
	HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 16);
	//HAL_SPI_TransmitReceive_IT(&hspi2,buffer_SPI_TX,buffer_SPI_RX,16);
	//HAL_SPI_TransmitReceive_DMA(&hspi2,buffer_SPI_TX,buffer_SPI_RX,16);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float ref, ref3, value, realvaluemes;
	ref3 = (float)(adc[2])*3.3/4095;
	ref = (float)(adc[1])*3.3/4095;
	value = (float)(adc[0])*3.3/4095;
	realvaluemes = (float)(value-ref)/0.056;
	N=3;
	
	MessageToSend.fr.SOF = 0XC5;
	MessageToSend.fr.Length = 16;
	MessageToSend.fr.OD_INDEX = 0X80;
	MessageToSend.fr.EOF		=  0XFF;
	MessageToSend.fr.data 	= (uint8_t*)(Measures_TM.measures);
	
	while (1)
  {
		//uint8_t _transmitter = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
		/*if (!_transmitter)
		{
				buffer_SPI_TX[0] = MessageToSend.fr.SOF;
				buffer_SPI_TX[1] = MessageToSend.fr.Length;
				buffer_SPI_TX[2] = MessageToSend.fr.OD_INDEX;
				for(int i = 0;i<15;i++)
				{
					buffer_SPI_TX[i+3] = ((uint8_t*)(Measures_TM.measures))[i];
				}
				buffer_SPI_TX[15] = MessageToSend.fr.EOF;
				
				HAL_SPI_Transmit_IT(&hspi2, buffer_SPI_TX , 16);
		}
		else
		{
			 HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 16);
		}*/
		P_Loop_Motor();
		
		
		//htim3.Instance->CCR1 =10000;
		
		/*for(int i = 0; i<0xFFFF;i+=10)
		{
			htim3.Instance->CCR1 =i;
			HAL_Delay(1);
			//HAL_SPI_Receive_IT(&hspi2,receiveData,1);
		}*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
