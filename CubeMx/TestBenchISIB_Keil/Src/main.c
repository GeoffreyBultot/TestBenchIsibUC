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
#include "Register.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/** @brief Enum Type to define the Motor Regulation Loop Mode*/
typedef enum
{
	E_MOT_REG_FACTOR_SPEED 		= 1,
	E_MOT_REG_FACTOR_VOLTAGE	= 2,
	E_MOT_REG_FACTOR_CURRENT 	= 3
}T_Motor_Regulation_Factor;

/** @brief Enum Type to define the Regulation Mode*/
typedef enum
{
	E_BRAKE_REG_FACTOR_TORQUE		= 1,
	E_BRAKE_REG_FACTOR_VOLTAGE	= 2,
	E_BRAKE_REG_FACTOR_CURRENT 	= 3
}T_Brake_Regulation_Factor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PWM 0xfff-10//65535	//Vitesse max robot

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_buffer[4], adc[4], nmoyenne;
uint32_t moyenne[4];

uint8_t buffer_SPI_TX[6];
uint8_t buffer_SPI_RX[6];


uint8_t buffer_UART_TX[8] = {0xC5,0X80,0X80,0X80,0X80,0X80,0X80,0X80};
uint8_t buffer_UART_RX[8];


uint16_t Table_Tm_Reg[C_TM_TABLE_SIZE];

/** @brief Saved Local CMD Count */
uint16_t Vi_Last_Cmd_Count;

/** @brief TELECOMMAND Table register */
__attribute__((section (".registers"))) volatile uint16_t Table_Tc_Reg[C_TC_TABLE_SIZE];


unsigned char regulation_Motor_ON = 0;
unsigned char regulation_Brake_ON = 0;
			

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Refresh_TM()
{
	//Table_Tm_Reg[C_TM_CR_MOT] 	= 	1;//couple_measure;//1000;//adc[C_ADCTAB_LOADCEL] - adc[C_ADCTAB_165REF];	
	Table_Tm_Reg[C_TM_U_MOT] 		= 	adc[C_ADCTAB_UMOTOR];
	Table_Tm_Reg[C_TM_I_MOT] 		= 	adc[C_ADCTAB_IMOTOR];
	Table_Tm_Reg[C_TM_U_BRAKE] 	= 	adc[C_ADCTAB_UBRAKE];
	Table_Tm_Reg[C_TM_I_BRAKE] 	= 	adc[C_ADCTAB_IBRAKE];
	//Table_Tm_Reg[C_TM_SP_MOT]		=		1;//counter_speed;
	Table_Tm_Reg[C_TM_PWM_MOT]		=		htim3.Instance->CCR2;//counter_speed;
	Table_Tm_Reg[C_TM_PWM_BRAKE]		=		htim3.Instance->CCR1;//counter_speed;
}

uint16_t *ptr_Measure_Motor = NULL;
uint16_t *ptr_Measure_Brake = NULL;
uint16_t C_Motor = 0.0;//136		//Consigne
uint16_t C_Brake = 0.0;
void Regulation_Loop_Motor()
{
	int E = 0.0;	//Erreur
	int M = 0.0;
	if(ptr_Measure_Motor == NULL)
	{
		C_Motor = 0;
		htim3.Instance->CCR2 = 0;
	}
	else
	{
		M = *(ptr_Measure_Motor);//Table_Tm_Reg[C_TM_U_MOT];//Mesure
		E = C_Motor-M;
		
		int pwm = htim3.Instance->CCR2;
		if(E>20)
		{
			if(pwm < (0xfff-10))
					pwm +=10;
		}
		else if(E<-20)
		{
			if(pwm > 10 )
					pwm -=10;
		}
		else
		{
			if(E<-2)
			{
				if(pwm > 0 )
					pwm --;
			}
			else if(E >0)
			{
				if(pwm < (0xfff-10))
					pwm ++;
			}
			else
			{
				//pwm = 0;//1424;//C/2;
			}
		}
		htim3.Instance->CCR2 = pwm;
	}
}

void Regulation_Loop_Brake(void)
{
	int E = 0;	//Erreur
	int M = 0;
	if(ptr_Measure_Brake == NULL)
	{
		C_Brake = 0;
		htim3.Instance->CCR1 = 0;
	}
	else
	{
		M = *(ptr_Measure_Brake);//Table_Tm_Reg[C_TM_U_BRAKE];//Mesure
		E = C_Brake-M;
		
		int pwm = htim3.Instance->CCR1;
		if(E>20)
		{
			if(pwm < (0xfff-10))
					pwm +=10;
		}
		else if(E<-20)
		{
			if(pwm > 10 )
					pwm -=10;
		}
		else
		{
			if(E<-2)
			{
				if(pwm > 0 )
					pwm --;
			}
			else if(E >0)
			{
				if(pwm < (0xfff-10))
					pwm ++;
			}
			else
			{
				//pwm = 0;//1424;//C/2;
			}
		}
		htim3.Instance->CCR1 = pwm;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		Regulation_Loop_Motor();
		Regulation_Loop_Brake();
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	int i;
	for (i =0; i<4; i++)
	{
		 moyenne[i] += ADC_buffer[i];  // store the values in adc[]
	}
	nmoyenne++;
	
	if(nmoyenne==100)
	{
		for (i =0; i<4; i++)
		{
			adc[i] = moyenne[i]/100;  // store the values in adc[]
			moyenne[i] = 0;
		}
		nmoyenne = 0;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	uint16_t value;
	uint8_t type = buffer_SPI_RX[0];
	if(hspi->Instance == SPI2)
	{
		if(type == 0x80)
		{
			value = buffer_SPI_RX[1];
			buffer_SPI_TX[1] = Table_Tm_Reg[value];
			buffer_SPI_TX[2] = Table_Tm_Reg[value]>>8;
		}
		else if(type == 0X20)
		{
			Table_Tc_Reg[C_TC_CMD_COUNT_ID] = (1+Vi_Last_Cmd_Count);
			Table_Tc_Reg[C_TC_CMD_ID] 		= (buffer_SPI_RX [1]);
			Table_Tc_Reg[C_TC_PARAM_1_ID] = (buffer_SPI_RX[2]<<8) + buffer_SPI_RX[3];
			Table_Tc_Reg[C_TC_PARAM_2_ID] = (buffer_SPI_RX[4]<<8) + buffer_SPI_RX[5];
			//TC DISPATCHER
		}
		else
		{
			//HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 1);
			//HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 1);
		}
		HAL_SPI_Transmit_IT(&hspi2, buffer_SPI_TX, 6);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI2)
		HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 6);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1,buffer_UART_RX,8);
	//HAL_UART_Transmit_IT(&huart1,buffer_UART_TX,8);
}

void get_ATMega_Infos()
{
	int couple_measure,counter_speed;
	HAL_UART_Transmit(&huart1,buffer_UART_TX,8,1);
	HAL_UART_Receive(&huart1,buffer_UART_RX,8,20);
	if( (buffer_UART_RX[0] == 0XC5) && (buffer_UART_RX[1] == 0X08) && (buffer_UART_RX[7] == 0XC5) )
	{
		
		couple_measure 	=	(buffer_UART_RX[2])<<8;
		couple_measure += (buffer_UART_RX[3]);
		counter_speed 	= (buffer_UART_RX[4])<<16;
		counter_speed  += (buffer_UART_RX[5])<<8;
		counter_speed  += (buffer_UART_RX[6]);
		//HAL_UART_Transmit_IT(&huart1,buffer_UART_TX,8);
		
		Table_Tm_Reg[C_TM_SP_MOT] = counter_speed;
		Table_Tm_Reg[C_TM_CR_MOT] = couple_measure;
	}
	
}

uint8_t Is_New_Command_Received(void)
{
	uint8_t Vb_Command_Received;
	Vb_Command_Received = 0;

	volatile uint16_t New_Cmd_Cnt;	/**< Temporary Variable used to store the CMD_COUNT register, read only once */

	/** Read the CMD_CNT register */
	New_Cmd_Cnt = Table_Tc_Reg[C_TC_CMD_COUNT_ID];
	
	if ((New_Cmd_Cnt - Vi_Last_Cmd_Count) == 1)
	{	/** Read the TC and take it into account */
		/** Return TRUE */
		Vb_Command_Received = 1;
	}
	/** Save the last Cmd count for next check */
	Vi_Last_Cmd_Count = New_Cmd_Cnt;

	return Vb_Command_Received;
}
/*
	Table_Tm_Reg[C_TM_U_MOT] 		= 	adc[C_ADCTAB_UMOTOR];
	Table_Tm_Reg[C_TM_I_MOT] 		= 	adc[C_ADCTAB_IMOTOR];
	Table_Tm_Reg[C_TM_U_BRAKE] 	= 	adc[C_ADCTAB_UBRAKE];
	Table_Tm_Reg[C_TM_I_BRAKE] 	= 	adc[C_ADCTAB_IBRAKE];
	//Table_Tm_Reg[C_TM_SP_MOT]		=		1;//counter_speed;
	Table_Tm_Reg[C_TM_PWM_MOT]		=		1;//counter_speed;
	Table_Tm_Reg[C_TM_PWM_BRAKE]		=		1;//counter_speed;
*/
void TC_Dispatcher(const uint8_t p_Cmd_Id, const uint16_t p_Param1, const uint16_t p_Param2)
{
	switch(p_Cmd_Id)
	{
		case C_TC_SET_MODE :
			break;
		
		case C_TC_SET_U_MOT :
			ptr_Measure_Motor = &(Table_Tm_Reg[C_TM_U_MOT]);
			C_Motor = p_Param1;
			break;
		
		case C_TC_SET_I_MOT :
			ptr_Measure_Motor = &(Table_Tm_Reg[C_TM_I_MOT]);
			C_Motor = p_Param1;
			break;
		
		case C_TC_SET_SP_MOT :
			ptr_Measure_Motor = &(Table_Tm_Reg[C_TM_SP_MOT]);
			C_Motor = p_Param1;
			break;
		
		case C_TC_SET_PWM_MOT :
			ptr_Measure_Motor = &(Table_Tm_Reg[C_TM_PWM_MOT]);
			C_Motor = p_Param1;
			htim3.Instance->CCR2 = p_Param1;
			break;
		
		case C_TC_SET_CR :
			ptr_Measure_Brake = &(Table_Tm_Reg[C_TM_CR_MOT]);
			C_Motor = p_Param1;
			break;
		
		case C_TC_SET_U_BRAKE :
			ptr_Measure_Brake = &(Table_Tm_Reg[C_TM_U_BRAKE]);
			C_Brake = p_Param1;
			break;
		
		case C_TC_SET_I_BRAKE :
			ptr_Measure_Brake = &(Table_Tm_Reg[C_TM_I_BRAKE]);
			C_Brake = p_Param1;
			break;
		 
		case C_TC_SET_PWM_BRAKE :
			ptr_Measure_Brake = &(Table_Tm_Reg[C_TM_PWM_BRAKE]);
			C_Brake = p_Param1;
			htim3.Instance->CCR2 = p_Param1;
			break;
	}
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
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	htim3.Instance->CCR1 = 0; //50% duty cycle
	htim3.Instance->CCR2 = 00; //25% duty cycle
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Start the PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Start the PWM
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_buffer, 4);
	
	HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 6);
	HAL_SPI_Transmit_IT(&hspi2, buffer_SPI_TX, 6);
	
	//HAL_UART_Transmit_IT(&huart1,buffer_UART_TX,8);
	//HAL_UART_Receive_IT(&huart1,buffer_UART_RX,8);
	HAL_TIM_Base_Start_IT(&htim2);
	//HAL_TIM_Base_Init(&htim2);
	
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
	//HAL_UART_Transmit(&huart1,buffer_UART_TX,8,100);
	while (1)
  {		
		Refresh_TM();
		get_ATMega_Infos();
		if (Is_New_Command_Received())
		{
			/** - Call TC Dispatcher to treat the command, with associated Parameter */
			TC_Dispatcher(Table_Tc_Reg[C_TC_CMD_ID], Table_Tc_Reg[C_TC_PARAM_1_ID],Table_Tc_Reg[C_TC_PARAM_2_ID]);
		}
		
		HAL_SPI_Receive_IT(&hspi2, buffer_SPI_RX, 6);
		//HAL_SPI_Transmit_IT(&hspi2, buffer_SPI_TX, 6);
		//PI_Loop_Motor();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
