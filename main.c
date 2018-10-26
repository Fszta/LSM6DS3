/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define READ 0x80
#define LSM6DS3_WAI 0x69
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL3_C 0x12
#define CTRL4_C 0x13
#define CTRL9_XL 0x18
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D
#define M_PI   3.14159265358979323846264338327950288
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void test_device(void); 
void write_register(uint8_t reg, uint8_t value);
int read_register(uint8_t register_address); 
int get_acceleration(uint8_t register_accel_L, uint8_t register_accel_H);
double compute_angle(int16_t axe_1, int16_t axe_2);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t Tx[2], Rx[2], TxB[2], RxB[2], spiTxBuf[2], spiRxBuf[2], spiTxBuff[2], spiRxBuff[2], TxBuffer[2], RxBuffer[2], TB[2], RB[2];
int16_t x, y, z, x_accel, y_accel, z_accel;
double angle1, angle2, angle_rad, angle_deg;
//uint8_t test_device;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	uint8_t TxBuf[2], RxBuf[2];
	RxBuf[0] = 0x00;
	RxBuf[1] = 0x00;
	RxB[0] = 0x00;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		//test_device = read_register(WHO_AM_I);
		write_register(CTRL1_XL,0x30);
		
		x_accel = get_acceleration(OUTX_L_XL,OUTX_H_XL);
		y_accel = get_acceleration(OUTY_L_XL,OUTY_H_XL);
		z_accel = get_acceleration(OUTZ_L_XL,OUTZ_H_XL);
		
		angle1 = compute_angle(y_accel,z_accel);
		angle2 = compute_angle(x_accel,z_accel);
		
		
		if (Rx[0] == 0x30)
		{
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12|GPIO_PIN_13);
			HAL_Delay(100);
			
		}
		test_device();
//		TxBuf[0] = WHO_AM_I | READ;

//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);	
//		HAL_SPI_Transmit(&hspi3, TxBuf, 1, 50);
//		HAL_SPI_Receive(&hspi3, RxBuf, 1, 50);
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
//		
//		if (RxBuf[0] == LSM6DS3_WAI) 
//			{
//				HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14|GPIO_PIN_15);
//				HAL_Delay(100);
//			}
		
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		TxBuf[0] = 0x13;
		TxBuf[1] = 0x00;
		
		HAL_SPI_Transmit(&hspi3,TxBuf,2,50);

		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);	
		TxBuf[0] = (uint8_t) (0x13|0x80);
		HAL_SPI_Transmit(&hspi3, TxBuf,1,50);
		HAL_SPI_Receive(&hspi3, RxBuf,1,50);
		
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		TxB[0] = 0x18;
		TxB[1] = 0x38;
		
		HAL_SPI_Transmit(&hspi3,TxB,2,50);

		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);	
		TxB[0] = (uint8_t) (0x18|0x80);
		HAL_SPI_Transmit(&hspi3, TxB,1,50);
		HAL_SPI_Receive(&hspi3, RxB,1,50);
		
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI3 init function */
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void test_device(void) 
{
	uint8_t TxBuf[2], RxBuf[2];
	TxBuf[0] = WHO_AM_I | READ;

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);	
	HAL_SPI_Transmit(&hspi3, TxBuf, 1, 50);
	HAL_SPI_Receive(&hspi3, RxBuf, 1, 50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
		
	if (RxBuf[0] == LSM6DS3_WAI) 
	{
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14|GPIO_PIN_15);
		HAL_Delay(100);
	}
}

/* Write register function */
void write_register(uint8_t reg, uint8_t value)
{
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	Tx[0] = reg;
	Tx[1] = value;
	
	HAL_SPI_Transmit(&hspi3,Tx,2,100);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);	
	Tx[0] = (uint8_t) (0x10|0x80);
	HAL_SPI_Transmit(&hspi3, Tx,1,100);
	HAL_SPI_Receive(&hspi3, Rx,1,100);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
}

/* Read register function */
int read_register(uint8_t register_address)
{
	uint8_t val;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	TxBuffer[0] = (uint8_t) (READ|register_address);
	HAL_SPI_Transmit(&hspi3,TxBuffer,1,100);
	HAL_SPI_Receive(&hspi3,RxBuffer,1,100);
	val = RxBuffer[0];
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
	
	return val;
}


/* Get linear acceleration value, expressed as a 16-bit word */
int get_acceleration (uint8_t register_accel_L, uint8_t register_accel_H) 
{
	int16_t accel;
	uint8_t accel_Lsb, accel_Msb;
	
	/* Get linear acceleration (LSByte) */
	accel_Lsb = read_register(register_accel_L);
	
	/* Get linear acceleration (MSByte) */
	accel_Msb = read_register(register_accel_H);
	
	accel = (accel_Msb <<8) + accel_Lsb;
	return accel;
	
}	


/* Compute angle from acceleration */
double compute_angle(int16_t axe_1, int16_t axe_2)
{		
	
	
	/* Compute angle, in radians */
	angle_rad = atan2(axe_1,axe_2);
	
	/* Convert radians in degrees */
	angle_deg = angle_rad * 180.0 / M_PI;
	
	return angle_deg;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
