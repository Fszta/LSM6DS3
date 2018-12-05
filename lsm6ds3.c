/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "lsm6ds3.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

double angles[4],angle_rad, angle_deg;
uint8_t TxBuffer[2], RxBuffer[2], init_flag = 0, test;
uint16_t CS_PIN;
uint8_t test_who;
char STATUS;

/**
  * @brief  Get angle calculated from all accelerometer.
  * @param CS_PIN_ID: Chip select pin.
  * @retval *angles : address of the angles .
  */
double *get_angle(uint8_t CS_PIN_ID)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Initialisation when call the function for the first time */
		if (init_flag == 0)
		{
			/* Init SPI communication*/
			MX_SPI3_Init();
			init_flag = 1;
		}
			
		if (CS_PIN_ID == 0)
		{
			CS_PIN = GPIO_PIN_3;
		}
		
		else 
		{
			CS_PIN = GPIO_PIN_2;
		}

		/* Check the WHO AM I register of the device */
		test_device();
		
		/* Configure the registers for accelerometer */
		configure_register();
		
		/* Get acceleration for each axes */
		Acceleration acceleration;
		acceleration.x = get_acceleration(OUTX_L_XL,OUTX_H_XL);
		acceleration.y = get_acceleration(OUTY_L_XL,OUTY_H_XL);
		acceleration.z = get_acceleration(OUTZ_L_XL,OUTZ_H_XL);
			
		/* Compute angles from acceleration */
		if(CS_PIN_ID == 0)
			{
				angles[0] = compute_angle(acceleration.y, acceleration.z);
				angles[1] = compute_angle(acceleration.x, acceleration.z);
			}
		else 
		{
			angles[2] = compute_angle(acceleration.y, acceleration.z);
			angles[3] = compute_angle(acceleration.x, acceleration.z);
		}
		
		/* Return an array containing the angles */
		return angles;
}


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

/**
  * @brief  Check that the device is correctly recognized.
  * @param  None
  * @retval None
  */
void test_device(void) 
{
	test_who = read_register(WHO_AM_I);
	
	if(test_who==LSM6DS3_WAI) 
	{
		STATUS = DEVICE_RECOGNIZED;
	}
	
	else 
	{
		STATUS = DEVICE_NOT_RECOGNIZED;
	}
}

/**
  * @brief  This function write a  value into a register
  * @param  reg: The address of the register to write in
	* @param  value: value to write in the register
  * @retval None
  */
void write_register(uint8_t reg, uint8_t value)
{
	/* Set chip select to 0 to enable SPI communication */
	HAL_GPIO_WritePin(GPIOD,CS_PIN,GPIO_PIN_RESET);
	TxBuffer[0] = reg;
	TxBuffer[1] = value;
	
	HAL_SPI_Transmit(&hspi3,TxBuffer,2,50);

	HAL_GPIO_WritePin(GPIOD, CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, CS_PIN, GPIO_PIN_RESET);	
	TxBuffer[0] = (uint8_t) (reg|0x80);
	HAL_SPI_Transmit(&hspi3, TxBuffer,1,50);
	HAL_SPI_Receive(&hspi3, RxBuffer,1,50);
	
	/* Set chip select to 1 to disable SPI communication */
	HAL_GPIO_WritePin(GPIOD, CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  This function read a register.
  * @param  register_address: The address of the register to read.
  * @retval value of the readed register.
  */
int read_register(uint8_t register_address)
{
	uint8_t val;
	
	/* ¨Put CS to 0 : enable spi communication */
	HAL_GPIO_WritePin(GPIOD,CS_PIN, GPIO_PIN_RESET);
	
	/* Transmit READ address */
	TxBuffer[0] = (uint8_t) (READ|register_address);
	HAL_SPI_Transmit(&hspi3,TxBuffer,1,50);
	
	/* Read value of register address */
	HAL_SPI_Receive(&hspi3,RxBuffer,1,50);
	val = RxBuffer[0];
	
	HAL_GPIO_WritePin(GPIOD, CS_PIN, GPIO_PIN_SET);
	
	return val;
}

/**
  * @brief  Init registers for normal mode configuration.
  * @param  None.
  * @retval None.
  */
void configure_register(void)
{
	/* ODR_XL = 56Hz (Low Power), FS_XL = 2G, BW_XL = 400Hz */
	write_register(CTRL1_XL,0x30);
	write_register(CTRL3_C, 0x44);
	write_register(CTRL4_C,0x00);
	
	/* Accelerometer only mode */
	write_register(CTRL6_C,0x10);
	
	/* Accelerometer X,Y,Z axis output enabled, soft-iron disabled */
	write_register(CTRL9_XL,0x38);
	write_register(CTRL10_C,0x04);
}

/**
  * @brief  Get linear acceleration value, expressed as a 16-bit word.
  * @param  register_acceleration L.
	* @param  register_acceleration H.
  * @retval acceleration.
  */
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

/**
  * @brief  Compute angle from acceleration.
	* @param  axe_1: acceleration first axe
	* @param  axe_2: acceleration second axe
  * @retval angle.
  */
double compute_angle(int16_t axe_1, int16_t axe_2)
{		
	/* Compute angle, in radians */
	angle_rad = atan2(axe_1,axe_2);
	
	/* Convert radians in degrees */
	angle_deg = angle_rad * 180.0 / M_PI;
	
	return angle_deg;
}

/**
  * @brief  Put accelerometer in power down mode 
	* @param  None
  * @retval None
  */
void power_down(void) 
{
	/* Put accelerometer in power down mode 
	 * Power consumption : 6 µA
	 */
	write_register(CTRL1_XL, 0x00);
}

/**
  * @brief  Put accelerometer in low power mode 
	* @param  None
  * @retval None
  */
void low_power(void)
{
	/* Output data rate : 52 Hz
	 * Power consumption : 45 µA
	 */ 
	write_register(CTRL1_XL,0x30);
}
