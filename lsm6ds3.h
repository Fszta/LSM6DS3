#include "stdint.h"

/* DEVICE REGISTER MAPPING ----------------------------------------------------------*/
#define READ 								0x80
#define LSM6DS3_WAI 				0x69
#define WHO_AM_I 						0x0F
#define DEVICE_RECOGNIZED		0x01
#define DEVICE_NOT_RECOGNIZED 0x02

#define CTRL1_XL 						0x10
#define CTRL2_G							0x11
#define CTRL3_C 						0x12
#define CTRL4_C 						0x13
#define CTRL5_C 						0x14
#define CTRL6_C							0x15
#define CTRL7_G							0x16
#define CTRL8_XL						0x17
#define CTRL9_XL 						0x18
#define CTRL10_C						0x19

#define MASTER_CONFIG				0x1A
#define WAKE_UP_SRC					0x1B
#define TAP_SRC							0x1C
#define D6D_SRC							0x1D
#define STATUS_REG					0x1E

#define OUT_TEMP_L					0x20
#define OUT_TEMP						0x21
#define	OUTX_L_G						0x22
#define OUTX_H_G						0x23
#define OUTY_L_G						0x24
#define OUTY_H_G						0x25
#define OUTZ_L_G						0x26
#define OUTZ_H_G						0x27
#define OUTX_L_XL 					0x28
#define OUTX_H_XL 					0x29
#define OUTY_L_XL 					0x2A
#define OUTY_H_XL 					0x2B
#define OUTZ_L_XL 					0x2C
#define OUTZ_H_XL 					0x2D

#define SENSORHUB1_REG			0x2E
#define SENSORHUB2_REG			0x2F
#define SENSORHUB3_REG			0x30
#define SENSORHUB4_REG			0x31
#define SENSORHUB5_REG			0x32
#define SENSORHUB6_REG			0x33
#define SENSORHUB7_REG			0x34
#define SENSORHUB8_REG			0x35
#define SENSORHUB9_REG			0x36
#define SENSORHUB10_REG			0x37
#define SENSORHUB11_REG			0x38
#define SENSORHUB12_REG			0x39
#define FIFO_STATUS1				0x3A
#define FIFO_STATUS2				0x3B
#define FIFO_STATUS3				0x3C
#define FIFO_STATUS4				0x3D
#define FIFO_DATA_OUT_L			0x3E
#define FIFO_DATA_OUT_H			0x3F
#define TIMESTAMP0_REG			0x40
#define TIMESTAMP1_REG			0x41
#define TIMESTAMP2_REG			0x42
#define STEP_TIMESTAMP_L		0x49
#define STEP_TIMESTAMP_H		0x4A
#define STEP_COUNTER_L			0x4B
#define STEP_COUNTER_H			0x4C
#define SENSORHUB13_REG			0x4D
#define SENSORHUB14_REG			0x4E
#define SENSORHUB15_REG			0x4F
#define SENSORHUB16_REG			0x50
#define SENSORHUB17_REG			0x51
#define SENSORHUB18_REG			0x52
#define FUNC_SRC						0x53
#define TAP_CFG 						0x58
#define WAKE_UP_THS 				0x5B
#define WAKE_UP_DUR 				0x5C
#define FREE_FALL						0x5D
#define MD1_CFG 						0x5E
#define MD2_CFG							0x5F
#define OUT_MAG_RAW_X_L			0x66
#define OUT_MAG_RAW_X_H			0x67
#define OUT_MAG_RAW_Y_L			0x68
#define OUT_MAG_RAW_Y_H			0x69
#define OUT_MAG_RAW_Z_L			0x6A
#define OUT_MAG_RAW_Z_H			0x6B

#define M_PI   							3.14159265358979323846264338327950288


/* Private function prototypes -----------------------------------------------*/

/* Init SPI function */
static void MX_SPI3_Init(void);

/* Function returning angles from one device */
double *get_angle(uint8_t CS_PIN_ID);

/* Check Who am i register */
void test_device(void); 

/* Function for writing a value into a register */
void write_register(uint8_t reg, uint8_t value);

/* Function for reading register */
int read_register(uint8_t register_address); 

/* Init registers at the start of the programm */
void configure_register(void);

void low_power(void);

void power_down(void);

/* Get acceleration for each axes */
int get_acceleration(uint8_t register_accel_L, uint8_t register_accel_H);

/* Compute angle from acceleration */
double compute_angle(int16_t axe_1, int16_t axe_2);

/* Wake up interrupt */
void setup_wake_up(void);

/* Structure for acceleration */
typedef struct Acceleration Acceleration;
struct Acceleration
{
	int16_t x;
	int16_t y;
	int16_t z;
};
