//extern double *test();
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


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
double *get_angle();
void test_device(void); 
void write_register(uint8_t reg, uint8_t value);
int read_register(uint8_t register_address); 
void configure_register(void);
int get_acceleration(uint8_t register_accel_L, uint8_t register_accel_H);
double compute_angle(int16_t axe_1, int16_t axe_2);