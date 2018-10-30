///* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "test2.h"

/* Variable declaration */
double *address_angles, angle_x, angle_y;


int main(void)
{

  /* Infinite loop */
  while (1)
  {	
			/* Return pointer on the first element of the array containing the angles */
			address_angles = get_angle();
			
			/* Get the first angle */
			angle_x = *address_angles;
			
			/* Get the second angle */
			angle_y = *(address_angles + 1); 
  }
}

