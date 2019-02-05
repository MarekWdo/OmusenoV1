/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l0xx.h"
#include "DistanceSensor.h"

I2C_HandleTypeDef i2c;

uint8_t a;

int main(void)
{
	SystemCoreClock = 8000000;
	HAL_Init();
	// Initialise SysTick to tick at 1ms by initialising it with SystemCoreClock (Hz)/1000
	SysTick_Config(SystemCoreClock / 1000);

	__HAL_RCC_I2C1_CLK_ENABLE();
	GPIO_InitTypeDef gpioI2C;
	gpioI2C.Mode = GPIO_MODE_AF_OD;
	gpioI2C.Pin = GPIO_PIN_6 | GPIO_PIN_7; //SCL, SDA
	gpioI2C.Pull = GPIO_PULLUP;
	gpioI2C.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpioI2C);

	i2c.Instance = I2C1;
	i2c.Init.Timing = 0x00707CBB;
	i2c.Init.OwnAddress1 = 0xff;
	i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c.Init.OwnAddress2 = 0xff;
	i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	HAL_I2C_Init(&i2c);

	DistanceSensor distanceSensor;
	DistanceSensor_ctor(&distanceSensor, &i2c);
	uint8_t address = distanceSensor.address;
	int x = 0;
	for(;;)
	{
		x++;
	}
}
