/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "alecs_led_lib.h"

char GYR_WHO_AM_I = 0x0f;
char* GYR_CTRL_REG1 = (char*)0x20;
char* GYR_STATUS_REG = (char*)0x27;
char GYR_X_L = 0x28;
char GYR_X_H = 0x29;
char* GYR_OUT_X= (char*)0xa8;
char 	GYR_X = 0xa8;
char 	GYR_Y = 0xaa;
char* GYR_OUT_Y = (char*)0xaa;
char* GYR_SLAVE_ADDR = (char*)0x6B;


void i2c_init_read(uint32_t, uint8_t*);
void i2c_init_write(uint32_t, uint8_t*);

void gyro_init();

void SystemClock_Config(void);
void Error_Handler(void);
void sys_init(){
	/*Enable clocks for I2C GPIOC and GPIOB*/
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	/*set gyro gpio pins to push/pull output mode*/
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	
	//set gpio mode to alternate function open drain output
	GPIOB->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1 ;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_13;
	
	//set gpio alternate function modes*/
	GPIOB->AFR[1] |= (uint32_t)(0x1 << 12); // af1 for gpiob11 i2c2sda 
	GPIOB->AFR[1] |= (uint32_t)(0x5 << 20); // af5 for gpiob13 i2c2scl
	
	//turn on gpiob14 and gpioc0
	GPIOB->BSRR |= GPIO_BSRR_BS_14;
	GPIOC->BSRR |= GPIO_BSRR_BS_0;
	
	/*set timer to 100khz*/
	I2C2->CR2 &= ~I2C_CR2_AUTOEND;	//turn off auto-end
	I2C2->TIMINGR |= (0x1 << 28);		//set prescale value to 1
	I2C2->TIMINGR |= (0x13);				//set scl low period
	I2C2->TIMINGR |= (0x0f << 8);		//set scl high period
	I2C2->TIMINGR |= (0x2 << 16);		//set data hold time
	I2C2->TIMINGR |= (0x4 << 20);		//set data set up time
	I2C2->CR1 |= I2C_CR1_PE;				//enable I2C1 peripheral
	//I2C2->CR2 |= I2C_CR2_START;		//start transmission
	gyro_init();
}
void get_slave_addr(){
	uint8_t who_reg[1];
	who_reg[0] = 0x0f;
	
	i2c_init_write(1,who_reg);
	//I2C2->TXDR |= GYR_WHO_AM_I; //write who am i to txdr
//	while((I2C2->ISR & I2C_ISR_TC) == 0){
//		//enable timeout
//	}
	uint8_t data_buff[1];
	data_buff[0] = 0;
	//I2C2->CR2 |= I2C_CR2_START;
	//I2C2->CR2 |= I2C_CR2_STOP;
	i2c_init_read(1,data_buff);
	uint8_t data_in = data_buff[0];
	if(data_in != GYR_WHO_AM_I){
		//error
	}
	
}

void i2c_init_write(uint32_t num_bytes, uint8_t* buff_to_send){
	I2C2->CR2 &= ~((0x7ff << 16) | (0x3ff << 0));//clear sadd and nbytes
	I2C2->CR2 |= (num_bytes << 16);		//num bytes to trans
	I2C2->CR2 |= (0x6b << 1);		//set slave addr to 0x6b
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;	//clear rd/wrt to write
	//I2C2->CR2 &= ~(0x1<< 10);		//clear rd/wrt to write
	I2C2->CR2 &= ~I2C_CR2_AUTOEND;	//make sure auto end is off
	I2C2->CR2 |= I2C_CR2_START;		//start transmission
	for(int i = 0 ; i < num_bytes; i++){
		while((I2C2->ISR & I2C_ISR_TXIS) == 0){
		}
		I2C2->TXDR = buff_to_send[i];
	}
	while((I2C2->ISR & I2C_ISR_TC) == 0){
		
	}
}

void i2c_init_read(uint32_t num_bytes, uint8_t* data_buff){
	I2C2->CR2 &= ~((0x7f << 16) | (0x3ff << 0));//clear sadd and nbytes
	I2C2->CR2 |= (num_bytes << 16);		//num bytes to read t = 1
	I2C2->CR2 |= (0x6b << 1);		//set slave addr to 0x6b
	I2C2->CR2 &= ~I2C_CR2_AUTOEND;	//make sure auto end is off
	I2C2->CR2 |= I2C_CR2_RD_WRN;	//initiate read
	uint32_t flags = I2C2->ISR;
	uint32_t test = 0;
	I2C2->CR2 |= I2C_CR2_START;		//start transmission
	test |= I2C2->ISR;
	test &= I2C_ISR_RXNE;
	for(int i = 0; i < num_bytes; i++){
		while(!test){
			test |= I2C2->ISR & I2C_ISR_RXNE;
				//enable timeout
		}
		data_buff[i] = I2C2->RXDR;
	}

}

void gyro_init(){
	uint8_t cr1[2];
	uint8_t flags = 0;
	flags |= (0x1 | 0x2 | 0x8);
	cr1[0] = 0x20; //ctrl 1 address
	cr1[1] = flags;
	
	i2c_init_write(2,cr1);
	//I2C2->CR2 |= I2C_CR2_STOP;
	uint8_t gyr_addr[1];
	uint8_t ctrl_reg1_addr[1];
	ctrl_reg1_addr[0]= 0x20;
	
	i2c_init_write(1,ctrl_reg1_addr);
	uint8_t read_buff[1];
	read_buff[0] = 0;
	i2c_init_read(1,read_buff);
	I2C2->CR2 |= I2C_CR2_STOP;
}
int16_t measure_y(){
	uint8_t data_reg[1];
	uint8_t data_reg_low[1]; uint8_t data_reg_high[1];
	data_reg_low[0] = 0x2a; data_reg_high[0] = 0x2b;
	data_reg[0] = GYR_X;
	i2c_init_write(1,data_reg_low);
	//I2C2->TXDR |= GYR_X;
	uint8_t data[2];
	uint8_t data_low[1]; uint8_t data_high[1];
	
	data_low[0] = 0; data_high[0] = 0;
	data[0] = 0; data[1] = 0;
	i2c_init_read(1,data_low);
	I2C2->CR2 |= I2C_CR2_STOP;
	
	i2c_init_write(1,data_reg_high);
	i2c_init_read(1,data_high);
	uint16_t samp_low = (uint16_t)(data_low[0] & 0xff);
	uint16_t samp_hi  = (uint16_t)(data_high[0] & 0xff);
	samp_hi = samp_hi << 8;
	samp_low |= samp_hi;
	turn_off_all_LED();
	int16_t samp = (int16_t)samp_low;
	return samp;
}

int16_t measure_x(){
		uint8_t data_reg[1];
		uint8_t data_reg_low[1]; uint8_t data_reg_high[1];
		data_reg_low[0] = GYR_X_L; data_reg_high[0] = GYR_X_H;
		data_reg[0] = GYR_X;
		i2c_init_write(1,data_reg_low);
		//I2C2->TXDR |= GYR_X;
		uint8_t data[2];
		uint8_t data_low[1]; uint8_t data_high[1];
		
		data_low[0] = 0; data_high[0] = 0;
		data[0] = 0; data[1] = 0;
		i2c_init_read(1,data_low);
		I2C2->CR2 |= I2C_CR2_STOP;
		
		i2c_init_write(1,data_reg_high);
		i2c_init_read(1,data_high);
		uint16_t samp_low = (uint16_t)(data_low[0] & 0xff);
		uint16_t samp_hi  = (uint16_t)(data_high[0] & 0xff);
		samp_hi = samp_hi << 8;
		samp_low |= samp_hi;
		turn_off_all_LED();
		int16_t samp = (int16_t)samp_low;
		return samp;
}

int main(void)
{
	/*Using PB13 for I2C1_SCL, PB11 for I2C1_SDA*/
  HAL_Init();
	
  /* Configure the system clock */
  SystemClock_Config();
	initiate_LEDs();
	sys_init();
	//i2c_init_read(1);
	get_slave_addr();
	while(1){
		int16_t x_samp = measure_x();
		int16_t y_samp = measure_y();
		turn_off_all_LED();
		if(x_samp < -700){
			turn_on_orange_LED();
			if(y_samp < -400){
				turn_on_blue_LED();
			}else if(y_samp > 600){
				turn_on_red_LED();
			}else{
				turn_off_blue_LED();
				turn_off_red_LED();
			}
		}else if( x_samp > 700){
			turn_on_green_LED();
			if(y_samp < -400){
				turn_on_blue_LED();
			}else if(y_samp > 600){
				turn_on_red_LED();
			}else{
				turn_off_blue_LED();
				turn_off_red_LED();
			}
		}else{
			turn_off_green_LED();
			turn_off_orange_LED();
		}
		HAL_Delay(50);
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
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

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  
}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
