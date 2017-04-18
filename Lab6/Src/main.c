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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void sys_init(void){
	/*Send AHB clock to GPIOC*/
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	//RCC->CR2 |= RCC_CR2_HSI14ON;
	//RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	/*Configure GPIO LEDs on board*/
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR6_0|GPIO_OSPEEDR_OSPEEDR7_0|GPIO_OSPEEDR_OSPEEDR8_0|GPIO_OSPEEDR_OSPEEDR9_0;	
	GPIOC->MODER |= GPIO_MODER_MODER4;
	/*Set PB0 to analog input mode*/
	//GPIOB->MODER |= GPIO_MODER_MODER1;
	//GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR1;
	/*configure ADC to 8-bit continous conversion mode*/
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	//ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
	/*Set ADC channel to 8 which is connected to pin PB0*/
	ADC1->CHSELR |= ADC_CHSELR_CHSEL14;
	
	/*Calibrate ADC*/
	if((ADC1->CR & ADC_CR_ADEN) != 0){
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while((ADC1->CR & ADC_CR_ADEN) != 0){
		/*implement a time-out here*/
	}
	//ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0){
		/*implement a time-out here*/
	}
	
	/*Enable sequence code*/
	if((ADC1->ISR & ADC_ISR_ADRDY) != 0){
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0){
		/*implement a time out here*/
	}
	
	//ADC1->CR |= ADC_CR_ADCAL;
	//while(!ADC1->CR & ADC_CR_ADCAL); //should have a time out incase ADC does not work.
	
	//if((ADC1->ISR & ADC_ISR_ADRDY) != 0){
	//	ADC1->ISR |= ADC_ISR_ADRDY;
	//}
	/*Turn the ADC on*/
	//ADC1->CR |= ADC_CR_ADEN;
	//while((ADC1->ISR & ADC_ISR_ADRDY) == 0);


	/*Make sure that no pull up/down resistors are enabled on GPIOB0*/
	//GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk;
		/*Enable apb bus clock for ADC*/
	//RCC->APB2ENR |= RCC_APB2ENR_ADCEN;



	//ADC1->CR |= ADC_CR_ADSTART;

	
}

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void turn_on_red_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_6;
}

void turn_on_green_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_9;
}

void turn_on_blue_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_7;
}

void turn_on_orange_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BS_8;
}

void turn_off_orange_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_8;
}

void turn_off_blue_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_7;
}

void turn_off_green_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_9;
}

void turn_off_red_LED(){
	GPIOC->BSRR |= GPIO_BSRR_BR_6;
}

int main(void)
{
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	SysTick_Config(HAL_RCC_GetHCLKFreq());
	sys_init();
	uint8_t reading = 0;
			ADC1->CR |= ADC_CR_ADSTART;

	//ADC1->CFGR1 |= ADC_CFGR1_ALIGN;
	//ADC->CCR |= ADC_CCR_VREFEN;
	//ADC1->CR |= ADC_CR_ADSTART;
  while (1)
  {
		while((ADC1->ISR & ADC_ISR_EOC) == 0);
		//ADC1->CR |= ADC_CR_ADSTART;
		//while(ADC1->
		reading = ADC1->DR;
		//uint8_t div = reading/55;
		//switch(div){
			if(reading <60){
				turn_off_green_LED();
				turn_off_blue_LED();
				turn_off_orange_LED();
				turn_on_red_LED();
			}else if(reading <128){
				turn_on_green_LED();
				turn_off_blue_LED();
				turn_off_orange_LED();
				turn_off_red_LED();
			}else if(reading < 192){
				turn_on_green_LED();
				turn_on_blue_LED();
				turn_off_orange_LED();
				turn_off_red_LED();
			}else if(reading < 228){
				turn_on_green_LED();
				turn_on_blue_LED();
				turn_on_orange_LED();
				turn_off_red_LED();
			}else{
				turn_on_green_LED();
				turn_on_blue_LED();
				turn_on_orange_LED();
				turn_on_red_LED();
			}	
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
