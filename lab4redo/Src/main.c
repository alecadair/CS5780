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

char input_data[2];
char data_count = 0;
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
void transmit_char_toUSART(char c){
		while((USART1->ISR & USART_ISR_TXE) == 0){
			/*insert time out here*/
		}
		//USART2->ICR |= USART_ICR_TCCF; //clear transmission complete flag
		//USART2->CR1 |= USART_CR1_TCIE; //Enable TC interrupt
		USART1->TDR = c;
		
}

void sys_init(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_6;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_6;
	
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;
	//GPIOB->AFR[0] |= (0x1 << 
	USART1->BRR = (HAL_RCC_GetHCLKFreq()/9600);
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; // enable usart transmitter/receiver
	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);				//enable usart1 interrupt in nvic 
	NVIC_SetPriority(USART1_IRQn, 1); 	//configure to high priority
	USART1->CR1 |= USART_CR1_UE; //enable USART
	
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);	//set pins 6 and 7 on GPIOC to output (push/pull)
	GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);	//set pins 6 and 7 on GPIOC to output (push/pull)
	GPIOC->PUPDR = 0;				//turn off all pull up/down resistors for GPIOC
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR6_0 | GPIO_OSPEEDR_OSPEEDR7_0);	//set GPIOC output speed low for pins 6 and 7
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR9_0);	//set GPIOC output speed low for pins 8 and 9
	//GPIOA->AFR[0] |= GPIO_AF
	//RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	//GPIOA->MODER |= 
	//GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR2 | GPIO_OSPEEDR_OSPEEDR3;
	/*select alternate function mode on GPIOA pins 2 & 3*/
	GPIOA->AFR[0] |= (0x1 << 12);
	GPIOA->AFR[0] |= (0x1 << 8);
	USART2->BRR = (HAL_RCC_GetHCLKFreq()/9600);
	
	/*enable receiver and transmitter in UART*/
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
	/*enable the USART*/
	USART2->CR1 |= USART_CR1_UE;
	
	//uint32_t clock_freq = HAL_RCC_GetHCLKFreq();
	//uint32_t baud_rate = clock_freq/9600;
}

void USART1_IRQHandler(void){
	/*
		Test status flags to determine conditions that triggered interrupt
		Only the "receiver register not empty" even is shown
	*/
	char fail = 0;
	if(USART1->ISR & USART_ISR_RXNE){
		USART1->RQR |= USART_RQR_RXFRQ;
		char data = USART1->RDR;
		input_data[data_count] = data;
		if(data_count == 1){
			data_count = 0;
			switch(input_data[0]){
				case 'r':
					if(input_data[1] == '0')
						turn_off_red_LED();
					else if(input_data[1] == '1')
						turn_on_red_LED();
					else if(input_data[1] == '2'){
						if(GPIOC->ODR & GPIO_ODR_6)
							turn_off_red_LED();
						else
							turn_on_red_LED();
					}else{
						transmit_char_toUSART('E');
						transmit_char_toUSART('R');
						transmit_char_toUSART('R');
						transmit_char_toUSART('\r');
						transmit_char_toUSART('\n');
						fail = 1;
					}
					break;
				case 'b':
					if(input_data[1] == '0')
						turn_off_blue_LED();
					else if(input_data[1] == '1')
						turn_on_blue_LED();
					else if(input_data[1] == '2'){
						if(GPIOC->ODR & GPIO_ODR_7)
							turn_off_blue_LED();
						else
							turn_on_blue_LED();
					}else{
						transmit_char_toUSART('E');
						transmit_char_toUSART('R');
						transmit_char_toUSART('R');
						transmit_char_toUSART('\r');
						transmit_char_toUSART('\n');
						fail = 1;
					}
					break;
				case 'o':
					if(input_data[1] == '0')
						turn_off_orange_LED();
					else if(input_data[1] == '1')
						turn_on_orange_LED();
					else if(input_data[1] == '2'){
						if(GPIOC->ODR & GPIO_ODR_8)
							turn_off_orange_LED();
						else
							turn_on_orange_LED();
					}else{
						transmit_char_toUSART('E');
						transmit_char_toUSART('R');
						transmit_char_toUSART('R');
						transmit_char_toUSART('\r');
						transmit_char_toUSART('\n');
						fail = 1;
					}
					break;
				case 'g':
					if(input_data[1] == '0')
						turn_off_green_LED();
					else if(input_data[1] == '1')
						turn_on_green_LED();
					else if(input_data[1] == '2'){
						if(GPIOC->ODR & GPIO_ODR_9)
							turn_off_green_LED();
						else
							turn_on_green_LED();
					}else{
						transmit_char_toUSART('E');
						transmit_char_toUSART('R');
						transmit_char_toUSART('R');
						transmit_char_toUSART('\r');
						transmit_char_toUSART('\n');
						fail = 1;
					}
					break;
					default:
						transmit_char_toUSART('E');
						transmit_char_toUSART('R');
						transmit_char_toUSART('R');
						transmit_char_toUSART('\r');
						transmit_char_toUSART('\n');
						data_count = 0;
						fail = 1;
					break;
						
			}
			if(!fail){
				transmit_char_toUSART(input_data[0]);
				transmit_char_toUSART(input_data[1]);
				transmit_char_toUSART('\r');
				transmit_char_toUSART('\n');
			}
			transmit_char_toUSART('C');
			transmit_char_toUSART('M');
			transmit_char_toUSART('D');
			transmit_char_toUSART('\r');
			transmit_char_toUSART('\n');
					
		}else{
			data_count++;
		}		
		//transmit_char_toUSART(data);
		//rxbuf_push(USART1->RDR); // save data in rx registers
	}
}

int main(void)
{
	/*using USART2 PA2 = TX PA3 = RX*/
  HAL_Init();
  SystemClock_Config();
	sys_init();
	char str[5];
	str[0] = 'a';
	str[1] = 'l';
	str[2] = 'e';
	str[3] = 'c';
	str[4] = '\0';
	int i = 0;
	for(; i < 5; i++)
		transmit_char_toUSART(str[i]);
	//transmit_char_toUSART(str[i]);
	transmit_char_toUSART('\r');
	transmit_char_toUSART('\n');
	transmit_char_toUSART('C');
	transmit_char_toUSART('M');
	transmit_char_toUSART('D');
	transmit_char_toUSART('\r');
			transmit_char_toUSART('\n');
  while (1)
  {
		
		//transmit_char_toUSART(str[i]);
		HAL_Delay(10);
		if(i < 4)
			i++;
		else
			i = 0;
/*		if(data_count == 0){
			transmit_char_toUSART('C');
			transmit_char_toUSART('M');
			transmit_char_toUSART('D');
			transmit_char_toUSART('\r');
			transmit_char_toUSART('\n');
		}*/
		__WFI();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

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