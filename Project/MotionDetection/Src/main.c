/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

#define MEASURE 0
#define LAST_PULSE 1

void SystemClock_Config(void);
void Error_Handler(void);
uint8_t test = 0;
uint8_t timer_count = 0;
uint8_t red = 0;
uint8_t blue = 0;
uint8_t green = 0;
uint8_t orange = 0;
const uint8_t THRESHOLD = 10;// 1s = 100ms * 50 //threshold for lights to begin turning off
uint16_t pulse_time = 0;
uint8_t signal_detected = 0;
void transmit_char_toUSART(char c);
void transmit_string(char* word);
void process_last_pulse();
void process_measure();
void process_command();
const uint16_t MSG_SIZE = 1024;

char last_command[MSG_SIZE];
	
uint16_t command_len = 0;

/*Control UART messages*/
char measure[MSG_SIZE] = "measure\n";
char last_pulse[MSG_SIZE] = "last_pulse\n";

char* commands[2] = {measure, last_pulse};

void sys_init(void){
	/*Send AHB clock to GPIOC*/
	HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	//configure GPIO for UART
	GPIOA->AFR[0] |= 0x1 << 12;
	GPIOA->AFR[0] |= 0x1 << 8;//select alternate function 1 for pa2 and pa3
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;
	
	USART1->BRR |= (HAL_RCC_GetHCLKFreq()/9600);
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
	USART1->CR1 |= USART_CR1_RXNEIE;
	
	USART2->BRR = HAL_RCC_GetHCLKFreq()/9600;
	
	
	GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR6_0|GPIO_OSPEEDR_OSPEEDR7_0|GPIO_OSPEEDR_OSPEEDR8_0|GPIO_OSPEEDR_OSPEEDR9_0;	
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;					//enable pull down resistor
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;	//configure multiplexing 
	EXTI->RTSR |= EXTI_RTSR_TR0;	//set interrupt to rising edge
	EXTI->IMR |= EXTI_IMR_MR0;		//set line pa0 to interrupt capable
	//set timer interrupt for every 100ms
	TIM16->PSC = 1025;
	TIM16->ARR = 782;
	TIM16->CR1 = TIM_CR1_CEN;//enable timer
	TIM16->DIER = TIM_CR1_CEN;
	//EXTI->PR |= EXTI_PR_PR0 ;
	NVIC_SetPriority(EXTI0_1_IRQn,2);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(TIM16_IRQn,3);
	NVIC_EnableIRQ(TIM16_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 1);	
	/*enable receiver and transmitter in UART*/
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
	/*enable the USART*/
	USART2->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_UE;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	
}
void transmit_char_toUSART(char c){
		while((USART1->ISR & USART_ISR_TXE) == 0){
			/*insert time out here*/
		}
		//USART2->ICR |= USART_ICR_TCCF; //clear transmission complete flag
		//USART2->CR1 |= USART_CR1_TCIE; //Enable TC interrupt
		USART1->TDR = (char)c;
}
void transmit_string(char* word){
	uint16_t i = 0;
	for(; word[i] != '\0'; i++){
		transmit_char_toUSART(word[i]);
	}
	return;
}

uint8_t strings_equal(volatile char s1[MSG_SIZE], volatile char s2[MSG_SIZE]){
	uint16_t i = 0;
	for(;i < MSG_SIZE; i++){
		if(s1[i] != s2[i])
			return 0;
	}
	return 1;
}

void process_last_pulse(){
	
}

void process_measure(){
	
}

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

void process_command(){
	return;
}

int main(void)
{
	//SysTick_Config(HAL_RCC_GetHCLKFreq());
	sys_init();
	while(1)
		__WFI();
}
void USART1_IRQHandler(void){
	char data = 0;
	if(USART1->ISR & USART_ISR_RXNE){
		USART1-> RQR |= USART_RQR_RXFRQ;
		data = USART1->RDR;
		transmit_char_toUSART(data);
		if(data == '\r')
			last_command[command_len] = '\n';
		else
			last_command[command_len] = data;
		command_len++;
		if(data == '\r'){
			process_command();
			for(uint16_t i = 0; i < MSG_SIZE; i++){
				last_command[i] = 0;
			}
			command_len = 0;
		}
	}
	return;
}
void TIM16_IRQHandler(void){
	if(TIM16->SR & TIM_SR_UIF)
		TIM16->SR &= ~TIM_SR_UIF;
	timer_count++;
	if(timer_count == 2*THRESHOLD && signal_detected)
		turn_off_green_LED();
	if(timer_count == 3*THRESHOLD && signal_detected)
		turn_off_blue_LED();
	if(timer_count == 4*THRESHOLD && signal_detected)
		turn_off_orange_LED();
	if(timer_count == 5*THRESHOLD && signal_detected){
		turn_off_red_LED();
		signal_detected = 0;
		timer_count = 0;
	}
}

void EXTI0_1_IRQHandler(void){
	char* msg = "Motion Detected\n\0";
	
	transmit_string(msg);
	while(GPIOA->IDR & 0x1){
		turn_on_red_LED();
		turn_on_blue_LED();
		turn_on_green_LED();
		turn_on_orange_LED();
	}
	pulse_time = 0;
	timer_count = 0;
	signal_detected = 1;
  EXTI->PR |= EXTI_PR_PR0 ;

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
