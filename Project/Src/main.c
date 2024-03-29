/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "pins.h"
#include <stdbool.h>
#include "motor.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim15;
volatile uint32_t pMillis;
volatile uint16_t startTime;
volatile uint16_t Value1 = 0;
volatile uint16_t Value2 = 0;
volatile uint16_t leftDistance = 0;
volatile uint16_t rightDistance = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LED_RED GPIO_PIN_6
#define LED_BLUE GPIO_PIN_7
#define LED_ORANGE GPIO_PIN_8
#define LED_GREEN GPIO_PIN_9
#define TOGGLE_LED(led) (HAL_GPIO_TogglePin(GPIOC, led))

void LED_Init(uint32_t pins) {
	/* __HAL_RCC_GPIOC_CLK_ENABLE(); */

	GPIO_InitTypeDef gpio_led_init = {pins,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	HAL_GPIO_Init(GPIOC, &gpio_led_init);
}

/**
 * @brief Enable proper GPIO pins and set up their alternate
 *        functions appropriately.
 *        Pins defined in pins.h and pins.c.
 */
void GPIO_AF_Init() {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// uart_pins
	HAL_GPIO_Init(uart_pins.rx.gpio, &(uart_pins.rx.pin));
	HAL_GPIO_Init(uart_pins.tx.gpio, &(uart_pins.tx.pin));

	// motor_left_pins
	HAL_GPIO_Init(motor_left_pins.enable.gpio, &motor_left_pins.enable.pin);
	HAL_GPIO_Init(motor_left_pins.dir_a.gpio, &motor_left_pins.dir_a.pin);
	HAL_GPIO_Init(motor_left_pins.dir_b.gpio, &motor_left_pins.dir_b.pin);
	HAL_GPIO_Init(motor_left_pins.enc_a.gpio, &motor_left_pins.enc_a.pin);
	HAL_GPIO_Init(motor_left_pins.enc_b.gpio, &motor_left_pins.enc_b.pin);

	// motor_right_pins
	HAL_GPIO_Init(motor_right_pins.enable.gpio, &motor_right_pins.enable.pin);
	HAL_GPIO_Init(motor_right_pins.dir_a.gpio, &motor_right_pins.dir_a.pin);
	HAL_GPIO_Init(motor_right_pins.dir_b.gpio, &motor_right_pins.dir_b.pin);
	HAL_GPIO_Init(motor_right_pins.enc_a.gpio, &motor_right_pins.enc_a.pin);
	HAL_GPIO_Init(motor_right_pins.enc_b.gpio, &motor_right_pins.enc_b.pin);

	// ultrasonic_left_pins
	HAL_GPIO_Init(ultrasonic_left_pins.echo.gpio, &ultrasonic_left_pins.echo.pin);
	HAL_GPIO_Init(ultrasonic_left_pins.trig.gpio, &ultrasonic_left_pins.trig.pin);

	// ultrasonic_right_pins
	HAL_GPIO_Init(ultrasonic_right_pins.echo.gpio, &ultrasonic_right_pins.echo.pin);
	HAL_GPIO_Init(ultrasonic_right_pins.trig.gpio, &ultrasonic_right_pins.trig.pin);
}

/**
 * @brief Enable the USART3 peripheral.
 */
void USART_Init() {
	__HAL_RCC_USART3_CLK_ENABLE();
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	//Set baud rate
	USART3->CR1 &= ~(1<<15);
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600; //this is the baud rate we need to 
												//use with the BT adapter

	//Enable transmitter
	USART3->CR1 |= (1<<3);

	//Enable receiver
	USART3->CR1 |= (1<<2);

	//Enable the receive register not empty interrupt
	USART3->CR1 |= USART_CR1_RXNEIE;

	//Enable USART3
	USART3->CR1 |= 1;
}

/**
 * @brief Sends a single character on the USART.
 *
 * @param c: the character to be sent
 */
void USART_SendChar(uint8_t c) {
	while(!(USART3->ISR & USART_ISR_TXE))
		;

	USART3->TDR = c;
}

/**
 * @brief Sends a string on the USART.
 *
 * @param p_string: pointer to string to send
 */
void USART_SendString(uint8_t* p_string) {
	while(*p_string != 0) {
		USART_SendChar(*p_string);
		p_string++;
	}
	USART_SendChar(0);
}

volatile uint8_t command[3];
volatile uint8_t incomingCommand = 0;

void ClearCommand(){
	uint8_t prompt[] = "CMD> ";
	memset(command, 0, sizeof(command));
	incomingCommand = 0;
	USART_SendString(prompt);
}

void USART3_4_IRQHandler(){
	uint8_t err[] = "Command too long!\n";
	incomingCommand = 1;
	while(!(USART3->ISR & (1<<5))){
	}
	if(command[0]){
		if(command[1]){
			USART_SendString(err);
			ClearCommand();
		}
		else{
			command[1] = USART3->RDR;
			ProcessCommand(command[0], command[1]);
		}
	}
	else{
		command[0] = USART3->RDR;
		if(command[0] != 'w' &&
				command[0] != 'a' &&
				command[0] != 'd'){
			ProcessCommand(command[0], NULL);
		}
	}
}

void Log(){
	uint8_t str_buff[32];

	sprintf(str_buff, "Ultrasonic left: %d\n", GetUltrasonic(&ultrasonic_left_pins));
	USART_SendString(str_buff);

	sprintf(str_buff, "Ultrasonic right: %d\n", GetUltrasonic(&ultrasonic_right_pins));
	USART_SendString(str_buff);

	sprintf(str_buff, "Distance travelled: %d\n", (int)get_distance());
	USART_SendString(str_buff);

	sprintf(str_buff, "Heading: %d\n", (int)get_heading());
	USART_SendString(str_buff);

}

void Proceed(){
	uint8_t* err = "Not yet implemented\n";
	USART_SendString(err);
}

void ProcessCommand(uint8_t direction, uint8_t distance){
	uint8_t err[] = "ERROR: Invalid command!\n";

	uint8_t forward[] = "Moving forward ";
	uint8_t left[] = "Turning left ";
	uint8_t right[] = "Turning right ";
	uint8_t log[] = "Logging sensor data\n";
	uint8_t proceed[] = "Entering autonomous mode...\n";
	uint8_t stop[] = "Stopping all motors!\n";

	MotorCommand motorcmd = {0};

	uint8_t* part1;
	uint8_t part2[15];
	uint8_t zero = 0;
	memcpy(part2, &zero, 15);

	switch(direction){
		case 'w':
			part1 = forward;
			motorcmd.dir = FORWARD;
			break;
		case 'a':
			part1 = left;
			motorcmd.dir = LEFT;
			break;
		case 'd':
			part1 = right;
			motorcmd.dir = RIGHT;
			break;
		case 'l':
			part1 = log;
			USART_SendString(part1);
			Log();
			ClearCommand();
			return;
		case 'p':
			part1 = proceed;
			USART_SendString(part1);
			Proceed();
			ClearCommand();
			return;
		case 'x':
			part1 = stop;
			USART_SendString(part1);
			motorcmd.dir = OFF;
			MoveMotors(&motorcmd);
			ClearCommand();
			return;
		default:
			USART_SendString(err);
			ClearCommand();
			return;
	}

	//these are for "vector commands" only:
	if(distance == '0' && direction == 'w'){
		sprintf(part2, "indefinitely\n");
		motorcmd.amount = 0;
	}
	else if(distance < '1' || distance > '9'){
		USART_SendString(err);
		ClearCommand();
		return;
	}
	else{
		uint8_t dist = (distance - '0') * 20;
		if(dist == 80)
			dist = 90; //support for 90-degree turns
		motorcmd.amount = dist;
	}


	USART_SendString(part1);
	USART_SendString(part2);

	/* anyone know what this does?? or why it's in process command lol
	 * HAL_Delay hangs when in an interrupt context, due to SysTick being
	 * lower priority.
	 * This is called from the USART interrupt, so this can't be here.
	 * If you need it, use timers instead.
	TIM2->CCR1 = CH1_DC;
	HAL_Delay(1000);
	TIM2->CCR1 = 0;
	*/

	uint8_t* result = MoveMotors(&motorcmd);
	USART_SendString(result);

	ClearCommand();

}

void PrintDistance()
{
	char dist[32] = "";
	sprintf(dist, "%f\n", get_distance());
	USART_SendString(dist);
}

uint32_t GetUltrasonic(ultrasonic_pins_t* ultrasonic){
	uint32_t i = 0, j = 0;
	TIM15->EGR |= 1; //clears counter
	while(TIM15->EGR & 1)
		;
	HAL_GPIO_WritePin(ultrasonic->trig.gpio, ultrasonic->trig.pin.Pin, GPIO_PIN_SET);
	while (TIM15->CNT < 200){
	}
	HAL_GPIO_WritePin(ultrasonic->trig.gpio, ultrasonic->trig.pin.Pin, GPIO_PIN_RESET);

	pMillis = TIM15->CNT;
	while (!(HAL_GPIO_ReadPin (ultrasonic->echo.gpio, ultrasonic->echo.pin.Pin)) && pMillis + 200 >  TIM15->CNT){
		i++;
	}
	uint32_t val1 = TIM15->CNT;

	pMillis = val1;
	while ((HAL_GPIO_ReadPin (ultrasonic->echo.gpio, ultrasonic->echo.pin.Pin)) && pMillis + 1200 > TIM15->CNT){
		j++;
	}

	uint32_t val2 = TIM15->CNT;
	if(val2-val1 == 1200)
		return -1;
	return (val2-val1)/4.42; //this is a magic number that gives us centimeters with our prescaler value
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	HAL_Init();								// Initialize HAL
	SystemClock_Config();

	//USART Initalizations
	GPIO_AF_Init();
	USART_Init();

	motor_init();                           // Initialize motor code

	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,2);

	//PWM and Ultrasonic Initalizations
	MX_GPIO_Init();
	MX_TIM15_Init(); 

	while (1) {
		//Handled in interrupts.
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
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
}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{
	//hal machine Broke
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	TIM15->PSC |= 100;
	TIM15->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	/* __HAL_RCC_GPIOA_CLK_ENABLE(); */

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add their own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
