/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motor.h"
#include "main.h"
#include <math.h>

volatile int16_t motorl_speed = 0;   	// Measured left motor speed
volatile int16_t motorr_speed = 0;   	// Measured left motor speed
volatile float target_dist = 0;
volatile float current_dist = 0;
volatile float absolute_dist = 0;
volatile float heading = 0;
volatile bool turning = false;

#define max(a,b) ((a>b) ? a : b)
#define min(a,b) ((a>b) ? b : a)

#define PRINT_DEBUG 0
#define STOP_ULTRASONIC 20 //cm
#define INFINITY 5000 //long long course lmao

// Sets up the entire motor drive system
void motor_init(void) {
	pwm_init();
	encoder_init();
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void) {
	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)
	/*
	   GPIOA->MODER |= (1 << 9);
	   GPIOA->MODER &= ~(1 << 8);

	// Set up pin PB3 for H-bridge PWM output (TIMER 2 CH2)
	GPIOB->MODER |= (1 << 7);
	GPIOB->MODER &= ~(1 << 6);

	// Set PA4 to AF4,
	GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,
	GPIOA->AFR[0] |= (1 << 18);

	// Set PB3 to AF2,
	GPIOB->AFR[0] &= 0xFFFF0FFF; // clear PB3 bits,
	GPIOB->AFR[0] |= (1 << 13);

	// Set up a PA5, PA8 as GPIO output pins for motor direction control
	GPIOA->MODER &= 0xFFFCF3FF; // clear PA5, PA6 bits,
	GPIOA->MODER |= (1 << 10) | (1 << 16);

	// Set up a PB2, PB10 as GPIO output pins for motor direction control
	GPIOB->MODER &= 0xFFCFFFCF; // clear PB2, PB10 bits,
	GPIOB->MODER |= (1 << 4) | (1 << 20);
	*/

	//Initialize one direction pin to high, the other low
	HAL_GPIO_WritePin(motor_left_pins.dir_a.gpio, motor_left_pins.dir_a.pin.Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor_left_pins.dir_b.gpio, motor_left_pins.dir_b.pin.Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(motor_right_pins.dir_a.gpio, motor_right_pins.dir_a.pin.Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor_right_pins.dir_b.gpio, motor_right_pins.dir_b.pin.Pin, GPIO_PIN_RESET);

	// Set up PWM timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->CR1 = 0;                         // Clear control registers
	TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
	TIM14->CCER = 0;

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 = 0;                         // Clear control registers
	TIM2->CCMR1 = 0;                       // (prevents having to manually clear bits)
	TIM2->CCER = 0;

	// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
	TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
	TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
	TIM14->PSC = 1;                         // Run timer on 24Mhz
	TIM14->ARR = 1200;                      // PWM at 20kHz
	TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle

	TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer

	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
	TIM2->CCER |= TIM_CCER_CC2E;           // Enable capture-compare channel 2
	TIM2->PSC = 1;                         // Run timer on 24Mhz
	TIM2->ARR = 1200;                      // PWM at 20kHz
	TIM2->CCR2 = 0;                        // Start PWM at 0% duty cycle

	TIM2->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM Left, accepts (0-100)
void pwm_setDutyCycleL(uint8_t duty) {
	if(duty <= 100) {
		TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
														// (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
	}
}

// Set the duty cycle of the PWM Right, accepts (0-100)
void pwm_setDutyCycleR(uint8_t duty) {
	if(duty <= 100) {
		TIM2->CCR2 = ((uint32_t)duty*TIM2->ARR)/100;  // Use linear transform to produce CCR1 value
													  // (CCR2 == "pulse" parameter in PWM struct used by peripheral library)
	}
}

/*
 * This sets an individual motor's direction.
 * Note: use MoveMotors to change multiple motor values, eg to turn.
 */
void set_Motor_Direction(Direction dir, motor_pins_t* pins){
	switch(dir){
		case FORWARD:
			HAL_GPIO_WritePin(pins->dir_a.gpio, pins->dir_a.pin.Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(pins->dir_b.gpio, pins->dir_b.pin.Pin, GPIO_PIN_RESET);
			break;
		case BACKWARD:
			HAL_GPIO_WritePin(pins->dir_a.gpio, pins->dir_a.pin.Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(pins->dir_b.gpio, pins->dir_b.pin.Pin, GPIO_PIN_SET);
			break;
		default:
			;
	}
}

//Experimentation led to this 91 number for driving straight.
volatile uint8_t pwm_right = 100;
volatile uint8_t pwm_left = 100;

void set_Forward(){
	//left go forward
	set_Motor_Direction(FORWARD, &motor_left_pins);

	//right go forward
	set_Motor_Direction(FORWARD, &motor_right_pins);

	pwm_setDutyCycleR(pwm_right);
	pwm_setDutyCycleL(pwm_left);


	/* the old way
	   GPIOA->ODR |= (1 << 5);
	   GPIOA->ODR &= ~(1 << 8);
	   GPIOB->ODR |= (1 << 10);
	   GPIOB->ODR &= ~(1 << 2);
	   */
}

void set_Backward(){
	//left go backward
	set_Motor_Direction(BACKWARD, &motor_left_pins);

	//right go backward
	set_Motor_Direction(BACKWARD, &motor_right_pins);
	pwm_setDutyCycleR(pwm_right);
	pwm_setDutyCycleL(pwm_left);
	/* the old way
	   GPIOA->ODR |= (1 << 8);
	   GPIOA->ODR &= ~(1 << 5);
	   GPIOB->ODR |= (1 << 2);
	   GPIOB->ODR &= ~(1 << 10);
	   */
}

void set_Right(){
	//left go forward
	set_Motor_Direction(FORWARD, &motor_left_pins);

	//right go backward
	set_Motor_Direction(BACKWARD, &motor_right_pins);
	pwm_setDutyCycleR(pwm_right);
	pwm_setDutyCycleL(pwm_left);
	/* old way
	   GPIOA->ODR |= (1 << 5);
	   GPIOA->ODR &= ~(1 << 8);
	   GPIOB->ODR |= (1 << 2);
	   GPIOB->ODR &= ~(1 << 10);
	   */
}

void set_Left(){
	//left go backward
	set_Motor_Direction(BACKWARD, &motor_left_pins);

	//right go forward
	set_Motor_Direction(FORWARD, &motor_right_pins);

	pwm_setDutyCycleR(pwm_right);
	pwm_setDutyCycleL(pwm_left);
	/*
	   GPIOA->ODR |= (1 << 8);
	   GPIOA->ODR &= ~(1 << 5);
	   GPIOB->ODR |= (1 << 10);
	   GPIOB->ODR &= ~(1 << 2);
	   */
}

void motors_Off(){
	pwm_setDutyCycleR(0);
	pwm_setDutyCycleL(0);
}

uint8_t* MoveMotors(MotorCommand* cmd){
	motors_Off();
	pwm_right = 100;
	uint8_t* err = "MoveMotors executed!\n";
	switch(cmd->dir){
		case FORWARD:
			turning = false;
			if(cmd->amount==0.0)
				target_dist = INFINITY;
			else
				target_dist = cmd->amount;
			set_Forward();
			break;
		case LEFT:
			turning = true;
			target_dist = cmd->amount / 11.4592;
			heading += cmd->amount * 0.0174533; //convert to radians
			set_Left();
			break;
		case RIGHT:
			turning = true;
			target_dist = cmd->amount / 11.4592;
			heading -= cmd->amount * 0.0174533; //convert to radians
			set_Right();
			break;
		case OFF:
			target_dist = 1.0;
			motors_Off();
			break;
		default:
			err = "Invalid command to MoveMotors!\n";
	}
	return err;
}

// Sets up encoder interface to read motor speed
void encoder_init(void) {

	// Set up encoder input pins (TIMER 3 CH1 and CH2)
	/*
	   GPIOB->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
	   GPIOB->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
	   GPIOB->AFR[0] |= ( (1 << 16) | (1 << 20) );

	// Set up encoder input pins (TIMER 15 CH1 and CH2)
	GPIOB->MODER &= ~(GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
	GPIOB->MODER |= (GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);
	GPIOB->AFR[1] |= ( (1 << 24) | (1 << 28) );
	*/

	// Set up encoder interface (TIM3 encoder input mode)
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM3->CCMR1 = 0;
	TIM3->CCER = 0;
	TIM3->SMCR = 0;
	TIM3->CR1 = 0;

	TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
	TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
	TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
	TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
															// (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
															//  just another option, the mid-bias is a bit simpler to understand though.)
	TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer

	TIM1->CCMR1 = 0;
	TIM1->CCER = 0;
	TIM1->SMCR = 0;
	TIM1->CR1 = 0;
	TIM1->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
	TIM1->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
	TIM1->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
	TIM1->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
															// (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
															//  just another option, the mid-bias is a bit simpler to understand though.)
	TIM1->CR1 |= TIM_CR1_CEN;     

	// Configure a second timer (TIM6) to fire an ISR on update event
	// Used to periodically check and update speed variable
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Select PSC and ARR values that give an appropriate interrupt rate
	TIM6->PSC = 11;
	TIM6->ARR = 30000;

	TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
	TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

	NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
	NVIC_SetPriority(TIM6_DAC_IRQn,3);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
#define ULTRASONIC_SAMPLES 1

uint8_t ObjectDetected(){
	uint32_t us_left_sum = 0;
	uint32_t us_right_sum = 0;

	for(uint8_t i=0;i<ULTRASONIC_SAMPLES;i++){
		us_left_sum += GetUltrasonic(&ultrasonic_left_pins);
		us_right_sum += GetUltrasonic(&ultrasonic_right_pins);
	}

	/* us_left_sum >>= 1; //div by 2 */
	/* us_right_sum >>= 1; */

	return (us_left_sum < STOP_ULTRASONIC) || (us_right_sum < STOP_ULTRASONIC);
}

void TIM6_DAC_IRQHandler(void) {
	/* Calculate the motor speed in raw encoder counts
	 * Note the motor speed is signed! Motor can be run in reverse.
	 * Speed is measured by how far the counter moved from center point
	 */
	uint8_t usart_buffer[32];


	motorl_speed = (TIM3->CNT - 0x7FFF);
	/* sprintf(usart_buffer, "motorl: %d\n", motorl_speed); */
	/* USART_SendString(usart_buffer); */
	TIM3->CNT = 0x7FFF; // Reset back to center point
	motorr_speed = (TIM1->CNT - 0x7FFF);
	TIM1->CNT = 0x7FFF; // Reset back to center point

	if(abs(motorl_speed)>50){
		float ratio = ((float) abs(motorl_speed))/ ((float) abs(motorr_speed));
		pwm_right = min((int)(pwm_right * ratio), 100);
		pwm_setDutyCycleR(pwm_right);
#if PRINT_DEBUG
		sprintf(usart_buffer, "pwm_right: %d\n", pwm_right);
		USART_SendString(usart_buffer);
		sprintf(usart_buffer, "motorr: %d\n", motorr_speed);
		USART_SendString(usart_buffer);
#endif
	}

	if(target_dist != 0.0){
		current_dist += (float)abs(motorl_speed)/132;
#if PRINT_DEBUG
		sprintf(usart_buffer, "current_dist: %d\n", (int)current_dist);
		USART_SendString(usart_buffer);
#endif
		uint8_t obj = 0;
		if ((current_dist >= target_dist) || (!turning && (obj = ObjectDetected()))){
			if(obj)
				USART_SendString("Object detected\n");
			else
				USART_SendString("Hit distance target\n");
			motors_Off();
			motorl_speed = (TIM3->CNT - 0x7FFF);
			TIM3->CNT = 0x7FFF; // Reset back to center point
			motorr_speed = (TIM1->CNT - 0x7FFF);
			TIM1->CNT = 0x7FFF; // Reset back to center point
			pwm_right = 0;
			if(!turning){
				absolute_dist += current_dist * (float)cos((double)heading);
				sprintf(usart_buffer, "heading: %d\n", (int)heading);
				USART_SendString(usart_buffer);
			}
			target_dist = 0.0;
			current_dist = 0.0;
		}
	}

	// Call the PI update function
	TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

float get_distance(void){
	return absolute_dist;
}

int32_t get_heading(){
	return (int32_t)(heading * 57.3) % 360;
}
