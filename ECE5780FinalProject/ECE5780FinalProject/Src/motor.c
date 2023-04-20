/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motor.h"

volatile int16_t error_integral = 0;    // Integrated error signal
volatile uint8_t duty_cycle = 0;    	// Output PWM duty cycle
volatile int16_t target_rpm = 0;    	// Desired speed target
volatile int16_t motorl_speed = 0;   	// Measured left motor speed
volatile int16_t motorr_speed = 0;   	// Measured left motor speed
volatile int8_t adc_value = 0;      	// ADC measured motor current
volatile int16_t error = 0;         	// Speed error signal
volatile uint8_t Kp = 1;            	// Proportional gain
volatile uint8_t Ki = 1;            	// Integral gain

// Sets up the entire motor drive system
void motor_init(void) {
    pwm_init();
    encoder_init();
    ADC_init();
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void) {
	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
    // Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)
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
			pwm_setDutyCycleL(100);
			pwm_setDutyCycleR(100);
			break;
		case BACKWARD:
			HAL_GPIO_WritePin(pins->dir_a.gpio, pins->dir_a.pin.Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(pins->dir_b.gpio, pins->dir_b.pin.Pin, GPIO_PIN_RESET);
			pwm_setDutyCycleR(100);
			pwm_setDutyCycleL(100);
			break;
		default:
			pwm_setDutyCycleL(0);
			pwm_setDutyCycleR(0);
			;
	}
}

void set_Forward(){
	//left go forward
	set_Motor_Direction(FORWARD, &motor_left_pins);

	//right go forward
	set_Motor_Direction(FORWARD, &motor_right_pins);

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
	
	/*
		GPIOA->ODR |= (1 << 8);
		GPIOA->ODR &= ~(1 << 5);
		GPIOB->ODR |= (1 << 10);
		GPIOB->ODR &= ~(1 << 2);
	*/
}

void set_Off(){
	set_Motor_Direction(OFF, &motor_left_pins);
	set_Motor_Direction(OFF, &motor_right_pins);
}

uint8_t* MoveMotors(MotorCommand* cmd){
	set_Off();
	set_On();
	uint8_t* err = "MoveMotors executed!\n";
	switch(cmd->dir){
		case FORWARD:
			set_Forward();
			break;
		case LEFT:
			set_Left();
			break;
		case RIGHT:
			set_Right();
			break;
		default:
			err = "Invalid command to MoveMotors!\n";
	}
	return err;
}

// Sets up encoder interface to read motor speed
void encoder_init(void) {
    
    // Set up encoder input pins (TIMER 3 CH1 and CH2)
    GPIOB->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->AFR[0] |= ( (1 << 16) | (1 << 20) );
		
		// Set up encoder input pins (TIMER 15 CH1 and CH2)
		GPIOB->MODER &= ~(GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
    GPIOB->MODER |= (GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);
    GPIOB->AFR[1] |= ( (1 << 24) | (1 << 28) );

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
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
		
		RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->CCMR1 = 0;
    TIM15->CCER = 0;
    TIM15->SMCR = 0;
    TIM15->CR1 = 0;

    TIM15->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM15->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM15->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM15->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM15->CR1 |= TIM_CR1_CEN;     

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    
    // Select PSC and ARR values that give an appropriate interrupt rate
    TIM6->PSC = 11;
    TIM6->ARR = 30000;
    
    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    motorl_speed = (TIM3->CNT - 0x7FFF);
    TIM3->CNT = 0x7FFF; // Reset back to center point
	  motorr_speed = (TIM15->CNT - 0x7FFF);
    TIM15->CNT = 0x7FFF; // Reset back to center point
    
    // Call the PI update function
    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

void ADC_init(void) {

    // Configure PA1 for ADC input (used for current monitoring)
    GPIOA->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1);

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;                        // Default resolution is 12-bit (RES[1:0] = 00 --> 12-bit)
    ADC1->CFGR1 |= ADC_CFGR1_CONT;          // Set to continuous mode
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1;      // Enable channel 1

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
}