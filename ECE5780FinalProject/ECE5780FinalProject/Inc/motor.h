
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "pins.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern volatile int16_t error_integral;    // Integrated error signal
extern volatile uint8_t duty_cycle;    // Output PWM duty cycle
extern volatile int16_t target_rpm;    // Desired speed target
extern volatile int16_t motor_speed;   // Measured motor speed
extern volatile int8_t adc_value;      // ADC measured motor current
extern volatile int16_t error;         // Speed error signal
extern volatile uint8_t Kp;            // Proportional gain
extern volatile uint8_t Ki;            // Integral gain


enum direction{
	FORWARD,
	LEFT,
	RIGHT,
	BACKWARD,
	OFF
};
typedef enum direction Direction;

typedef struct{
	Direction dir;
	uint8_t amount;
} MotorCommand;

/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the entire motor drive system
void motor_init(void);

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycleL(uint8_t duty);
void pwm_setDutyCycleR(uint8_t duty);

// Set direction of movement
void set_Forward();
void set_Backward();
void set_Left();
void set_Right();

// PI control code is called within a timer interrupt
void PI_update(void);

uint8_t* MoveMotors(MotorCommand* cmd);


/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void);

// Sets up encoder interface to read motor speed
void encoder_init(void);

// Sets up ADC to measure motor current
void ADC_init(void);

#endif /* MOTOR_H_ */
