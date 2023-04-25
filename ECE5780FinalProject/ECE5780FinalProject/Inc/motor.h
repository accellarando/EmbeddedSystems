
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "pins.h"
#include <stdbool.h>
#include <math.h>

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern volatile int16_t motor_speed;   // Measured motor speed
extern volatile uint8_t target_dist;
extern volatile float current_dist;
extern volatile float absolute_dist;
extern volatile float heading;
extern volatile bool turning;
									   //
extern volatile uint8_t pwm_right;
extern volatile uint8_t pwm_left;


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

// Distance return
float get_distance(void);

uint8_t* MoveMotors(MotorCommand* cmd);


/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void);

// Sets up encoder interface to read motor speed
void encoder_init(void);

#endif /* MOTOR_H_ */
