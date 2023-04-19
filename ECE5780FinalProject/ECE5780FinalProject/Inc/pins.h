/**
 * This header file contains all pin definitions for the project.
 */
#ifndef __PINS_H
#define __PINS_H

#include <stdint.h>
#include "stm32f0xx_hal.h"

typedef struct{
	GPIO_TypeDef gpio;
	GPIO_InitTypeDef pin;
} pin_t;

typedef struct{
	pin_t rx;
	pin_t tx;
} uart_pins_t;
extern uart_pins_t uart_pins;

typedef struct{
	pin_t enable;
	pin_t dir_a;
	pin_t dir_b;
	pin_t enc_a;
	pin_t enc_b;
} motor_pins_t;
extern motor_pins_t motor_left_pins;
extern motor_pins_t motor_right_pins;

typedef struct{
	pin_t echo;
	pin_t trig;
} ultrasonic_pins_t;
extern ultrasonic_pins_t ultrasonic_left_pins;
extern ultrasonic_pins_t ultrasonic_right_pins;

#endif
