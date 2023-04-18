/**
 * C file to initiate pin structs with values.
 */

#include "pins.h"
#include "stm32f0xx_hal.h"

// USART pins
pin_t usart_rx = {
    .gpio = GPIOC,
    .pin = GPIO_PIN_5,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF1_USART3
};

pin_t usart_tx = {
    .gpio = GPIOC,
    .pin = GPIO_PIN_4,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF1_USART3
};

uart_pins_t uart_pins = {
    .rx = usart_rx,
    .tx = usart_tx
};

// Left motor pins
pin_t motor_left_enable = {
    .gpio = GPIOA,
    .pin = GPIO_PIN_4,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF4_TIM14
};

pin_t motor_left_dir_a = {
    .gpio = GPIOA,
    .pin = GPIO_PIN_5,
    .mode = GPIO_MODE_OUTPUT_PP,
    .af = 0
};

pin_t motor_left_dir_b = {
    .gpio = GPIOA,
    .pin = GPIO_PIN_6,
    .mode = GPIO_MODE_OUTPUT_PP,
    .af = 0
};

pin_t motor_left_enc_a = {
    .gpio = GPIOB,
    .pin = GPIO_PIN_4,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF1_TIM3
};

pin_t motor_left_enc_b = {
    .gpio = GPIOB,
    .pin = GPIO_PIN_5,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF1_TIM3
};

motor_pins_t motor_left_pins = {
    .enable = motor_left_enable,
    .dir_a = motor_left_dir_a,
    .dir_b = motor_left_dir_b,
    .enc_a = motor_left_enc_a,
    .enc_b = motor_left_enc_b
};

// Right motor pins
pin_t motor_right_enable = {
    .gpio = GPIOB,
    .pin = GPIO_PIN_3,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF2_TIM2
};

pin_t motor_right_dir_a = {
    .gpio = GPIOB,
    .pin = GPIO_PIN_2,
    .mode = GPIO_MODE_OUTPUT_PP,
    .af = 0
};

pin_t motor_right_dir_b = {
    .gpio = GPIOB,
    .pin = GPIO_PIN_10,
    .mode = GPIO_MODE_OUTPUT_PP,
    .af = 0
};

pin_t motor_right_enc_a = {
    .gpio = GPIOB,
    .pin = GPIO_PIN_14,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF1_TIM15
};

pin_t motor_right_enc_b = {
    .gpio = GPIOB,
    .pin = GPIO_PIN_15,
    .mode = GPIO_MODE_AF_PP,
    .af = GPIO_AF1_TIM15
};

motor_pins_t motor_right_pins = {
    .enable = motor_right_enable,
    .dir_a = motor_right_dir_a,
    .dir_b = motor_right_dir_b,
    .enc_a = motor_right_enc_a,
    .enc_b = motor_right_enc_b
};

// Ultrasonic Sensor Pins
ultrasonic_pins_t ultrasonic_left_pins = {
    .echo = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_11,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = 0
        }
    },
    .trig = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_12,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = 0
        }
    }
};

ultrasonic_pins_t ultrasonic_right_pins = {
    .echo = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_13,
            .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = 0
        }
    },
    .trig = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_14,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = 0
        }
    }
};
