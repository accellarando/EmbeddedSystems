/**
 * C file to initiate pin structs with values.
 */

#include "pins.h"
#include "stm32f0xx_hal.h"

// UART Pins
uart_pins_t uart_pins = {
    .rx = {
        .gpio = GPIOC,
        .pin = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF1_USART3
        }
    },
    .tx = {
        .gpio = GPIOC,
        .pin = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF1_USART3
        }
    }
};

// Motor Pins
motor_pins_t motor_left_pins = {
    .enable = {
        .gpio = GPIOA,
        .pin = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF4_TIM14
        }
    },
    .dir_a = {
        .gpio = GPIOA,
        .pin = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = 0
        }
    },
    .dir_b = {
        .gpio = GPIOA,
        .pin = {
            .Pin = GPIO_PIN_6,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = 0
        }
    },
    .enc_a = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_4,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF1_TIM3
        }
    },
    .enc_b = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF1_TIM3
        }
    }
};

motor_pins_t motor_right_pins = {
    .enable = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_3,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF2_TIM2
        }
    },
    .dir_a = {
        .gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_2,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = 0
		}
	},
	.dir_b = {
		.gpio = GPIOB,
        .pin = {
            .Pin = GPIO_PIN_10,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = 0
		}
	},
	.enc_a = {
		.gpio = GPIOB,
		.pin = {
            .Pin = GPIO_PIN_14,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF1_TIM15
		}
	},
	.enc_b = {
		.gpio = GPIOB,
		.pin = {
            .Pin = GPIO_PIN_15,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF1_TIM15
		}
	}
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
