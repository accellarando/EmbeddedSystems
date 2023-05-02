# Pinouts for the Project

## USART
| Function | Pin | Configuration |
| --- | --- | --- |
| RX | PC5 | AF1: USART3\_RX |
| TX | PC4 | AF1: USART3\_TX |

## Left Motor
| Function | Pin | Configuration |
| --- | --- | --- |
| Enable | PA4 | PWM: AF4: TIM14\_CH1 |
| DirectionA | PA5 | Output |
| DirectionB | PC9 | Output |
| EncoderA | PB4 | AF1: TIM3\_CH1 |
| EncoderB | PB5 | AF1: TIM3\_CH2 |

## Right Motor
| Function | Pin | Configuration |
| --- | --- | --- |
| Enable | PB3 | PWM: AF2: TIM2\_CH2 |
| DirectionA | PB2 | Output |
| DirectionB | PB10 | Output |
| EncoderA | PA8 | AF2: TIM1\_CH1 |
| EncoderA | PA9 | AF2: TIM1\_CH2 |

## Left Ultrasonic Sensor
| Function | Pin | Configuration |
| --- | --- | --- |
| Echo | PB11 | Input |
| Trig | PB12 | Output |

## Right Ultrasonic Sensor
| Function | Pin | Configuration |
| --- | --- | --- |
| Echo | PB14 | Input |
| Trig | PB13 | Output |
