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
#include "main.h"
#include "string.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define LED_RED GPIO_PIN_6
#define LED_BLUE GPIO_PIN_7
#define LED_ORANGE GPIO_PIN_8
#define LED_GREEN GPIO_PIN_9
#define TOGGLE_LED(led) (HAL_GPIO_TogglePin(GPIOC, led))
/**
 * @brief Initialize LEDs on the given pins.
 *
 * @param pins - pins to enable
 */
void LED_Init(uint32_t pins) {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio_led_init = {pins,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_LOW,
        GPIO_NOPULL
    };
    HAL_GPIO_Init(GPIOC, &gpio_led_init);
}

/**
 * @brief Enable proper GPIO pins for USART, and set up their alternate
 *        functions appropriately.
 */
void GPIO_AF_Init() {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio_af_init = {GPIO_PIN_4 | GPIO_PIN_5,
        GPIO_MODE_AF_PP,
        GPIO_SPEED_FREQ_HIGH,
        GPIO_NOPULL
    };
    HAL_GPIO_Init(GPIOC, &gpio_af_init);

    //Set AF1 on PC4 and PC5
    GPIOC->AFR[0] &= ~(0xFF << 16);
    GPIOC->AFR[0] |= (0x11 << 16);
}

/**
 * @brief Enable the USART3 peripheral.
 */
void USART_Init() {
    //__HAL_RCC_USART3_CLK_ENABLE();
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    //Set baud rate
    USART3->CR1 &= ~(1<<15);
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;

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

volatile uint8_t command[2];
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
    while(!(USART3->ISR & (1<<5)))
        ;
    if(command[0])
        if(command[1]){
            USART_SendString(err);
            ClearCommand();
        }
        else
            command[1] = USART3->RDR;
    else
        command[0] = USART3->RDR;

}

void ProcessCommand(uint8_t led, uint8_t mode){
    uint32_t led_pin;

    uint8_t err[] = "ERROR: Invalid command!\n";

    uint8_t off[] = "Turning off the ";
    uint8_t on[] = "Turning on the ";
    uint8_t toggle[] = "Toggling the ";


    uint8_t red[] = "red LED.\n";
    uint8_t green[] = "green LED.\n";
    uint8_t blue[] = "blue LED.\n";
    uint8_t orange[] = "orange LED.\n";

    if(led != 'r' && led != 'o' && led != 'b' && led != 'g'){
        USART_SendString(err);
        ClearCommand();
        return;
    }

    uint8_t* part1;
    uint8_t* part2;

    switch(led){
        case 'r':
            part2 = red;
            led_pin = LED_RED;
            break;
        case 'g':
            part2 = green;
            led_pin = LED_GREEN;
            break;
        case 'b':
            part2 = blue;
            led_pin = LED_BLUE;
            break;
        case 'o':
            part2 = orange;
            led_pin = LED_ORANGE;
            break;
        default:
            USART_SendString(err);
            ClearCommand();
    }
    switch(mode){
        case '0':
            part1 = off;
            HAL_GPIO_WritePin(GPIOC, led_pin, GPIO_PIN_RESET);
            break;
        case '1':
            part1 = on;
            HAL_GPIO_WritePin(GPIOC, led_pin, GPIO_PIN_SET);
            break;
        case '2':
            part1 = toggle;
            TOGGLE_LED(led_pin);
            break;
        default:
            USART_SendString(err);
            ClearCommand();
            return;
    }

    USART_SendString(part1);
    USART_SendString(part2);
    ClearCommand();
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */

    GPIO_AF_Init();
    USART_Init();
    LED_Init(LED_RED | LED_ORANGE | LED_GREEN | LED_BLUE);

    NVIC_EnableIRQ(USART3_4_IRQn);
    NVIC_SetPriority(USART3_4_IRQn,1);

    uint8_t prompt[] = "CMD> ";

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    USART_SendString(prompt);
    while (1)
    {
        /* USER CODE END WHILE */

        //Send an 'a'
        /*
           uint8_t hi[] = "hello world";
           USART_SendString(hi);
           HAL_Delay(1000);
           */


        /* USER CODE BEGIN 3 */
        if(incomingCommand){
            if(command[1]){
                ProcessCommand(command[0], command[1]);
            }
        }
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
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
            |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
      LD4_Pin LD5_Pin */
    GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
        |LD4_Pin|LD5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
    GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : SPI2_SCK_Pin SPI2_MISO_Pin SPI2_MOSI_Pin */
    GPIO_InitStruct.Pin = SPI2_SCK_Pin|SPI2_MISO_Pin|SPI2_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    /* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */