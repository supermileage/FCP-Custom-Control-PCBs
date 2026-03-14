#include "ina228.h"
#include <string.h>
#include <stdio.h>


// EDIT: need to add the peripherials, systemclock, and two other functions within the platform.io instead of using stm32cubemx
 UART_HandleTypeDef huart2;
 I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART2_UART_Init(void);

// --- System Clock Configuration ---
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | 
                                  RCC_CLOCKTYPE_SYSCLK | 
                                  RCC_CLOCKTYPE_PCLK1 | 
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00707CBB; // Standard 100kHz for L4 series
    HAL_I2C_Init(&hi2c1);
}

void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart2);
}


static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    uart_print("\r\n--- NUCLEO STARTING ---\r\n");

    INA228_Handle_t ina;
    INA228_Init(&ina, &hi2c1, 0x40, 10.0f, 30.0f);

    if (!INA228_Begin(&ina, 0x0000, 0xFB68, false))
        uart_print("INA228 init failed\r\n");
    else
        uart_print("INA228 init OK\r\n");

    while (1)
    {
        float current_a = 0.0f;
        if (INA228_ReadCurrentA(&ina, &current_a))
        {
            char msg[64];
            snprintf(msg, sizeof(msg), "Current: %.4f A\r\n", current_a);
            uart_print(msg);
        }
        else
        {
            uart_print("Read current failed\r\n");
        }

        HAL_Delay(500);
    }
}
