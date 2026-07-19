#include "ina228.h"
#include <string.h>
#include <stdio.h>

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