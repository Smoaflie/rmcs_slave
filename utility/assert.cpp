/**
 * @author Qzh (zihanqin2048@gmail.com)
 * @brief The assertion error handling.
 * @copyright Copyright (c) 2023 by Alliance, All Rights Reserved.
 */

#include <cassert>
#include <cstdio>
#include <main.h>
#include <spi.h>

#include "app/logger/logger.hpp"

const char* __assert_file = nullptr;
int __assert_line = 0;
const char* __assert_function = nullptr;
const char* __assert_expression = nullptr;

void __assert_func(const char* file, int line, const char* function, const char* expression) {
    __disable_irq();
    __assert_file = file;
    __assert_line = line;
    __assert_function = function;
    __assert_expression = expression;

    char buffer[128];
    sprintf(buffer, "Assertion failed: %s, function %s, file %s, line %d.\n",
        expression, function, file, line);
    logger::Logger().printf(buffer);

    while (true){
        static uint8_t txbuf[24];
        const uint8_t WS2812_HighLevel = 0xf0;
        const uint8_t WS2812_LowLevel  = 0xC0;
        uint32_t color = 0x020000; 
        for (int i = 0; i < 8; i++)
        {
            txbuf[7-i]  = (((color>>(i+8))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
            txbuf[15-i] = (((color>>(i+16))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
            txbuf[23-i] = (((color>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        }
        HAL_SPI_Transmit(&hspi6, (uint8_t *)txbuf, 24, 1);
        HAL_Delay(500);  // 延迟以便观察
    }
}