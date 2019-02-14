#pragma once

#include "stm32f0xx_hal.h"
#include "init.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);