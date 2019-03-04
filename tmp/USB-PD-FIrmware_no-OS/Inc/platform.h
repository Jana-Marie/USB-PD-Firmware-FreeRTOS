#include "stm32f0xx_hal.h"

void platform_usleep(uint64_t us) {
  HAL_Delay_Microseconds(us);
}

