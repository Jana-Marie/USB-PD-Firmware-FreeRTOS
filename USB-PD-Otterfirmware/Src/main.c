#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "init.h"
#include <stdint.h>

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;

osThreadId defaultTaskHandle;

void dfu_otter_bootloader(void);
void StartDefaultTask(void const * argument);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
 
  if(HAL_GPIO_ReadPin(GPIOA,BUTTON_Pin) == 1) {
    dfu_otter_bootloader();
  }

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osKernelStart();
  
  while (1)
  {
  }
}

void StartDefaultTask(void const * argument)
{
  HAL_GPIO_TogglePin(GPIOA,LED_POWER_Pin);
  for(;;)
  {
    osDelay(1);
    HAL_GPIO_TogglePin(GPIOA,LED_STATUS_Pin);
  }
}

void dfu_otter_bootloader(void)
{
  *((unsigned long *)0x20003FF0) = 0xDEADBEEF;
  NVIC_SystemReset();
}

void _Error_Handler(char *file, int line)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{ 

}
#endif /* USE_FULL_ASSERT */