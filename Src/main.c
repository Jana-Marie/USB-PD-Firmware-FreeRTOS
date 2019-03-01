#include "main.h"
#include "stddef.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usb_pd_driver.h"
#include "init.h"
#include "usbd_cdc_if.h"
#include "tcpm_driver.h"
#include "usb_pd.h"
#include "usb_pd_tcpm.h"

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;

osThreadId defaultTaskHandle;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);

const struct tcpc_config_t tcpc_config[CONFIG_USB_PD_PORT_COUNT] = {
  {&hi2c1, fusb302_I2C_SLAVE_ADDR, &fusb302_tcpm_drv},
};

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osKernelStart();

  while (1)
  {
  }
}

void dfu_otter_bootloader(void)
{
  *((unsigned long *)0x20003FF0) = 0xDEADBEEF;
  NVIC_SystemReset();
}

void StartDefaultTask(void const * argument)
{
  if(HAL_GPIO_ReadPin(GPIOA,BUTTON_Pin) == 1) {
    dfu_otter_bootloader();
  }  

  MX_USB_DEVICE_Init();

  HAL_GPIO_WritePin(GPIOA,LED_POWER_Pin,1);

  tcpm_init(0);
  osDelay(50);
  pd_init(0);
  osDelay(50);

  char str[100];
  uint8_t i = 0;
  for(;;)
  {
    i++;
    osDelay(1);
    HAL_GPIO_WritePin(GPIOA,LED_STATUS_Pin,HAL_GPIO_ReadPin(GPIOA,BUTTON_Pin));
    sprintf(str,"Otter! %d %d\n\r",pd_is_connected(0),pd_is_port_enabled(0));
    CDC_Transmit_FS((unsigned char*)str,sizeof(str));
    
    //if (HAL_GPIO_ReadPin(GPIOA,INT_N_Pin) == 0) {
    //  tcpc_alert(0);
    //}

    pd_run_state_machine(0);
  }
}

void Error_Handler(void)
{

}

#ifdef  USE_FULL_ASSERT
void assert_failed(char *file, uint32_t line)
{ 
}
#endif /* USE_FULL_ASSERT */