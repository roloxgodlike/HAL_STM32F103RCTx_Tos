/**
  ******************************************************************************
  * @file  main.c
  * @author  jinyiding
  * @date    
  * @brief   
  ******************************************************************************
  */
#include "main.h"
#include "init_time.h"
#include "init_iwdg.h"
#include "init_gpio.h"
#include "init_uart.h"
#include "demo_basic.h"
#include "demo_mutex.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_IWDG_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    TIM2_init(71, 1000);
    delay_systick_init();
    
    printf("clock freq: %u\n", HAL_RCC_GetSysClockFreq());
    delay_systick_ms(1000);
    printf("start\n");
    
    tos_knl_init();
//    demo_init_basic();//ª˘¥°task≤‚ ‘
    demo_init_mutex();//ª•≥‚À¯≤‚ ‘
    tos_knl_start();
}

void Error_Handler(void)
{
    __disable_irq();
    NVIC_SystemReset();
    while (1) {}
}
