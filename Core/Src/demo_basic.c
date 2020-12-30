/**
  ******************************************************************************
  * @file  demo_basic..c
  * @author  jinyiding
  * @date    
  * @brief   
  ******************************************************************************
  */
  
/* Includes --------------------------------------------------------------------*/
#include "demo_basic.h"
#include "biz_includes.h"

k_stack_t stack_task1[TASK1_STK_SIZE];
k_stack_t stack_task2[TASK2_STK_SIZE];

k_task_t task1;
k_task_t task2;

void entry_task1(void *pdata)
{
    int count = 0;
    while (1)
    {
        printf("TASK1-lock\n");
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        tos_knl_sched_lock();
        for (char i = 0 ; i < 10; i++)
        {
            printf("TASK1-locking-%d\n", count++);
            delay_TIM2_ms(200);
        }
        tos_knl_sched_unlock();
        printf("TASK1-unlock\n");
        count = 0;
        tos_sleep_ms(3000);
    }
}

void entry_task2(void *pdata)
{
    while (1)
    {
        printf("TASK2-feed\n");
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        IWDG_Refresh();
        tos_sleep_ms(1000);
    }
}

void demo_init_basic(void)
{
    tos_task_create(&task1, "task1", entry_task1, NULL,
                            4, stack_task1, TASK1_STK_SIZE, 0);
    tos_task_create(&task2, "task2", entry_task2, NULL,
                            4, stack_task2, TASK2_STK_SIZE, 0);
}
