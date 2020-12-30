/**
  ******************************************************************************
  * @file  init_time.h
  * @author  jinyiding
  * @version  0.0.1
  * @date    	
  * @brief   	
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INIT_TIME_H
#define __INIT_TIME_H

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void);

void delay_systick_init(void);
void delay_systick_us(uint32_t us);
void delay_systick_ms(uint16_t ms);

/**
  * @brief   TIM2定时器初始化
  * @note   最终定时器频率计算为： 72MHz /（Prescaler+1）/ Period
                    72MHz /（71+1）/ 1000 = 1KHz，即1ms周期
                    72MHz /（71+1）/ 10000 = 100Hz，即10ms周期
                    72MHz /（359+1）/ 4000 = 50Hz，即20ms周期
  * @param   Prescaler  a number between Min_Data = 0x0000 and Max_Data = 0xFFFF
  * @param   Period  a number between Min_Data = 0x0000 and Max_Data = 0xFFFF
  * @retval  None
  */
void TIM2_init(uint32_t Prescaler, uint32_t Period);
/**
  * @brief   TIM2定时器的延迟实现
  * @note    None
  * @param   ms 延迟毫秒
  * @retval  None
  */
void delay_TIM2_ms(uint32_t ms);

/* Internal functions --------------------------------------------------------*/

#endif
