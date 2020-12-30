/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "main.h"
#include "stdio.h"
#include "cmsis_os.h"

IWDG_HandleTypeDef hiwdg;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
static uint8_t fac_us = 0;
static uint16_t fac_ms = 0;

//task1
#define TASK1_STK_SIZE		256
void task1(void *pdata);
osThreadDef(task1, osPriorityNormal, 1, TASK1_STK_SIZE);
//task2
#define TASK2_STK_SIZE		256
void task2(void *pdata);
osThreadDef(task2, osPriorityNormal, 1, TASK2_STK_SIZE);

 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);

/* Private user code ---------------------------------------------------------*/
/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart5, &ch, 1, 0xffff);
  return ch;
}

void Delay_init(void)
{
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8); //选择外部时钟  HCLK/8
  fac_us = HAL_RCC_GetSysClockFreq() / 1000000 / 8;
  fac_ms=(uint16_t)(fac_us * 1000);
}

void Delay_us(uint32_t nus)
 {
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//时间加载	  		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 //清空计数器
}

//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void Delay_ms(uint16_t ms){	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)ms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       					//清空计数器	  	    
}

TIM_HandleTypeDef hTIM2;

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
void TIM2_Init(uint32_t Prescaler, uint32_t Period)
{
    /* 基本定时器外设时钟使能 */
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    __HAL_RCC_TIM2_CLK_ENABLE();
//    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
//    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
      
    
    hTIM2.Instance = TIM2;
    hTIM2.Init.Prescaler = Prescaler;
    hTIM2.Init.Period = Period;
    hTIM2.Init.CounterMode = TIM_COUNTERMODE_UP;
    hTIM2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&hTIM2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&hTIM2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&hTIM2, &sMasterConfig);
    
    __HAL_TIM_CLEAR_FLAG(&hTIM2, TIM_FLAG_UPDATE);
}

/**
  * @brief   TIM2定时器的延迟实现
  * @note    None
  * @param   ms 延迟毫秒
  * @retval  None
  */
void Delay_TIM_ms(uint32_t ms)
{     
    /* 清零计数器并使能滴答定时器 */ 
	uint32_t num = ms;
    TIM2->CNT = 0;
    __HAL_TIM_ENABLE(&hTIM2);
    for( ; num > 0 ; num--)
    {
		if (num % 1000 == 0)
            HAL_IWDG_Refresh(&hiwdg);
        
		/* 等待一个延时单位的结束 */
		while(!__HAL_TIM_GET_FLAG(&hTIM2, TIM_FLAG_UPDATE));
            __HAL_TIM_CLEAR_FLAG(&hTIM2, TIM_FLAG_UPDATE);
    }
    __HAL_TIM_DISABLE(&hTIM2);
}

void task1(void *pdata)
{
     int count = 0;
     while(1)
     {
       printf("TASK1-lock\n");
       HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
       tos_knl_sched_lock();
       for (int i = 0; i < 10; i++)
       {
            printf("TASK1-locking-%d\n", count++);
            Delay_TIM_ms(200);
       }
       tos_knl_sched_unlock();
       printf("TASK1-unlock\n");
       count = 0;
       osDelay(1000);
     }
}
void task2(void *pdata)
{
     while(1)
     {
       printf("TASK2-feed\n");
       HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
       HAL_IWDG_Refresh(&hiwdg);
       osDelay(500);
     }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
    
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  TIM2_Init(71, 1000);
  
  printf("clock freq: %u\n", HAL_RCC_GetSysClockFreq());
  Delay_init();
  printf("fac_us: %d, fac_ms: %d\n", fac_us, fac_ms);
  Delay_ms(1000);
  printf("start\n");
    
//  while (1)
//  {
//    Delay_TIM_ms(1000);
//    HAL_IWDG_Refresh(&hiwdg);
//    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9);
//  }

  osKernelInitialize(); //TOS Tiny kernel initialize
  osThreadCreate(osThread(task1), NULL);// Create task1
  osThreadCreate(osThread(task2), NULL);// Create task2
  osKernelStart();//Start TOS Tiny

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 0x0FFF;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 921600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_Initure;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
  
   GPIO_Initure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
   GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_Initure.Pull = GPIO_PULLUP;
   GPIO_Initure.Speed = GPIO_SPEED_HIGH;
   HAL_GPIO_Init(GPIOC, &GPIO_Initure);
  
}

/* USER CODE BEGIN 4 */
    
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
