/**
  ******************************************************************************
  * @file  init_time.c
  * @author  jinyiding
  * @date    
  * @brief   
  ******************************************************************************
  */
  
/* Includes --------------------------------------------------------------------*/
#include "init_time.h"
#include "init_iwdg.h"

/* Global variables ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
static uint8_t fac_us = 0;
static uint16_t fac_ms = 0;
static TIM_HandleTypeDef h_TIM2;

/* Private define -------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ----------------------------------------------------------*/

/* Private function prototypes -------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
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

void delay_systick_init(void)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8); //选择外部时钟  HCLK/8
    fac_us = HAL_RCC_GetSysClockFreq() / 1000000 / 8;
    fac_ms=(uint16_t)(fac_us * 1000);
    printf("fac_us: %d, fac_ms: %d\n", fac_us, fac_ms);
}

void delay_systick_us(uint32_t us)
 {
	uint32_t temp;	    	 
	SysTick->LOAD = us * fac_us; 					//时间加载	  		 
	SysTick->VAL = 0x00;        					//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do {
		temp = SysTick->CTRL;
	} while ((temp&0x01) && !(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL = 0X00;      					 //清空计数器
}
 
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_systick_ms(uint16_t ms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD = (uint32_t)ms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;							//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do {
		temp = SysTick->CTRL;
	} while ((temp&0x01) && !(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL = 0X00;       					//清空计数器	  	    
}

void TIM2_init(uint32_t Prescaler, uint32_t Period)
{
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    __HAL_RCC_TIM2_CLK_ENABLE();
//    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
//    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    
    h_TIM2.Instance = TIM2;
    h_TIM2.Init.Prescaler = Prescaler;
    h_TIM2.Init.Period = Period;
    h_TIM2.Init.CounterMode = TIM_COUNTERMODE_UP;
    h_TIM2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&h_TIM2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&h_TIM2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&h_TIM2, &sMasterConfig);
    
    __HAL_TIM_CLEAR_FLAG(&h_TIM2, TIM_FLAG_UPDATE);
}

void delay_TIM2_ms(uint32_t ms)
{     
	uint32_t num = ms;
    TIM2->CNT = 0;
    __HAL_TIM_ENABLE(&h_TIM2);
    for( ; num > 0 ; num--)
    {
        #if IWDG_ENABLE == 1
            if (num % 1000 == 0)
                IWDG_Refresh();
        #endif
		/* 等待一个延时单位的结束 */
		while(!__HAL_TIM_GET_FLAG(&h_TIM2, TIM_FLAG_UPDATE));
            __HAL_TIM_CLEAR_FLAG(&h_TIM2, TIM_FLAG_UPDATE);
    }
    __HAL_TIM_DISABLE(&h_TIM2);
}
