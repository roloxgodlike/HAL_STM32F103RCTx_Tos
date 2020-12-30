/**
  ******************************************************************************
  * @file  init_iwdg.c
  * @author  jinyiding
  * @date    
  * @brief   
  ******************************************************************************
  */
  
/* Includes --------------------------------------------------------------------*/
#include "init_iwdg.h"

/* Global variables ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
static IWDG_HandleTypeDef h_IWDG;

/* Private define -------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ----------------------------------------------------------*/

/* Private function prototypes -------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void MX_IWDG_Init(void)
{
    #if IWDG_ENABLE == 1
        h_IWDG.Instance = IWDG;
        h_IWDG.Init.Prescaler = IWDG_PRESCALER_64;
        h_IWDG.Init.Reload = 0x0FFF;
        if (HAL_IWDG_Init(&h_IWDG) != HAL_OK)
        {
            Error_Handler();
        }
     #endif
}

void IWDG_Refresh(void)
{
    HAL_IWDG_Refresh(&h_IWDG);
}
