/**
  ******************************************************************************
  * @file  init_gpio.c
  * @author  jinyiding
  * @date    
  * @brief   
  ******************************************************************************
  */
  
/* Includes --------------------------------------------------------------------*/
#include "init_gpio.h"

/* Global variables ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define -------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ----------------------------------------------------------*/

/* Private function prototypes -------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void MX_GPIO_Init(void)
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
