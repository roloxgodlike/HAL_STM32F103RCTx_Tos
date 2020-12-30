/**
  ******************************************************************************
  * @file  init_iwdg.h
  * @author  jinyiding
  * @version  0.0.1
  * @date    	
  * @brief   	
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INIT_IWDG_H
#define __INIT_IWDG_H

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
void MX_IWDG_Init(void);

void IWDG_Refresh(void);

/* Internal functions --------------------------------------------------------*/

#endif
