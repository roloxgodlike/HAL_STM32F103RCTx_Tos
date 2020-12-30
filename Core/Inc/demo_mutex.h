/**
  ******************************************************************************
  * @file  demo_mutex.h
  * @author  jinyiding
  * @version  0.0.1
  * @date    	
  * @brief   	
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEMO_MUTEX_H
#define __DEMO_MUTEX_H

#include "common.h"

#define STK_SIZE_TASK_WRITER    512
#define STK_SIZE_TASK_READER    512

void demo_init_mutex(void);

#endif
