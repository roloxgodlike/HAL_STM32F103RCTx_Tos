/**
  ******************************************************************************
  * @file  init_uart.c
  * @author  jinyiding
  * @date    
  * @brief   
  ******************************************************************************
  */
  
/* Includes --------------------------------------------------------------------*/
#include "init_uart.h"

/* Global variables ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
static UART_HandleTypeDef h_UART4;
static UART_HandleTypeDef h_UART5;

/* Private define -------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ----------------------------------------------------------*/

/* Private function prototypes -------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&h_UART5, (uint8_t *)&ch, 1, 0xffff);
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
    HAL_UART_Receive(&h_UART5, &ch, 1, 0xffff);
    return ch;
}

void MX_UART4_Init(void)
{
    h_UART4.Instance = UART4;
    h_UART4.Init.BaudRate = 115200;
    h_UART4.Init.WordLength = UART_WORDLENGTH_8B;
    h_UART4.Init.StopBits = UART_STOPBITS_1;
    h_UART4.Init.Parity = UART_PARITY_NONE;
    h_UART4.Init.Mode = UART_MODE_TX_RX;
    h_UART4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h_UART4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&h_UART4) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_UART5_Init(void)
{
    h_UART5.Instance = UART5;
    h_UART5.Init.BaudRate = 921600;
    h_UART5.Init.WordLength = UART_WORDLENGTH_8B;
    h_UART5.Init.StopBits = UART_STOPBITS_1;
    h_UART5.Init.Parity = UART_PARITY_NONE;
    h_UART5.Init.Mode = UART_MODE_TX_RX;
    h_UART5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h_UART5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&h_UART5) != HAL_OK)
    {
        Error_Handler();
    }
}

