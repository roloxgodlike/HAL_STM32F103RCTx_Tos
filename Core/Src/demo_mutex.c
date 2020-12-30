/**
  ******************************************************************************
  * @file  demo_mutex.c
  * @author  jinyiding
  * @date    
  * @brief   
  ******************************************************************************
  */
  
/* Includes --------------------------------------------------------------------*/
#include "demo_mutex.h"

k_stack_t stack_task_writer[STK_SIZE_TASK_WRITER];
k_stack_t stack_task_reader[STK_SIZE_TASK_READER];

k_task_t task_writer;
k_task_t task_reader;

k_mutex_t critical_resource_locker;

// 一片临界区内存
static uint32_t critical_resource[3];

static void write_critical_resource(int salt)
{
    size_t i = 0;
	// 此函数每次向共享内存中按递增顺序写入三个无符号整数
    printf("writting critical resource:\n");
    for (i = 0; i < 3; ++i) {
        printf("%d\t", salt + i);
        critical_resource[i] = salt + i;
    }
    printf("\n");
}

void entry_task_writer(void *arg)
{
    size_t salt = 0;
    k_err_t err;

    while (K_TRUE) {
        // 在向临界区写入数据之前，先尝试获取临界区保护锁
        err = tos_mutex_pend(&critical_resource_locker);
        if (err == K_ERR_NONE) {
            // 成功获取锁之后，向临界区写入数据
            write_critical_resource(salt);
            // 写完数据后，释放互斥锁
            tos_mutex_post(&critical_resource_locker);
        }
        tos_task_delay(1000);
        ++salt;
    }
}

static void read_critical_resource(void)
{
    size_t i = 0;

    // 从临界区读取数据
    printf("reading critical resource:\n");
    for (i = 0; i < 3; ++i) {
        printf("%d\t", critical_resource[i]);
    }
    printf("\n");
}

void entry_task_reader(void *arg)
{
    k_err_t err;

    while (K_TRUE) {
        // 读取临界区数据之前，先尝试获取临界区保护锁
        err = tos_mutex_pend(&critical_resource_locker);
        if (err == K_ERR_NONE) {
            // 成功获取锁之后，从临界区读取数据
            read_critical_resource();
            // 读取数据完毕后，释放互斥锁
            tos_mutex_post(&critical_resource_locker);
        }
        tos_task_delay(1000);
    }
}

void demo_init_mutex(void)
{
    tos_mutex_create(&critical_resource_locker);
    (void)tos_task_create(&task_writer, "writer", entry_task_writer, NULL,
                            4, stack_task_writer, STK_SIZE_TASK_WRITER, 0);
    (void)tos_task_create(&task_reader, "reader", entry_task_reader, NULL,
                            4, stack_task_reader, STK_SIZE_TASK_READER, 0);
}
