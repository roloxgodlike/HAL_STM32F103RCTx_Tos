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

// һƬ�ٽ����ڴ�
static uint32_t critical_resource[3];

static void write_critical_resource(int salt)
{
    size_t i = 0;
	// �˺���ÿ�������ڴ��а�����˳��д�������޷�������
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
        // �����ٽ���д������֮ǰ���ȳ��Ի�ȡ�ٽ���������
        err = tos_mutex_pend(&critical_resource_locker);
        if (err == K_ERR_NONE) {
            // �ɹ���ȡ��֮�����ٽ���д������
            write_critical_resource(salt);
            // д�����ݺ��ͷŻ�����
            tos_mutex_post(&critical_resource_locker);
        }
        tos_task_delay(1000);
        ++salt;
    }
}

static void read_critical_resource(void)
{
    size_t i = 0;

    // ���ٽ�����ȡ����
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
        // ��ȡ�ٽ�������֮ǰ���ȳ��Ի�ȡ�ٽ���������
        err = tos_mutex_pend(&critical_resource_locker);
        if (err == K_ERR_NONE) {
            // �ɹ���ȡ��֮�󣬴��ٽ�����ȡ����
            read_critical_resource();
            // ��ȡ������Ϻ��ͷŻ�����
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
