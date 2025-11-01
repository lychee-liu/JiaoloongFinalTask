//
// Created by lychee on 2025/11/1.
//
#include "UserTask.h"

#include "cmsis_os2.h"

uint32_t send = 0;
uint32_t recv = 0;
osMessageQueueAttr_t test_queue_attributes = { .name = "test_queue" };
osMessageQueueId_t test_queue_handle;
osSemaphoreAttr_t test_semaphore_attributes = { .name = "test_semaphore" };
osSemaphoreId_t test_semaphore_handle;

[[noreturn]] void test_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        if (send++ % 5 == 0) {
            osSemaphoreRelease(test_semaphore_handle);
        }
        osDelayUntil(tick + 1);
    }
}

osThreadId_t test_task_handle;
constexpr osThreadAttr_t test_task_attributes = {
    .name = "test_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};

[[noreturn]] void test1_task(void*) {
    while (true) {
        osSemaphoreAcquire(test_semaphore_handle,osWaitForever);
        recv++;
    }
}

osThreadId_t test1_task_handle;
constexpr osThreadAttr_t test1_task_attributes = {
    .name = "test1_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};

void user_tasks_init() {
    test_task_handle = osThreadNew(test_task, nullptr, &test_task_attributes);
    test1_task_handle = osThreadNew(test1_task, nullptr, &test1_task_attributes);
    test_queue_handle = osMessageQueueNew(10, sizeof(uint32_t), &test_queue_attributes);
    test_semaphore_handle = osSemaphoreNew(1, 0, &test_semaphore_attributes);
}
