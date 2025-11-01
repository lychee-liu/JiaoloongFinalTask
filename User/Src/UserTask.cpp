//
// Created by lychee on 2025/11/1.
//
#include "UserTask.h"

#include "cmsis_os2.h"

uint32_t send = 0;
uint32_t recv = 0;
osMessageQueueAttr_t test_queue_attributes = { .name = "test_queue" };
osMessageQueueId_t test_queue_handle;

[[noreturn]] void test_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        ++send;
        osMessageQueuePut(test_queue_handle, &send, 0, 0);
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
        osMessageQueueGet(test_queue_handle, &recv, nullptr, osWaitForever);
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
}
