//
// Created by lychee on 2025/11/1.
//
#include "UserTask.h"

#include "cmsis_os2.h"

uint32_t send1 = 0;
uint32_t send2 = 0;
uint32_t recv = 0;
osMessageQueueAttr_t test_queue_attributes = { .name = "test_queue" };
osMessageQueueId_t test_queue_handle;
osSemaphoreAttr_t test_semaphore_attributes = { .name = "test_semaphore" };
osSemaphoreId_t test_semaphore_handle;
osEventFlagsAttr_t test_event_flags_attributes = { .name = "test_event_flags" };
osEventFlagsId_t test_event_flags_handle;
constexpr auto flag_1 = 1u << 0;
constexpr auto flag_2 = 1u << 1;

[[noreturn]] void test_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        osEventFlagsSet(test_event_flags_handle, flag_1);
        ++send1;
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
        const auto tick = osKernelGetTickCount();
        osEventFlagsSet(test_event_flags_handle, flag_2);
        ++send2;
        osDelayUntil(tick + 7);
    }
}

osThreadId_t test1_task_handle;
constexpr osThreadAttr_t test1_task_attributes = {
    .name = "test1_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};

[[noreturn]] void test2_task(void*) {
    while (true) {
        osEventFlagsWait(test_event_flags_handle, flag_1 | flag_2,osFlagsWaitAll,osWaitForever);
        osEventFlagsClear(test_event_flags_handle, flag_1 | flag_2);
        ++recv;
    }
}

osThreadId_t test2_task_handle;
constexpr osThreadAttr_t test2_task_attributes = {
    .name = "test2_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};

void user_tasks_init() {
    test_task_handle = osThreadNew(test_task, nullptr, &test_task_attributes);
    test1_task_handle = osThreadNew(test1_task, nullptr, &test1_task_attributes);
    test2_task_handle = osThreadNew(test2_task, nullptr, &test2_task_attributes);
    test_queue_handle = osMessageQueueNew(10, sizeof(uint32_t), &test_queue_attributes);
    test_semaphore_handle = osSemaphoreNew(1, 0, &test_semaphore_attributes);
    test_event_flags_handle = osEventFlagsNew(&test_event_flags_attributes);
}
