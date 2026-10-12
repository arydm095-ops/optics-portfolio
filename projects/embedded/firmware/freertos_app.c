#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "device.h"

static QueueHandle_t sample_queue;
static SemaphoreHandle_t core_mutex;
static volatile u32 received, command_done, order_errors;

void app_assert_failed(const char *file,int line) {
    fprintf(stderr,"FreeRTOS assertion: %s:%d\n",file,line);
    exit(2);
}
static void acquire_task(void *argument) {
    (void)argument;
    TickType_t last=xTaskGetTickCount();
    for (u32 sequence=0;sequence<40;sequence++) {
        xSemaphoreTake(core_mutex,portMAX_DELAY);
        ed_clock(5); ed_acquire();
        xSemaphoreGive(core_mutex);
        configASSERT(xQueueSend(sample_queue,&sequence,pdMS_TO_TICKS(50))==pdPASS);
        vTaskDelayUntil(&last,pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}
static void control_task(void *argument) {
    (void)argument;
    u32 sequence;
    for (;;) {
        configASSERT(xQueueReceive(sample_queue,&sequence,portMAX_DELAY)==pdPASS);
        if (sequence!=received) ++order_errors;
        xSemaphoreTake(core_mutex,portMAX_DELAY);
        ed_control();
        xSemaphoreGive(core_mutex);
        ++received;
    }
}
static void feed_command(u8 command,u32 value,u32 length) {
    u8 frame[16]={0xaa,0x55,1,command,(u8)length};
    for (u32 i=0;i<length;i++) frame[5+i]=(u8)(value>>(8*i));
    u16 crc=ed_crc16(frame,5+length);
    frame[5+length]=(u8)crc; frame[6+length]=(u8)(crc>>8);
    for (u32 i=0;i<length+7;i++) ed_uart_feed(frame[i]);
    ed_comm();
}
static void communication_task(void *argument) {
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(10));
    xSemaphoreTake(core_mutex,portMAX_DELAY);
    feed_command(CMD_TARGET,1300,2); feed_command(CMD_RUN,1,1);
    xSemaphoreGive(core_mutex);
    command_done=1;
    vTaskDelete(NULL);
}
static void health_task(void *argument) {
    (void)argument;
    while (received<40 || !command_done) vTaskDelay(pdMS_TO_TICKS(10));
    xSemaphoreTake(core_mutex,portMAX_DELAY);
    ed_health();
    configASSERT(order_errors==0);
    configASSERT(ed_get(4)==1300 && ed_get(1)==DEVICE_RUN);
    configASSERT(ed_get(27)==40 && ed_get(28)==40);
    configASSERT(xTaskGetTickCount()>=pdMS_TO_TICKS(180));
    xSemaphoreGive(core_mutex);
    puts("PASS\tfreertos_queue_preserves_sample_order");
    puts("PASS\tfreertos_blocked_control_task_wakes");
    puts("PASS\tfreertos_mutex_protects_shared_core");
    puts("PASS\tfreertos_communication_task_executes_commands");
    puts("PASS\tfreertos_four_tasks_execute_on_official_kernel");
    printf("KERNEL\t%s\n",tskKERNEL_VERSION_NUMBER);
    printf("SAMPLES\t%u\n",received);
    fflush(stdout);
    vTaskEndScheduler();
    for (;;) { }
}
int main(void) {
    ed_init();
    sample_queue=xQueueCreate(8,sizeof(u32));
    core_mutex=xSemaphoreCreateMutex();
    configASSERT(sample_queue && core_mutex);
    configASSERT(xTaskCreate(acquire_task,"Acquisition",512,NULL,4,NULL)==pdPASS);
    configASSERT(xTaskCreate(control_task,"Control",512,NULL,3,NULL)==pdPASS);
    configASSERT(xTaskCreate(communication_task,"Communication",512,NULL,2,NULL)==pdPASS);
    configASSERT(xTaskCreate(health_task,"Health",512,NULL,1,NULL)==pdPASS);
    vTaskStartScheduler();
    return 0;
}
