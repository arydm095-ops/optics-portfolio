#ifndef EMBEDDED_DEVICE_H
#define EMBEDDED_DEVICE_H

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

enum DeviceMode { DEVICE_STOP, DEVICE_RUN, DEVICE_FAULT, DEVICE_UPDATE };
enum FaultCode { FAULT_CURRENT=1, FAULT_VOLTAGE=2, FAULT_ENCODER=4, FAULT_WATCHDOG=8 };
enum TaskId { TASK_ACQUIRE=1, TASK_CONTROL, TASK_COMM, TASK_HEALTH };
enum EventCode { EV_START=1, EV_STOP, EV_COMMAND, EV_CRC, EV_OVERFLOW, EV_DMA,
    EV_FAULT, EV_RESET, EV_SAVE, EV_LOAD, EV_UPDATE, EV_READY, EV_BOOT,
    EV_CONFIRM, EV_ROLLBACK, EV_REJECT, EV_BAD_IMAGE };
enum UpgradeState { UPDATE_IDLE, UPDATE_WRITING, UPDATE_READY, UPDATE_TRIAL,
    UPDATE_CONFIRMED, UPDATE_BAD_CRC, UPDATE_INTERRUPTED };
enum CommandCode { CMD_RUN=1, CMD_TARGET=2, CMD_SAMPLE=3, CMD_SAVE=16, CMD_ACK=17 };
enum CommandStatus { COMMAND_OK, COMMAND_CRC, COMMAND_RANGE, COMMAND_BUSY,
    COMMAND_UNKNOWN, COMMAND_UNSAFE };

/* The byte protocol and storage image use explicit little-endian encoding. */
void ed_init(void);
void ed_clock(u32 elapsed_ms);
void ed_acquire(void);
void ed_control(void);
void ed_comm(void);
void ed_health(void);
void ed_step(u32 elapsed_ms);
void ed_uart_feed(u32 byte);
void ed_action(u32 action, u32 value);
void ed_power_cycle(void);
double ed_get(u32 field);
u32 ed_event_get(u32 row, u32 field);
u32 ed_task_get(u32 row, u32 field);
u8 *ed_buffer(void);
u32 ed_tx_get(u32 byte);
u32 ed_dma_get(u32 bank, u32 index);
u16 ed_crc16(const u8 *bytes, u32 length);
u32 ed_crc32(const u8 *bytes, u32 length);
u32 ed_stage_image(u32 length, u32 version, u32 expected_crc);
u32 ed_export_nv(void);
u32 ed_import_nv(u32 length);
void ed_test_corrupt_nv(void);

#define ED_IMAGE_MAX 16384u
#define ED_BUFFER_SIZE 65536u
#define ED_EVENT_CAPACITY 128u
#define ED_TASK_CAPACITY 256u
#endif
