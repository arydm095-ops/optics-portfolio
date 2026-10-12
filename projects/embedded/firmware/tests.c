#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "device.h"

static unsigned total;
static void check(const char *name, int passed) {
    printf("%s\t%s\n",passed?"PASS":"FAIL",name);
    ++total;
    if (!passed) exit(1);
}
static u32 encode(u8 *frame,u8 command,u32 value,u32 length) {
    frame[0]=0xaa; frame[1]=0x55; frame[2]=7; frame[3]=command; frame[4]=(u8)length;
    for (u32 i=0;i<length;i++) frame[5+i]=(u8)(value>>(8*i));
    u16 crc=ed_crc16(frame,5+length);
    frame[5+length]=(u8)crc; frame[6+length]=(u8)(crc>>8); return length+7;
}
static void command(u8 cmd,u32 value,u32 length) {
    u8 frame[23]; u32 count=encode(frame,cmd,value,length);
    for (u32 i=0;i<count;i++) ed_uart_feed(frame[i]);
    ed_comm();
}
static u32 stage(u32 version,u32 corrupt) {
    u8 *bytes=ed_buffer();
    for (u32 i=0;i<8192;i++) bytes[i]=(u8)(i*31+7);
    u32 crc=ed_crc32(bytes,8192);
    if (corrupt) bytes[37]^=1;
    return ed_stage_image(8192,version,crc);
}
int main(int argc,char **argv) {
    if (argc>1 && strcmp(argv[1],"--snapshot")==0) {
        ed_init(); command(CMD_TARGET,1200,2); command(CMD_RUN,1,1); ed_step(2000);
        ed_action(1,1); ed_step(20);
        for (u32 i=0;i<39;i++) printf("%.9f%s",ed_get(i),i==38?"\n":",");
        return 0;
    }
    const u8 vector[]="123456789";
    check("crc16_modbus_known_vector",ed_crc16(vector,9)==0x4b37);
    check("crc32_known_vector",ed_crc32(vector,9)==0xcbf43926u);
    ed_init();
    check("boot_selects_verified_slot",ed_get(20)==100 && ed_get(33)==1);
    command(CMD_TARGET,1200,2);
    check("valid_uart_frame_changes_target",ed_get(4)==1200 && ed_tx_get(5)==COMMAND_OK);
    u8 frame[23]; u32 length=encode(frame,CMD_TARGET,600,2);
    frame[length-1]^=1;
    for(u32 i=0;i<length;i++) ed_uart_feed(frame[i]);
    ed_comm();
    check("bad_crc_rejects_command",ed_get(4)==1200 && ed_get(12)==1);
    length=encode(frame,CMD_TARGET,900,2);
    for(u32 i=0;i<4;i++) ed_uart_feed(frame[i]);
    ed_comm();
    check("partial_frame_waits_for_tail",ed_get(4)==1200);
    for(u32 i=4;i<length;i++) ed_uart_feed(frame[i]);
    ed_comm(); check("split_frame_reassembles",ed_get(4)==900);
    command(CMD_TARGET,2401,2); check("target_range_checked",ed_get(4)==900 && ed_tx_get(5)==COMMAND_RANGE);
    command(CMD_SAMPLE,0,2); check("sampling_range_checked",ed_get(15)==5 && ed_tx_get(5)==COMMAND_RANGE);
    ed_init(); for(u32 i=0;i<80;i++) ed_uart_feed(0);
    check("uart_ring_overflow_bounded",ed_get(26)==64 && ed_get(13)==16);
    ed_comm(); command(CMD_TARGET,1000,2);
    check("parser_resynchronises_after_noise",ed_get(4)==1000);
    ed_init(); ed_step(80);
    check("dma_double_buffer_switches",ed_get(10)==1 && ed_get(34)==1 && ed_get(35)==0);
    check("dma_block_mean_consumed",ed_get(17)>500 && ed_get(17)<2800 && ed_get(9)==0);
    ed_init(); ed_action(3,0);
    check("sample_queue_overflow_bounded",ed_get(9)==8 && ed_get(36)==3);
    ed_control(); check("sample_queue_drained",ed_get(9)==0);
    ed_init(); command(CMD_RUN,1,1); ed_step(3000);
    check("controller_tracks_reference",ed_get(3)>790 && ed_get(3)<810);
    check("controller_output_limited",ed_get(5)>=0 && ed_get(5)<=100);
    ed_action(1,1); ed_step(20);
    check("overcurrent_latches_and_disables_drive",ed_get(1)==DEVICE_FAULT && ((u32)ed_get(14)&FAULT_CURRENT) && ed_get(5)==0);
    command(CMD_ACK,0,0); check("reset_denied_while_fault_present",ed_get(1)==DEVICE_FAULT && ed_tx_get(5)==COMMAND_UNSAFE);
    ed_action(1,0); ed_step(20); command(CMD_ACK,0,0);
    check("explicit_ack_restores_stopped_state",ed_get(1)==DEVICE_STOP && ed_get(14)==0);
    ed_init(); command(CMD_RUN,1,1); ed_action(1,2); ed_step(20);
    check("undervoltage_protection",((u32)ed_get(14)&FAULT_VOLTAGE)!=0);
    ed_init(); command(CMD_RUN,1,1); ed_action(1,4); ed_step(20);
    check("encoder_failure_protection",((u32)ed_get(14)&FAULT_ENCODER)!=0);
    ed_init(); command(CMD_RUN,1,1); ed_action(1,8); ed_step(200);
    check("watchdog_detects_missing_control_task",((u32)ed_get(14)&FAULT_WATCHDOG)!=0);
    ed_init(); command(CMD_TARGET,1370,2); command(CMD_SAMPLE,2,2); command(CMD_SAVE,0,0); ed_power_cycle();
    check("parameters_survive_power_cycle",ed_get(4)==1370 && ed_get(15)==2 && ed_get(19)==1);
    ed_test_corrupt_nv(); ed_power_cycle();
    check("corrupt_parameters_fall_back_to_defaults",ed_get(4)==800 && ed_get(15)==5 && ed_get(19)==0);
    ed_init(); check("valid_image_is_staged",stage(101,0)==COMMAND_OK);
    ed_step(2600); check("new_slot_crc_verified_before_boot",ed_get(22)==UPDATE_READY && ed_get(20)==100);
    ed_power_cycle(); check("new_firmware_enters_trial_boot",ed_get(20)==101 && ed_get(22)==UPDATE_TRIAL);
    ed_step(300); check("healthy_firmware_confirmed",ed_get(22)==UPDATE_CONFIRMED);
    ed_power_cycle(); check("confirmed_firmware_survives_restart",ed_get(20)==101);
    ed_init(); stage(101,1); ed_step(2600);
    check("corrupt_image_keeps_old_firmware",ed_get(22)==UPDATE_BAD_CRC && ed_get(20)==100);
    ed_init(); stage(101,0); ed_step(200); ed_power_cycle();
    check("interrupted_programming_keeps_old_slot",ed_get(20)==100 && ed_get(22)==UPDATE_INTERRUPTED);
    ed_init(); stage(101,0); ed_step(2600); ed_power_cycle(); ed_power_cycle();
    check("unconfirmed_trial_rolls_back",ed_get(20)==100);
    ed_init(); stage(101,0); ed_step(2600); ed_power_cycle(); ed_step(300);
    check("firmware_downgrade_rejected",stage(100,0)==COMMAND_RANGE);
    u32 bytes=ed_export_nv(); u8 *storage=malloc(bytes);
    memcpy(storage,ed_buffer(),bytes); ed_init(); memcpy(ed_buffer(),storage,bytes);
    check("storage_image_restores_verified_firmware",ed_import_nv(bytes)==1 && ed_get(20)==101);
    memcpy(ed_buffer(),storage,bytes); ed_buffer()[100]^=1;
    check("storage_checksum_rejects_corruption",ed_import_nv(bytes)==0);
    free(storage);
    printf("TOTAL\t%u\n",total);
    return 0;
}
