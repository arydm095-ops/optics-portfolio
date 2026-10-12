#include "device.h"

#define RX_CAPACITY 64u
#define QUEUE_CAPACITY 8u
#define DMA_SIZE 16u
#define STORAGE_SIZE (40u + 2u*(16u + ED_IMAGE_MAX) + 4u)

typedef struct { u32 time, code, value; } Event;
typedef struct { u32 time, id; } TaskRecord;
typedef struct { u32 target, sample_ms, gain; } Parameters;
typedef struct { u8 bytes[ED_IMAGE_MAX]; u32 version, length, crc, valid; } FlashSlot;
typedef struct {
    u32 ms, mode, faults, injections, blocks, frames, crc_errors, rx_overflow;
    u32 queue_drops, rx_head, rx_tail, rx_count, queue_head, queue_tail, queue_count;
    u32 dma_position, dma_bank, task_counts[4], last_feed, last_cmd, tx_length;
    u32 adc_mv, mean_mv, current_ma, bus_mv, temperature, events, task_records;
    u32 update_state, update_length, update_version, update_crc, written;
    u32 update_slot, trial_started, saved, nv_valid;
    double speed, duty, integral;
    Parameters params;
    u8 rx[RX_CAPACITY], tx[32];
    u16 dma[2][DMA_SIZE];
    u32 queue[QUEUE_CAPACITY];
    Event event[ED_EVENT_CAPACITY];
    TaskRecord task[ED_TASK_CAPACITY];
} Device;

static Device d;
static FlashSlot flash[2];
static u32 active_slot, confirmed_slot, pending_slot, candidate_pending;
static Parameters saved_params;
static u32 saved_magic, saved_crc;
static u8 workspace[ED_BUFFER_SIZE], image[ED_IMAGE_MAX];

static void fill_bytes(u8 *p, u8 value, u32 length) {
    for (u32 i=0; i<length; ++i) p[i]=value;
}
static u32 read32(const u8 *p) {
    return (u32)p[0] | ((u32)p[1]<<8) | ((u32)p[2]<<16) | ((u32)p[3]<<24);
}
static void write32(u8 *p, u32 value) {
    for (u32 i=0; i<4; ++i) p[i]=(u8)(value>>(8*i));
}
static double absolute(double value) { return value < 0 ? -value : value; }
static double clamp(double value, double low, double high) {
    return value < low ? low : value > high ? high : value;
}
u16 ed_crc16(const u8 *bytes, u32 length) {
    u16 crc=0xffff;
    for (u32 i=0; i<length; ++i) {
        crc ^= bytes[i];
        for (u32 bit=0; bit<8; ++bit) crc=(u16)((crc>>1)^((crc&1)?0xa001:0));
    }
    return crc;
}
u32 ed_crc32(const u8 *bytes, u32 length) {
    u32 crc=0xffffffffu;
    for (u32 i=0; i<length; ++i) {
        crc ^= bytes[i];
        for (u32 bit=0; bit<8; ++bit) crc=(crc>>1)^((crc&1)?0xedb88320u:0);
    }
    return ~crc;
}
static void event(u32 code, u32 value) {
    d.event[d.events % ED_EVENT_CAPACITY]=(Event){d.ms, code, value};
    ++d.events;
}
static void task(u32 id) {
    ++d.task_counts[id-1];
    d.task[d.task_records % ED_TASK_CAPACITY]=(TaskRecord){d.ms,id};
    ++d.task_records;
}
static u32 params_crc(Parameters p) {
    u8 encoded[12];
    write32(encoded,p.target); write32(encoded+4,p.sample_ms); write32(encoded+8,p.gain);
    return ed_crc32(encoded,12);
}
static u32 slot_valid(u32 slot) {
    return flash[slot].valid && flash[slot].length > 0 &&
        flash[slot].length <= ED_IMAGE_MAX &&
        ed_crc32(flash[slot].bytes,flash[slot].length)==flash[slot].crc;
}
static void reset_volatile(void) {
    fill_bytes((u8*)&d,0,(u32)sizeof(d));
    d.params=(Parameters){800,5,250};
    d.bus_mv=24000; d.temperature=2600; d.last_feed=0;
    if (saved_magic==0x50415241 && params_crc(saved_params)==saved_crc &&
        saved_params.target<=2400 && saved_params.sample_ms>=1 && saved_params.sample_ms<=20) {
        d.params=saved_params; d.nv_valid=1; event(EV_LOAD,d.params.target);
    } else if (saved_magic) event(EV_REJECT,100);
}
void ed_init(void) {
    fill_bytes((u8*)flash,0,(u32)sizeof(flash));
    flash[0].bytes[0]=0x10; flash[0].bytes[1]=0x20;
    flash[0].length=2; flash[0].version=100;
    flash[0].crc=ed_crc32(flash[0].bytes,2); flash[0].valid=1;
    active_slot=confirmed_slot=pending_slot=candidate_pending=0;
    saved_magic=saved_crc=0;
    reset_volatile();
    event(EV_BOOT,100);
}
static u32 hazards(void) {
    u32 result=0;
    if (d.current_ma>3000 || (d.injections&1)) result|=FAULT_CURRENT;
    if (d.bus_mv<18000 || (d.injections&2)) result|=FAULT_VOLTAGE;
    if (d.injections&4) result|=FAULT_ENCODER;
    if (d.mode==DEVICE_RUN && d.ms-d.last_feed>120) result|=FAULT_WATCHDOG;
    return result;
}
static void trip(u32 mask) {
    u32 added=mask & ~d.faults;
    d.faults |= mask;
    if (mask) { d.mode=DEVICE_FAULT; d.duty=0; }
    if (added) event(EV_FAULT,added);
}
void ed_clock(u32 elapsed_ms) {
    if (elapsed_ms>10000) elapsed_ms=10000;
    for (u32 i=0; i<elapsed_ms; ++i) {
        ++d.ms;
        double requested=(d.mode==DEVICE_RUN?d.duty*2.5:0);
        d.speed += (requested-d.speed)*(0.001/0.08);
        if (d.speed<0.001) d.speed=0;
        d.current_ma=(u32)(d.mode==DEVICE_RUN?200+absolute(requested-d.speed)*0.65:0);
        if (d.injections&1) d.current_ma=5000;
        d.bus_mv=(d.injections&2)?8000:24000;
        d.temperature=2600+(u32)(d.current_ma*0.2);
    }
}
void ed_acquire(void) {
    task(TASK_ACQUIRE);
    u32 phase=d.ms%200, triangle=phase<100?phase:200-phase;
    d.adc_mv=500+triangle*23;
    d.dma[d.dma_bank][d.dma_position++]=(u16)(d.adc_mv*4095/3300);
    if (d.dma_position < DMA_SIZE) return;
    u32 sum=0;
    for (u32 i=0; i<DMA_SIZE; ++i) sum+=d.dma[d.dma_bank][i];
    u32 mean=sum*3300/(DMA_SIZE*4095);
    ++d.blocks;
    if (d.queue_count<QUEUE_CAPACITY) {
        d.queue[d.queue_head]=mean;
        d.queue_head=(d.queue_head+1)%QUEUE_CAPACITY;
        ++d.queue_count;
    } else { ++d.queue_drops; event(EV_OVERFLOW,2); }
    d.dma_bank ^= 1; d.dma_position=0;
    event(EV_DMA,mean);
}
void ed_control(void) {
    if (d.injections&8) return;
    task(TASK_CONTROL); d.last_feed=d.ms;
    while (d.queue_count) {
        d.mean_mv=d.queue[d.queue_tail];
        d.queue_tail=(d.queue_tail+1)%QUEUE_CAPACITY; --d.queue_count;
    }
    trip(hazards());
    if (d.mode!=DEVICE_RUN) { d.duty=0; d.integral=0; return; }
    double error=(double)d.params.target-d.speed;
    double candidate=d.integral+error*0.02;
    double raw=error*(double)d.params.gain/1000+candidate;
    /* Conditional integration prevents accumulation beyond the actuator limit. */
    if (!((raw>1000 && error>0)||(raw<0 && error<0))) d.integral=candidate;
    d.duty=clamp(error*(double)d.params.gain/1000+d.integral,0,1000);
}
static u8 rx_peek(u32 index) { return d.rx[(d.rx_tail+index)%RX_CAPACITY]; }
static void rx_drop(u32 length) {
    d.rx_tail=(d.rx_tail+length)%RX_CAPACITY; d.rx_count-=length;
}
void ed_uart_feed(u32 byte) {
    if (d.rx_count==RX_CAPACITY) { ++d.rx_overflow; event(EV_OVERFLOW,1); return; }
    d.rx[d.rx_head]=(u8)byte; d.rx_head=(d.rx_head+1)%RX_CAPACITY; ++d.rx_count;
}
static u32 execute(u32 command, const u8 *payload, u32 length) {
    if (command==CMD_RUN && length==1) {
        if (payload[0]>1) return COMMAND_RANGE;
        if (payload[0] && (d.faults || hazards() || d.update_state==UPDATE_WRITING)) return COMMAND_UNSAFE;
        d.mode=payload[0]?DEVICE_RUN:DEVICE_STOP; d.integral=0;
        d.last_feed=d.ms; if (!payload[0]) d.duty=0;
        event(payload[0]?EV_START:EV_STOP,d.params.target); return COMMAND_OK;
    }
    if ((command==CMD_TARGET || command==CMD_SAMPLE) && length==2) {
        u32 value=(u32)payload[0]|((u32)payload[1]<<8);
        if (command==CMD_TARGET && value<=2400) d.params.target=value;
        else if (command==CMD_SAMPLE && value>=1 && value<=20) d.params.sample_ms=value;
        else return COMMAND_RANGE;
        return COMMAND_OK;
    }
    if (command==CMD_SAVE && length==0) {
        saved_params=d.params; saved_crc=params_crc(saved_params); saved_magic=0x50415241;
        ++d.saved; d.nv_valid=1; event(EV_SAVE,d.params.target); return COMMAND_OK;
    }
    if (command==CMD_ACK && length==0) {
        if (hazards()) return COMMAND_UNSAFE;
        d.faults=0; d.mode=DEVICE_STOP; d.integral=0; d.duty=0;
        event(EV_RESET,0); return COMMAND_OK;
    }
    return COMMAND_UNKNOWN;
}
static void reply(u8 sequence, u8 command, u8 code) {
    d.tx[0]=0xaa; d.tx[1]=0x55; d.tx[2]=sequence;
    d.tx[3]=command|0x80; d.tx[4]=1; d.tx[5]=code;
    u16 crc=ed_crc16(d.tx,6); d.tx[6]=(u8)crc; d.tx[7]=(u8)(crc>>8); d.tx_length=8;
}
void ed_comm(void) {
    task(TASK_COMM);
    while (d.rx_count>=7) {
        if (rx_peek(0)!=0xaa || rx_peek(1)!=0x55) { rx_drop(1); continue; }
        u32 payload_length=rx_peek(4);
        if (payload_length>16) { rx_drop(1); ++d.crc_errors; event(EV_REJECT,101); continue; }
        u32 length=payload_length+7;
        if (d.rx_count<length) break;
        u8 bytes[23]; for (u32 i=0; i<length; ++i) bytes[i]=rx_peek(i);
        u16 received=(u16)(bytes[length-2]|((u16)bytes[length-1]<<8));
        if (ed_crc16(bytes,length-2)!=received) {
            ++d.crc_errors; event(EV_CRC,bytes[2]); reply(bytes[2],bytes[3],COMMAND_CRC);
            /* Drop one byte, then resynchronise rather than trusting a bad frame. */
            rx_drop(1); continue;
        }
        rx_drop(length); ++d.frames; d.last_cmd=bytes[3];
        u32 result=execute(bytes[3],bytes+5,payload_length);
        reply(bytes[2],bytes[3],(u8)result); event(result?EV_REJECT:EV_COMMAND,bytes[3]);
    }
}
void ed_health(void) {
    task(TASK_HEALTH);
    trip(hazards());
    if (d.update_state==UPDATE_TRIAL && d.ms-d.trial_started>=300 && !d.faults) {
        confirmed_slot=active_slot; candidate_pending=0;
        d.update_state=UPDATE_CONFIRMED; event(EV_CONFIRM,flash[active_slot].version);
    }
}
static void program_chunk(void) {
    if (d.update_state!=UPDATE_WRITING) return;
    u32 remaining=d.update_length-d.written, amount=remaining<64?remaining:64;
    for (u32 i=0; i<amount; ++i) flash[d.update_slot].bytes[d.written+i]&=image[d.written+i];
    d.written+=amount;
    if (d.written<d.update_length) return;
    FlashSlot *slot=&flash[d.update_slot];
    slot->length=d.update_length; slot->version=d.update_version; slot->crc=d.update_crc;
    if (ed_crc32(slot->bytes,slot->length)!=slot->crc) {
        slot->valid=0; d.update_state=UPDATE_BAD_CRC; d.mode=DEVICE_STOP;
        event(EV_BAD_IMAGE,slot->version); return;
    }
    slot->valid=1; pending_slot=d.update_slot; candidate_pending=1;
    d.update_state=UPDATE_READY; d.mode=DEVICE_STOP; event(EV_READY,slot->version);
}
void ed_step(u32 elapsed_ms) {
    if (elapsed_ms>10000) elapsed_ms=10000;
    for (u32 i=0; i<elapsed_ms; ++i) {
        ed_clock(1);
        if (d.ms%d.params.sample_ms==0) ed_acquire();
        if (d.ms%10==0) ed_control();
        if (d.ms%20==0) { ed_comm(); program_chunk(); }
        if (d.ms%100==0) ed_health();
    }
}
void ed_action(u32 action, u32 value) {
    if (action==1) d.injections=value&15;
    if (action==2) d.params.gain=value>1000?1000:value;
    if (action==3) {
        for (u32 i=0; i<QUEUE_CAPACITY+3; ++i) {
            if (d.queue_count==QUEUE_CAPACITY) { ++d.queue_drops; event(EV_OVERFLOW,2); }
            else { d.queue[d.queue_head]=1000+i; d.queue_head=(d.queue_head+1)%QUEUE_CAPACITY; ++d.queue_count; }
        }
    }
    if (action==4) program_chunk();
}
void ed_power_cycle(void) {
    u32 old_state=d.update_state;
    u32 rollback=0;
    if (candidate_pending && active_slot==pending_slot && active_slot!=confirmed_slot) {
        active_slot=confirmed_slot; candidate_pending=0; rollback=1;
    } else if (candidate_pending && slot_valid(pending_slot)) active_slot=pending_slot;
    if (!slot_valid(active_slot)) active_slot=slot_valid(confirmed_slot)?confirmed_slot:0;
    reset_volatile();
    if (rollback) { d.update_state=UPDATE_IDLE; event(EV_ROLLBACK,flash[active_slot].version); }
    else if (candidate_pending && active_slot==pending_slot) {
        d.update_state=UPDATE_TRIAL; d.trial_started=0; event(EV_BOOT,flash[active_slot].version);
    } else {
        d.update_state=old_state==UPDATE_WRITING?UPDATE_INTERRUPTED:UPDATE_IDLE;
        event(EV_BOOT,flash[active_slot].version);
    }
}
u8 *ed_buffer(void) { return workspace; }
u32 ed_tx_get(u32 byte) { return byte<d.tx_length?d.tx[byte]:0; }
u32 ed_dma_get(u32 bank,u32 index) { return bank<2 && index<DMA_SIZE?d.dma[bank][index]:0; }
u32 ed_stage_image(u32 length, u32 version, u32 expected_crc) {
    if (d.mode==DEVICE_RUN || d.update_state==UPDATE_WRITING || d.update_state==UPDATE_TRIAL) return COMMAND_BUSY;
    if (d.faults || hazards()) return COMMAND_UNSAFE;
    if (length<8 || length>ED_IMAGE_MAX || version<=flash[active_slot].version) return COMMAND_RANGE;
    for (u32 i=0; i<length; ++i) image[i]=workspace[i];
    d.update_slot=1-active_slot; flash[d.update_slot].valid=0;
    fill_bytes(flash[d.update_slot].bytes,0xff,ED_IMAGE_MAX);
    d.update_length=length; d.update_version=version; d.update_crc=expected_crc;
    d.written=0; d.update_state=UPDATE_WRITING; d.mode=DEVICE_UPDATE;
    event(EV_UPDATE,version); return COMMAND_OK;
}
void ed_test_corrupt_nv(void) { saved_crc^=1; }

/* Export is a CRC-protected storage image, independent of C struct padding. */
u32 ed_export_nv(void) {
    fill_bytes(workspace,0,STORAGE_SIZE);
    write32(workspace,0x45444e56); write32(workspace+4,active_slot);
    write32(workspace+8,confirmed_slot); write32(workspace+12,pending_slot);
    write32(workspace+16,candidate_pending); write32(workspace+20,saved_magic);
    write32(workspace+24,saved_params.target); write32(workspace+28,saved_params.sample_ms);
    write32(workspace+32,saved_params.gain); write32(workspace+36,saved_crc);
    u32 position=40;
    for (u32 slot=0; slot<2; ++slot) {
        write32(workspace+position,flash[slot].version); position+=4;
        write32(workspace+position,flash[slot].length); position+=4;
        write32(workspace+position,flash[slot].crc); position+=4;
        write32(workspace+position,flash[slot].valid); position+=4;
        for (u32 i=0; i<ED_IMAGE_MAX; ++i) workspace[position++]=flash[slot].bytes[i];
    }
    write32(workspace+position,ed_crc32(workspace,position));
    return position+4;
}
u32 ed_import_nv(u32 length) {
    u32 expected=40+2*(16+ED_IMAGE_MAX)+4;
    if (length!=expected || read32(workspace)!=0x45444e56 ||
        read32(workspace+length-4)!=ed_crc32(workspace,length-4)) return 0;
    if (read32(workspace+4)>1 || read32(workspace+8)>1 || read32(workspace+12)>1) return 0;
    u32 position=40;
    for (u32 slot=0; slot<2; ++slot) {
        u32 size=read32(workspace+position+4);
        if (size>ED_IMAGE_MAX) return 0;
        position+=16+ED_IMAGE_MAX;
    }
    active_slot=read32(workspace+4); confirmed_slot=read32(workspace+8);
    pending_slot=read32(workspace+12); candidate_pending=read32(workspace+16);
    saved_magic=read32(workspace+20); saved_params=(Parameters){read32(workspace+24),read32(workspace+28),read32(workspace+32)};
    saved_crc=read32(workspace+36); position=40;
    for (u32 slot=0; slot<2; ++slot) {
        flash[slot].version=read32(workspace+position); position+=4;
        flash[slot].length=read32(workspace+position); position+=4;
        flash[slot].crc=read32(workspace+position); position+=4;
        flash[slot].valid=read32(workspace+position); position+=4;
        for (u32 i=0; i<ED_IMAGE_MAX; ++i) flash[slot].bytes[i]=workspace[position++];
    }
    ed_power_cycle(); return 1;
}
double ed_get(u32 field) {
    switch (field) {
    case 0:return d.ms; case 1:return d.mode; case 2:return d.adc_mv;
    case 3:return d.speed; case 4:return d.params.target; case 5:return d.duty/10;
    case 6:return d.current_ma; case 7:return d.bus_mv; case 8:return d.temperature/100.0;
    case 9:return d.queue_count; case 10:return d.blocks; case 11:return d.frames;
    case 12:return d.crc_errors; case 13:return d.rx_overflow; case 14:return d.faults;
    case 15:return d.params.sample_ms; case 16:return d.params.gain;
    case 17:return d.mean_mv; case 18:return d.saved; case 19:return d.nv_valid;
    case 20:return flash[active_slot].version; case 21:return d.update_length?d.written*1000/d.update_length:0;
    case 22:return d.update_state; case 23:return active_slot; case 24:return flash[0].version;
    case 25:return flash[1].version; case 26:return d.rx_count;
    case 27:return d.task_counts[0]; case 28:return d.task_counts[1];
    case 29:return d.task_counts[2]; case 30:return d.task_counts[3];
    case 31:return d.events; case 32:return d.last_cmd; case 33:return slot_valid(active_slot);
    case 34:return d.dma_bank; case 35:return d.dma_position; case 36:return d.queue_drops;
    case 37:return d.tx_length; case 38:return d.task_records;
    case 39:return flash[0].valid; case 40:return flash[1].valid;
    case 41:return flash[0].length; case 42:return flash[1].length;
    case 43:return candidate_pending; case 44:return confirmed_slot;
    default:return 0;
    }
}
u32 ed_event_get(u32 row,u32 field) {
    u32 count=d.events<ED_EVENT_CAPACITY?d.events:ED_EVENT_CAPACITY;
    if (row>=count) return 0;
    u32 first=d.events-count;
    Event e=d.event[(first+row)%ED_EVENT_CAPACITY];
    return field==0?e.time:field==1?e.code:e.value;
}
u32 ed_task_get(u32 row,u32 field) {
    u32 count=d.task_records<ED_TASK_CAPACITY?d.task_records:ED_TASK_CAPACITY;
    if (row>=count) return 0;
    TaskRecord record=d.task[(d.task_records-count+row)%ED_TASK_CAPACITY];
    return field==0?record.time:record.id;
}
