#include "servo.h"
static sv_u8 workspace[256];
sv_u32 sv_buffer(void) {
    return (sv_u32)(unsigned long long)(void*)workspace;
}
sv_u8 sv_crc8(const sv_u8*p,sv_u32 size) {
    sv_u8 crc=255;
    for(sv_u32 i=0;i<size;++i) {
        crc^=p[i];
        for(sv_u32 b=0;b<8;++b)crc=(sv_u8)((crc&128u)?((sv_u32)crc<<1)^0x1du:((sv_u32)crc<<1));
    }
    return (sv_u8)(crc^255u);
}
int sv_can_receive(sv_u32 id,const sv_u8*b,sv_u32 length) {
    if(id!=0x201u||length!=8u) {
        sv.rejected++;
        return 1;
    }
    if(sv_crc8(b,7)!=b[7]) {
        sv.crc_errors++;
        return 2;
    }
    if(sv.have_counter&&b[0]!=(sv_u8)(sv.counter+1u)) {
        sv.seq_errors++;
        return 3;
    }
    sv.counter=b[0];
    sv.have_counter=1;
    sv.last_can_tick=sv.tick;
    sv.can_rx++;
    int raw=(int)b[4]|((int)b[5]<<8);
    if(raw>=32768)raw-=65536;
    double value=(double)raw;
    switch(b[1]) {
        case 1:return sv_enable()?0:4;
        case 2:sv_stop();
        return 0;
        case 3:if(b[2]!=sv.mode) {
            sv.rejected++;
            return 4;
        }
        return sv_set(1,sv.mode==0?value*.1047197551196598:sv.mode==1?value*.01:value*.00174532925199433)?0:4;
        case 4:return sv_set(0,value)?0:4;
        case 5:return sv_reset_fault()?0:4;
        case 6:return sv_set(b[3],value*.01)?0:4;
        case 8:return 0;
        default:sv.rejected++;
        return 1;
    }
}
int sv_can_rx(sv_u32 id,sv_u32 length) {
    return sv_can_receive(id,workspace,length);
}
static void word(sv_u32 offset,int value) {
    workspace[offset]=(sv_u8)value;
    workspace[offset+1]=(sv_u8)((unsigned int)value>>8);
}
void sv_can_feedback(void) {
    workspace[0]=(sv_u8)sv.counter;
    workspace[1]=(sv_u8)sv.enabled;
    workspace[2]=(sv_u8)sv.faults;
    word(3,(int)sv_clip(sv_get(4),-32768,32767));
    word(5,(int)sv_clip(sv_get(12)*100,-32768,32767));
    workspace[7]=sv_crc8(workspace,7);
}
