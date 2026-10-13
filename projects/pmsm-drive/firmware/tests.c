#include "servo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
static const char*passed[100];
static int total;
static void check(const char*name,int condition) {
    if(!condition) {
        fprintf(stderr,"FAIL %s: rpm=%.3f id=%.3f iq=%.3f fault=%u\n",name,sv_get(4),sv_get(8),sv_get(9),sv.faults);
        exit(1);
    }
    passed[total++]=name;
}
static void speed_run(double rpm,double load,unsigned steps) {
    sv_init(0);
    sv_set(0,0);
    sv_set(1,rpm*.1047197551196598);
    sv_set(2,load);
    sv_enable();
    sv_step(steps);
}
static void frame(sv_u8*b,sv_u8 seq,sv_u8 cmd,int value) {
    b[0]=seq;
    b[1]=cmd;
    b[2]=(sv_u8)sv.mode;
    b[3]=0;
    b[4]=(sv_u8)value;
    b[5]=(sv_u8)((unsigned int)value>>8);
    b[6]=0;
    b[7]=sv_crc8(b,7);
}
int main(void) {
    sv_init(0);
    check("reset_disabled",!sv.enabled&&!sv.faults);
    check("crc8_sae_j1850_reference",sv_crc8((const sv_u8*)"123456789",9)==0x4bu);
    speed_run(3000,0,20000);
    check("speed_step_tracks_3000rpm",fabs(sv_get(4)-3000)<5);
    check("dq_current_follows_reference",fabs(sv.id-sv.id_ref)<.2&&fabs(sv.iq-sv.iq_ref)<.2);
    check("multi_rate_counters",sv.tick==20000&&sv.speed_calls==2000&&sv.management_calls==1000&&sv.position_calls==0);
    check("svpwm_duty_bounds",sv.duty[0]>=.03&&sv.duty[0]<=.97&&sv.duty[1]>=.03&&sv.duty[1]<=.97&&sv.duty[2]>=.03&&sv.duty[2]<=.97);
    check("current_vector_limit",sv_sqrt(sv.id_ref*sv.id_ref+sv.iq_ref*sv.iq_ref)<=sv.p.imax+1e-6);
    sv_set(2,8);
    sv_step(12000);
    check("load_step_speed_recovery",fabs(sv_get(4)-3000)<5);
    check("load_step_torque_balance",fabs(sv_get(12)-8-sv.p.friction*sv.motor.omega)<.15);
    double d,q;
    sv_mtpa(30,&sv.p,&d,&q);
    double basic=30/(1.5*sv.p.pole_pairs*sv.p.flux);
    check("ipmsm_mtpa_reduces_current",sv_sqrt(d*d+q*q)<basic&&d<0);
    check("mtpa_preserves_requested_torque",fabs(1.5*sv.p.pole_pairs*(sv.p.flux+(sv.p.ld-sv.p.lq)*d)*q-30)<1e-6);
    double best=1e9;
    for(int n=0;n<=8000;n++) {
        double nd=-(double)n/100,niq=30/(1.5*sv.p.pole_pairs*(sv.p.flux+(sv.p.ld-sv.p.lq)*nd)),m=hypot(nd,niq);
        if(m<best)best=m;
    }
    check("mtpa_matches_independent_numeric_search",fabs(hypot(d,q)-best)<.001);
    SvParams sp;
    sv_defaults(&sp,1);
    sv_mtpa(.5,&sp,&d,&q);
    check("spmsm_mtpa_falls_back_to_id_zero",d==0);
    speed_run(6500,3,60000);
    check("weakening_reaches_above_base_speed",fabs(sv_get(4)-6500)<20&&sv.fw_id<0);
    check("weakening_voltage_limit",sv.voltage_use<=1.000001);
    sv_init(1);
    sv_set(0,2);
    sv_set(1,6.283185307179586);
    sv_enable();
    sv_step(40000);
    check("servo_position_tracks_one_revolution",fabs(sv_get(6)-360)<.15&&fabs(sv_get(4))<3);
    check("position_task_runs_at_1khz",sv.position_calls==2000&&sv.speed_calls==4000);
    check("s_curve_ends_with_zero_velocity_acceleration",fabs(sv.trajectory_velocity)<1e-8&&fabs(sv.trajectory_acceleration)<1e-8);
    sv_stop();
    sv_step(1);
    check("stop_removes_pwm_output",sv.duty[0]==0&&sv.duty[1]==0&&sv.duty[2]==0&&!sv.enabled);
    for(unsigned bit=1;bit<=16;bit<<=1) {
        speed_run(1000,0,4000);
        unsigned start=sv.tick;
        sv_inject(bit);
        sv_step(20);
        check(bit==1?"overcurrent_fast_trip":bit==2?"undervoltage_trip":bit==4?"encoder_loss_trip":bit==8?"communication_timeout_trip":"deadline_fault_trip",(sv.faults&bit)&&!sv.enabled&&sv.fault_tick-start<=20);
        check("fault_reset_blocked_while_injected",!sv_reset_fault());
        sv_inject(0);
        sv_step(20);
        check("fault_latched_after_condition_removed",sv.faults!=0);
        check("fault_reset_requires_explicit_reenable",sv_reset_fault()&&!sv.enabled&&sv.faults==0);
    }
    sv_init(0);
    sv_u8 b[8];
    frame(b,250,1,0);
    check("can_enable",sv_can_receive(0x201,b,8)==0&&sv.enabled);
    frame(b,251,3,2000);
    check("can_target_signed_scale",sv_can_receive(0x201,b,8)==0&&fabs(sv.target-2000*.1047197551196598)<1e-9);
    check("can_duplicate_counter_rejected",sv_can_receive(0x201,b,8)==3);
    frame(b,252,3,-1200);
    b[7]^=1;
    check("can_corrupt_crc_rejected",sv_can_receive(0x201,b,8)==2&&sv.crc_errors==1);
    b[7]^=1;
    check("can_negative_target_accepted",sv_can_receive(0x201,b,8)==0&&sv.target<0);
    check("can_wrong_id_dlc_rejected",sv_can_receive(0x202,b,8)==1&&sv_can_receive(0x201,b,7)==1);
    check("calibration_bounds_rejected",!sv_set(8,-.001)&&!sv_set(7,9000));
    check("calibration_adc_headroom",!sv_set(9,120));
    sv_set(11,.00001);
    check("calibration_electrical_time_constant",!sv_set(13,5));
    sv_set(11,.0004);
    sv_set(16,1);
    sv_step(2040);
    check("real_can_deadline_expires",(sv.faults&8)&&!sv.enabled);
    speed_run(2400,5,36000);
    printf("{\"passed\":%d,\"total\":%d,\"tests\":[",total,total);
    for(int n=0;n<total;n++)printf("%s{\"name\":\"%s\",\"passed\":true}",n?",":"",passed[n]);
    printf("],\"snapshot\":[");
    for(unsigned n=0;n<52;n++)printf("%s%.12g",n?",":"",sv_get(n));
    printf("]}\n");
    return 0;
}
