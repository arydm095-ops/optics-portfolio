#include "servo.h"
#define PI 3.14159265358979323846
SvRuntime sv;
static void zero_runtime(void) {
    sv_u8*p=(sv_u8*)&sv;
    for(sv_u32 i=0;i<(sv_u32)sizeof(sv);++i)p[i]=0;
}
void sv_init(sv_u32 profile) {
    zero_runtime();
    sv_defaults(&sv.p,profile);
    sv.bus_actual=sv.p.bus;
    sv.mtpa=1;
    sv.weakening=1;
    sv.feedforward=1;
    sv.trajectory_duration=.8;
}
void sv_trip(sv_u32 code) {
    if(!sv.faults)sv.fault_tick=sv.tick;
    sv.faults|=code;
    sv.enabled=0;
    sv.duty[0]=sv.duty[1]=sv.duty[2]=0;
}
void sv_management(void) {
    sv.management_calls++;
    if(sv.bus_actual<sv.p.bus*.65)sv_trip(2);
    if(sv.injection&4u)sv_trip(4);
    if(sv.enabled&&(sv.tick-sv.last_control_tick)>20u)sv_trip(16);
    if(sv.enabled&&((sv.injection&8u)||(sv.timeout_enabled&&(sv.tick-sv.last_can_tick)>2000u)))sv_trip(8);
}
int sv_enable(void) {
    if(sv.faults||sv.injection) {
        sv.rejected++;
        return 0;
    }
    sv.enabled=1;
    sv.int_d=sv.int_q=sv.int_speed=0;
    sv.notch_z1=sv.notch_z2=0;
    sv.trajectory_t=0;
    sv.trajectory_start=sv.theta_feedback;
    sv.last_can_tick=sv.tick;
    return 1;
}
void sv_stop(void) {
    sv.enabled=0;
    sv.int_d=sv.int_q=sv.int_speed=0;
    sv.notch_z1=sv.notch_z2=0;
    sv.id_ref=sv.iq_ref=sv.torque_ref=0;
}
int sv_reset_fault(void) {
    if(sv.injection||sv.bus_actual<sv.p.bus*.65) {
        sv.rejected++;
        return 0;
    }
    sv.faults=0;
    sv_stop();
    return 1;
}
void sv_inject(sv_u32 mask) {
    sv.injection=mask;
}
double sv_gain(sv_u32 index) {
    if(index>=6u)return 0;
    if(sv.p.manual_gains)return sv.p.gains[index];
    double wb=2*PI*sv.p.speed_bw,wc=2*PI*sv.p.current_bw;
    double inertia=sv.p.inertia+(sv.p.elastic?sv.p.load_inertia:0);
    switch(index) {
        case 0:return sv_clip(2*.9*wb*inertia-sv.p.friction,0,100000);
        case 1:return inertia*wb*wb;
        case 2:return sv.p.ld*wc;
        case 4:return sv.p.lq*wc;
        default:return sv.p.rs*wc;
    }
}
int sv_set(sv_u32 key,double v) {
    if(!(v==v)||sv_abs(v)>1e9) {
        sv.rejected++;
        return 0;
    }
    switch(key) {
        case 0: if(v<0||v>2)break;
        sv.mode=(sv_u32)v;
        sv.int_speed=0;
        sv.trajectory_start=sv.theta_feedback;
        sv.trajectory_t=0;
        return 1;
        case 1: if(sv_abs(v)>(sv.mode==0u?sv.p.speed_limit:1000))break;
        sv.target=v;
        sv.trajectory_start=sv.theta_feedback;
        sv.trajectory_t=0;
        return 1;
        case 2: if(v<0||v>100)break;
        sv.load=v;
        return 1;
        case 3: sv.mtpa=v!=0;
        return 1;
        case 4: sv.weakening=v!=0;
        return 1;
        case 5: sv.feedforward=v!=0;
        return 1;
        case 6: if(v<1||v>250)break;
        sv.p.speed_bw=v;
        return 1;
        case 7: if(v<50||v>4000)break;
        sv.p.current_bw=v;
        return 1;
        case 8: if(v<.000001||v>2)break;
        sv.p.inertia=v;
        return 1;
        case 9: if(v<.1||v>1000||v>sv.p.adc_range*.85)break;
        sv.p.imax=v;
        return 1;
        case 10: if(v<.0001||v>2)break;
        sv.p.flux=v;
        return 1;
        case 11: if(v<.000001||v>.1||v<sv.p.rs*.000025)break;
        sv.p.ld=v;
        return 1;
        case 12: if(v<.000001||v>.1||v<sv.p.rs*.000025)break;
        sv.p.lq=v;
        return 1;
        case 13: if(v<.001||v>20||v*.000025>sv.p.ld||v*.000025>sv.p.lq)break;
        sv.p.rs=v;
        return 1;
        case 14: if(v<5||v>1000)break;
        sv.p.bus=v;
        sv.bus_actual=v;
        return 1;
        case 15: if(v<.1||v>10)break;
        sv.trajectory_duration=v;
        return 1;
        case 16: sv.timeout_enabled=v!=0;
        sv.last_can_tick=sv.tick;
        return 1;
        case 20: case 21: case 22: case 23: case 24: case 25:
        if(v<0||v>(key==21u?100000000:(key==23u||key==25u)?10000000:key==20u?100000:10000))break;
        if(!sv.p.manual_gains) {
            for(sv_u32 i=0;i<6;++i)sv.p.gains[i]=sv_gain(i);
            sv.p.manual_gains=1;
        }
        sv.p.gains[key-20u]=v;
        return 1;
        case 26: if(v<0||v>150)break;
        sv.p.position_kp=v;
        return 1;
        case 27: if(v!=0)break;
        sv.p.manual_gains=0;
        return 1;
        case 28: if(v<1||v>32||v!=(double)(sv_u32)v)break;sv.p.pole_pairs=(sv_u32)v;return 1;
        case 29: if(v<64||v>4194304||v!=(double)(sv_u32)v)break;sv.p.encoder_lines=(sv_u32)v;return 1;
        case 30: if(v<1||v>2000||v*.85<sv.p.imax)break;sv.p.adc_range=v;return 1;
        case 31: if(v<0||v>2)break;sv.p.friction=v;return 1;
        case 32: sv.p.elastic=v!=0;return 1;
        case 33: if(v<.000001||v>5)break;sv.p.load_inertia=v;return 1;
        case 34: if(v<.01||v>100000)break;sv.p.stiffness=v;return 1;
        case 35: if(v<0||v>20)break;sv.p.shaft_damping=v;return 1;
        case 36: if(v<0||v>2)break;sv.p.load_friction=v;return 1;
        case 37: if(v<0||v>100)break;sv.p.coulomb=v;return 1;
        case 38: sv.p.notch=v!=0;sv.notch_z1=sv.notch_z2=0;return 1;
        case 39: if(v<2||v>800)break;sv.p.notch_hz=v;return 1;
        case 40: if(v<.001||v>1||v>sv.p.notch_pole)break;sv.p.notch_zero=v;return 1;
        case 41: if(v<.01||v>2||v<sv.p.notch_zero)break;sv.p.notch_pole=v;return 1;
        case 42: if(v<10||v>2000)break;sv.p.speed_limit=v;return 1;
        default:break;
    }
    sv.rejected++;
    return 0;
}
void sv_step(sv_u32 count) {
    for(sv_u32 k=0;k<count;++k) {
        double theta=sv.motor.theta*(double)sv.p.pole_pairs,cs=sv_cos(theta),sn=sv_sin(theta);
        double a=sv.motor.id*cs-sv.motor.iq*sn,beta=sv.motor.id*sn+sv.motor.iq*cs;
        double b=-.5*a+.8660254037844386*beta;
        sv.adc_a=(sv_u32)(sv_clip(2048+a*2047/sv.p.adc_range,0,4095)+.5);
        sv.adc_b=(sv_u32)(sv_clip(2048+b*2047/sv.p.adc_range,0,4095)+.5);
        sv.ia=((double)sv.adc_a-2048)*sv.p.adc_range/2047;
        sv.ib=((double)sv.adc_b-2048)*sv.p.adc_range/2047;
        sv.ic=-sv.ia-sv.ib;
        double count_float=sv.motor.theta*(double)sv.p.encoder_lines/(2*PI);
        long long unwrapped=(long long)(count_float+(count_float>=0?.5:-.5));
        sv.encoder_count=(sv_u32)((unwrapped%(long long)sv.p.encoder_lines+(long long)sv.p.encoder_lines)%(long long)sv.p.encoder_lines);
        sv.theta_feedback=(double)unwrapped*(2*PI)/(double)sv.p.encoder_lines;
        sv.speed+=.08*((sv.theta_feedback-sv.previous_theta)/.00005-sv.speed);
        sv.previous_theta=sv.theta_feedback;
        sv.bus_actual=(sv.injection&2u)?sv.p.bus*.5:sv.p.bus;
        if((sv.injection&1u)||sv_sqrt(sv.ia*sv.ia+sv.ib*sv.ib+sv.ic*sv.ic)>2.3*sv.p.imax)sv_trip(1);
        if(!(sv.injection&16u)) {
            sv_control();
            sv.last_control_tick=sv.tick;
        }
        /* Average inverter: reconstruct phase voltage from the calculated SVPWM duties. */
        double mean=(sv.duty[0]+sv.duty[1]+sv.duty[2])/3,va=(sv.duty[0]-mean)*sv.bus_actual,vb=(sv.duty[1]-mean)*sv.bus_actual;
        double valpha=va,vbeta=(va+2*vb)/1.7320508075688772;
        double vd=valpha*cs+vbeta*sn,vq=-valpha*sn+vbeta*cs;
        for(sv_u32 sub=0;sub<4;++sub)sv_plant_step(&sv.motor,&sv.p,vd,vq,sv.load,.0000125,sv.enabled);
        sv.tick++;
        if(sv.tick%20u==0u)sv_management();
    }
}
double sv_get(sv_u32 key) {
    switch(key) {
        case 0:return (double)sv.tick*.00005;
        case 1:return sv.enabled;
        case 2:return sv.mode;
        case 3:return sv.faults;
        case 4:return sv.motor.omega*60/(2*PI);
        case 5:return sv.speed_ref*60/(2*PI);
        case 6:return sv.motor.theta*180/PI;
        case 7:return sv.position_ref*180/PI;
        case 8:return sv.id;
        case 9:return sv.iq;
        case 10:return sv.id_ref;
        case 11:return sv.iq_ref;
        case 12:return 1.5*(double)sv.p.pole_pairs*(sv.p.flux+(sv.p.ld-sv.p.lq)*sv.motor.id)*sv.motor.iq;
        case 13:return sv.load;
        case 14:return sv.vd;
        case 15:return sv.vq;
        case 16:return sv.bus_actual;
        case 17:return sv.voltage_use;
        case 18:return sv.ia;
        case 19:return sv.ib;
        case 20:return sv.ic;
        case 21:return sv.duty[0];
        case 22:return sv.duty[1];
        case 23:return sv.duty[2];
        case 24:return sv.encoder_count;
        case 25:return sv.tick;
        case 26:return sv.speed_calls;
        case 27:return sv.position_calls;
        case 28:return sv.management_calls;
        case 29:return sv.can_rx;
        case 30:return sv.crc_errors;
        case 31:return sv.seq_errors;
        case 32:return sv.rejected;
        case 33:return (double)sv.fault_tick*.00005;
        case 34:return sv.torque_ref;
        case 35:return sv.mtpa_id;
        case 36:return sv.fw_id;
        case 37:return sv.motor.id;
        case 38:return sv.adc_a;
        case 39:return sv.adc_b;
        case 40:return sv.motor.iq;
        case 41:return sv.p.imax;
        case 42:return sv.p.inertia;
        case 43:return sv.p.ld;
        case 44:return sv.p.lq;
        case 45:return sv.p.flux;
        case 46:return sv.p.bus;
        case 47:return sv.p.speed_bw;
        case 48:return sv.p.current_bw;
        case 49:return sv.p.rs;
        case 50:return sv.trajectory_velocity;
        case 51:return sv.trajectory_acceleration;
        case 52:return sv.counter;
        case 53:return sv.have_counter;
        case 54:return sv.p.adc_range;
        case 55:return sv.motor.load_theta*180/PI;
        case 56:return sv.motor.load_omega*60/(2*PI);
        case 57:return sv.p.elastic?sv.p.stiffness*(sv.motor.theta-sv.motor.load_theta)+sv.p.shaft_damping*(sv.motor.omega-sv.motor.load_omega):0;
        case 58:return (sv.motor.theta-sv.motor.load_theta)*180/PI;
        case 59:return sv.p.coulomb*sv.motor.omega/sv_sqrt(sv.motor.omega*sv.motor.omega+.04);
        case 70:return sv.p.pole_pairs;case 71:return sv.p.encoder_lines;case 72:return sv.p.adc_range;
        case 73:return sv.p.friction;case 74:return sv.p.elastic;case 75:return sv.p.load_inertia;
        case 76:return sv.p.stiffness;case 77:return sv.p.shaft_damping;case 78:return sv.p.load_friction;
        case 79:return sv.p.coulomb;case 80:return sv.p.notch;case 81:return sv.p.notch_hz;
        case 82:return sv.p.notch_zero;case 83:return sv.p.notch_pole;case 84:return sv.p.speed_limit;
        case 85:return sv.trajectory_duration;
        case 60: case 61: case 62: case 63: case 64: case 65:return sv_gain(key-60u);
        case 66:return sv.p.position_kp;
        default:return 0;
    }
}
double sv_plant_probe(sv_u32 field,sv_u32 steps,double vd,double vq,double load) {
    SvPlant m= {
        0,0,0,0,0,0
    };
    SvParams p;
    sv_defaults(&p,0);
    for(sv_u32 i=0;i<steps;++i)sv_plant_step(&m,&p,vd,vq,load,.0000125,1);
    return field==0?m.id:field==1?m.iq:field==2?m.omega:m.theta;
}

double sv_mechanical_probe(sv_u32 field,sv_u32 steps) {
    SvParams p=sv.p;SvPlant m={0,0,1,0,0,0};
    p.elastic=1;p.friction=0;p.load_friction=0;p.shaft_damping=0;p.coulomb=0;
    for(sv_u32 n=0;n<steps;++n)sv_plant_step(&m,&p,0,0,0,.0000125,0);
    return field==0?m.omega:field==1?m.load_omega:field==2?m.theta:m.load_theta;
}
