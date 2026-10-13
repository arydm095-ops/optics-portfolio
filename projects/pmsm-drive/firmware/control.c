#include "servo.h"
#define PI 3.14159265358979323846
#define TS .00005
static void trajectory(void) {
    double dt=.001,duration=sv.trajectory_duration,u=sv_clip(sv.trajectory_t/duration,0,1),u2=u*u,u3=u2*u,u4=u3*u,u5=u4*u;
    double displacement=sv.target-sv.trajectory_start;
    sv.position_ref=sv.trajectory_start+displacement*(10*u3-15*u4+6*u5);
    sv.trajectory_velocity=displacement*(30*u2-60*u3+30*u4)/duration;
    sv.trajectory_acceleration=displacement*(60*u-180*u2+120*u3)/(duration*duration);
    if(sv.trajectory_t<duration)sv.trajectory_t+=dt;
    sv.speed_ref=sv.p.position_kp*(sv.position_ref-sv.theta_feedback)+(sv.feedforward?sv.trajectory_velocity:0);
    sv.speed_ref=sv_clip(sv.speed_ref,-500,500);
    sv.position_calls++;
}
void sv_control(void) {
    double theta=sv.theta_feedback*(double)sv.p.pole_pairs,cs=sv_cos(theta),sn=sv_sin(theta);
    double alpha=sv.ia,beta=(sv.ia+2*sv.ib)/1.7320508075688772;
    sv.id=alpha*cs+beta*sn;
    sv.iq=-alpha*sn+beta*cs;
    if(!sv.enabled) {
        sv.vd=0;
        sv.vq=0;
        sv.duty[0]=0;
        sv.duty[1]=0;
        sv.duty[2]=0;
        sv.voltage_use=0;
        return;
    }
    if(sv.tick%20u==0u&&sv.mode==2u)trajectory();
    if(sv.tick%10u==0u&&sv.mode!=1u) {
        double error=(sv.mode==2u?sv.speed_ref:sv.target)-sv.speed;
        if(sv.mode==0u)sv.speed_ref=sv.target;
        double kp=sv_gain(0),ki=sv_gain(1);
        double raw=kp*error+sv.int_speed;
        if(sv.mode==2u&&sv.feedforward)raw+=sv.p.inertia*sv.trajectory_acceleration;
        double max_torque=1.5*(double)sv.p.pole_pairs*sv.p.flux*sv.p.imax;
        sv.torque_ref=sv_clip(raw,-max_torque,max_torque);
        sv.int_speed=sv_clip(sv.int_speed+(ki*error+200*(sv.torque_ref-raw))*.0005,-max_torque,max_torque);
        sv.speed_calls++;
    }
    if(sv.mode==1u)sv.torque_ref=sv.target;
    double d=0,q=sv.torque_ref/(1.5*(double)sv.p.pole_pairs*sv.p.flux);
    if(sv.mtpa)sv_mtpa(sv.torque_ref,&sv.p,&d,&q);
    sv.mtpa_id=d;
    if(sv.weakening&&sv_abs(sv.speed)>.7*sv.bus_actual/(1.7320508075688772*(double)sv.p.pole_pairs*sv.p.flux)) {
        double change=(sv.voltage_use-.89)*sv.p.imax*40*TS;
        sv.fw_id=sv_clip(sv.fw_id-change,-.8*sv.p.imax,0);
    }
    else sv.fw_id=0;
    sv.id_ref=sv_clip(d+sv.fw_id,-.9*sv.p.imax,0);
    q=sv.torque_ref/(1.5*(double)sv.p.pole_pairs*(sv.p.flux+(sv.p.ld-sv.p.lq)*sv.id_ref));
    double qlimit=sv_sqrt(sv.p.imax*sv.p.imax-sv.id_ref*sv.id_ref);
    sv.iq_ref=sv_clip(q,-qlimit,qlimit);
    double ed=sv.id_ref-sv.id,eq=sv.iq_ref-sv.iq,we=(double)sv.p.pole_pairs*sv.speed;
    double rawd=sv_gain(2)*ed+sv.int_d-we*sv.p.lq*sv.iq;
    double rawq=sv_gain(4)*eq+sv.int_q+we*(sv.p.ld*sv.id+sv.p.flux);
    double norm=sv_sqrt(rawd*rawd+rawq*rawq),limit=.94*sv.bus_actual/1.7320508075688772,scale=norm>limit?limit/norm:1;
    sv.vd=rawd*scale;
    sv.vq=rawq*scale;
    sv.int_d=sv_clip(sv.int_d+(sv_gain(3)*ed+1000*(sv.vd-rawd))*TS,-limit,limit);
    sv.int_q=sv_clip(sv.int_q+(sv_gain(5)*eq+1000*(sv.vq-rawq))*TS,-limit,limit);
    sv.voltage_use=sv_sqrt(sv.vd*sv.vd+sv.vq*sv.vq)/limit;
    sv_svpwm(sv.vd*cs-sv.vq*sn,sv.vd*sn+sv.vq*cs,sv.bus_actual,sv.duty);
}
