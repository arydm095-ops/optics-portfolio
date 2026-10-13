#include "servo.h"
#define PI 3.14159265358979323846
void sv_defaults(SvParams*p,sv_u32 profile) {
    p->rs=.12;
    p->ld=.0004;
    p->lq=.0008;
    p->flux=.085;
    p->inertia=.006;
    p->friction=.001;
    p->bus=300;
    p->imax=80;
    p->adc_range=120;
    p->current_bw=800;
    p->speed_bw=25;
    p->position_kp=35;
    p->manual_gains=0;
    for(sv_u32 i=0;i<6;++i)p->gains[i]=0;
    p->pole_pairs=4;
    p->encoder_lines=131072;
    if(profile==1) {
        p->rs=.55;
        p->ld=.0012;
        p->lq=.0012;
        p->flux=.025;
        p->inertia=.00012;
        p->friction=.000025;
        p->bus=48;
        p->imax=8;
        p->adc_range=12;
        p->current_bw=600;
        p->speed_bw=35;
    }
}
static SvPlant derivative(SvPlant m,const SvParams*p,double vd,double vq,double load,sv_u32 gate) {
    SvPlant d;
    double we=(double)p->pole_pairs*m.omega;
    d.id=gate?(vd-p->rs*m.id+we*p->lq*m.iq)/p->ld:-p->rs*m.id/p->ld;
    d.iq=gate?(vq-p->rs*m.iq-we*(p->ld*m.id+p->flux))/p->lq:-p->rs*m.iq/p->lq;
    double torque=1.5*(double)p->pole_pairs*(p->flux+(p->ld-p->lq)*m.id)*m.iq;
    /* load is a resisting torque. Near zero a smooth sign avoids back-driving an unpowered shaft. */
    double opposing=load*m.omega/sv_sqrt(m.omega*m.omega+.04);
    d.omega=(torque-opposing-p->friction*m.omega)/p->inertia;
    d.theta=m.omega;
    return d;
}
void sv_plant_step(SvPlant*m,const SvParams*p,double vd,double vq,double load,double dt,sv_u32 gate) {
    SvPlant a=derivative(*m,p,vd,vq,load,gate),mid=*m;
    mid.id+=.5*dt*a.id;
    mid.iq+=.5*dt*a.iq;
    mid.omega+=.5*dt*a.omega;
    mid.theta+=.5*dt*a.theta;
    SvPlant b=derivative(mid,p,vd,vq,load,gate);
    m->id+=dt*b.id;
    m->iq+=dt*b.iq;
    m->omega+=dt*b.omega;
    m->theta+=dt*b.theta;
}
void sv_mtpa(double torque,const SvParams*p,double*id,double*iq) {
    double q=torque/(1.5*(double)p->pole_pairs*p->flux),d=0,delta=p->lq-p->ld;
    if(delta>1e-9) {
        for(sv_u32 n=0;n<16;++n) {
            d=(p->flux-sv_sqrt(p->flux*p->flux+4*delta*delta*q*q))/(2*delta);
            q=torque/(1.5*(double)p->pole_pairs*(p->flux-delta*d));
        }
    }
    double magnitude=sv_sqrt(d*d+q*q);
    if(magnitude>p->imax) {
        d*=p->imax/magnitude;
        q*=p->imax/magnitude;
    }
    *id=d;
    *iq=q;
}
void sv_svpwm(double alpha,double beta,double bus,double*d) {
    double a=alpha,b=-.5*alpha+.8660254037844386*beta,c=-.5*alpha-.8660254037844386*beta;
    double high=a>b?(a>c?a:c):(b>c?b:c),low=a<b?(a<c?a:c):(b<c?b:c),offset=.5*(high+low);
    d[0]=sv_clip(.5+(a-offset)/bus,.03,.97);
    d[1]=sv_clip(.5+(b-offset)/bus,.03,.97);
    d[2]=sv_clip(.5+(c-offset)/bus,.03,.97);
}
