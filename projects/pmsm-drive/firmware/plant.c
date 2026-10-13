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
    p->elastic=0;p->notch=0;p->load_inertia=.018;p->stiffness=1776.528792196;
    p->shaft_damping=.08;p->load_friction=.001;p->coulomb=0;
    p->notch_hz=100;p->notch_zero=.04;p->notch_pole=.35;p->speed_limit=1000;
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
        p->load_inertia=.00036;p->stiffness=35.5305758439;
        p->shaft_damping=.0016;p->load_friction=.000025;
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
    double friction=p->coulomb*m.omega/sv_sqrt(m.omega*m.omega+.04);
    if(p->elastic) {
        double shaft=p->stiffness*(m.theta-m.load_theta)+p->shaft_damping*(m.omega-m.load_omega);
        opposing=load*m.load_omega/sv_sqrt(m.load_omega*m.load_omega+.04);
        d.omega=(torque-shaft-p->friction*m.omega-friction)/p->inertia;
        d.load_omega=(shaft-opposing-p->load_friction*m.load_omega)/p->load_inertia;
        d.load_theta=m.load_omega;
    } else {
        d.omega=(torque-opposing-p->friction*m.omega-friction)/p->inertia;
        d.load_omega=d.omega;d.load_theta=m.omega;
    }
    d.theta=m.omega;
    return d;
}
void sv_plant_step(SvPlant*m,const SvParams*p,double vd,double vq,double load,double dt,sv_u32 gate) {
    SvPlant a=derivative(*m,p,vd,vq,load,gate),mid=*m;
    mid.id+=.5*dt*a.id;
    mid.iq+=.5*dt*a.iq;
    mid.omega+=.5*dt*a.omega;
    mid.theta+=.5*dt*a.theta;
    mid.load_omega+=.5*dt*a.load_omega;mid.load_theta+=.5*dt*a.load_theta;
    SvPlant b=derivative(mid,p,vd,vq,load,gate);
    m->id+=dt*b.id;
    m->iq+=dt*b.iq;
    m->omega+=dt*b.omega;
    m->theta+=dt*b.theta;
    m->load_omega+=dt*b.load_omega;m->load_theta+=dt*b.load_theta;
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
