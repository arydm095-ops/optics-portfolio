#ifndef SERVO_H
#define SERVO_H
typedef unsigned char sv_u8;
typedef unsigned int sv_u32;
typedef struct  {
    double rs, ld, lq, flux, inertia, friction, bus, imax, adc_range;
    double current_bw, speed_bw, position_kp;
    sv_u32 pole_pairs, encoder_lines;
}
SvParams;
typedef struct  {
    double id, iq, omega, theta;
}
SvPlant;
typedef struct  {
    SvParams p;
    SvPlant motor;
    sv_u32 tick, enabled, mode, faults, injection, fault_tick;
    sv_u32 speed_calls, position_calls, management_calls, can_rx, crc_errors, seq_errors, rejected;
    sv_u32 last_can_tick, last_control_tick, have_counter, counter, timeout_enabled, encoder_count, adc_a, adc_b;
    double target, load, bus_actual, id, iq, speed, theta_feedback, previous_theta;
    double speed_ref, position_ref, torque_ref, id_ref, iq_ref, mtpa_id, fw_id;
    double int_d, int_q, int_speed, vd, vq, duty[3], ia, ib, ic, voltage_use;
    double trajectory_t, trajectory_start, trajectory_duration, trajectory_velocity, trajectory_acceleration;
    sv_u32 mtpa, weakening, feedforward;
}
SvRuntime;
extern SvRuntime sv;
double sv_sin(double x);
double sv_cos(double x);
double sv_sqrt(double x);
double sv_clip(double x,double lo,double hi);
double sv_abs(double x);
void sv_defaults(SvParams *p,sv_u32 profile);
void sv_plant_step(SvPlant *m,const SvParams *p,double vd,double vq,double load,double dt,sv_u32 gate);
void sv_mtpa(double torque,const SvParams *p,double *id,double *iq);
void sv_svpwm(double alpha,double beta,double bus,double duty[3]);
void sv_control(void);
void sv_trip(sv_u32 code);
void sv_management(void);
void sv_init(sv_u32 profile);
void sv_step(sv_u32 count);
int sv_enable(void);
void sv_stop(void);
int sv_reset_fault(void);
int sv_set(sv_u32 key,double value);
double sv_get(sv_u32 key);
void sv_inject(sv_u32 mask);
sv_u8 sv_crc8(const sv_u8 *p,sv_u32 size);
int sv_can_receive(sv_u32 id,const sv_u8 *bytes,sv_u32 length);
int sv_can_rx(sv_u32 id,sv_u32 length);
void sv_can_feedback(void);
sv_u32 sv_buffer(void);
double sv_plant_probe(sv_u32 field,sv_u32 steps,double vd,double vq,double load);
#endif
