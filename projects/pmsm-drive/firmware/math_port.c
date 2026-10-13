#include "servo.h"
/* Host math is supplied by libm or WebAssembly's Math imports. */
extern double sin(double);
extern double cos(double);
extern double sqrt(double);
double sv_sin(double x) {
    return sin(x);
}
double sv_cos(double x) {
    return cos(x);
}
double sv_sqrt(double x) {
    return sqrt(x);
}
double sv_abs(double x) {
    return x<0?-x:x;
}
double sv_clip(double x,double lo,double hi) {
    return x<lo?lo:(x>hi?hi:x);
}
void *memset(void *dest,int value,unsigned long size) {
    sv_u8 *p=(sv_u8*)dest;
    for(unsigned long i=0;i<size;++i)p[i]=(sv_u8)value;
    return dest;
}
void *memcpy(void *dest,const void *source,unsigned long size) {
    sv_u8 *a=(sv_u8*)dest;
    const sv_u8*b=(const sv_u8*)source;
    for(unsigned long i=0;i<size;++i)a[i]=b[i];
    return dest;
}
