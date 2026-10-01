#include "attitude.h"
#include <math.h>
#include <string.h>
static float clamp_unit(float v) {return v>1?1:v< -1?-1:v;}
void uav_attitude_init(uav_attitude_t *s) {memset(s,0,sizeof(*s));s->q[0]=1;}
int uav_attitude_step(uav_attitude_t *s,const float a[3],const float g[3],const float m[3],float dt) {
    float q0=s->q[0],q1=s->q[1],q2=s->q[2],q3=s->q[3];
    float ax=a[0],ay=a[1],az=a[2],mx=m[0],my=m[1],mz=m[2];
    float gx=g[0],gy=g[1],gz=g[2],ex=0,ey=0,ez=0,n,vx,vy,vz;
    float hx,hy,bx,bz,wx,wy,wz;unsigned i;
    if(!isfinite(dt)||dt<=0||dt>0.1f) return -1;
    for(i=0;i<3;i++) if(!isfinite(a[i])||!isfinite(g[i])||!isfinite(m[i])) return -1;
    n=ax*ax+ay*ay+az*az;
    if(n>1e-12f) {
        n=1.0f/sqrtf(n);ax*=n;ay*=n;az*=n;
        vx=2*(q1*q3-q0*q2);vy=2*(q0*q1+q2*q3);vz=q0*q0-q1*q1-q2*q2+q3*q3;
        ex=ay*vz-az*vy;ey=az*vx-ax*vz;ez=ax*vy-ay*vx;
    }
    /* Preserve original integral gain per 5 ms sample, scaled for explicit dt. */
    s->integral[0]+=ex*0.05f*(dt/0.005f);s->integral[1]+=ey*0.05f*(dt/0.005f);
    s->integral[2]+=ez*0.05f*(dt/0.005f);
    gx+=6*ex+s->integral[0];gy+=6*ey+s->integral[1];gz+=6*ez+s->integral[2];
    n=mx*mx+my*my+mz*mz;
    if(n>1e-12f) {
        n=1.0f/sqrtf(n);mx*=n;my*=n;mz*=n;
        hx=(q0*q0+q1*q1-q2*q2-q3*q3)*mx+2*(q1*q2-q0*q3)*my+2*(q1*q3+q0*q2)*mz;
        hy=2*(q1*q2+q0*q3)*mx+(q0*q0-q1*q1+q2*q2-q3*q3)*my+2*(q2*q3-q0*q1)*mz;
        bz=2*(q1*q3-q0*q2)*mx+2*(q2*q3+q0*q1)*my+(q0*q0-q1*q1-q2*q2+q3*q3)*mz;
        bx=sqrtf(hx*hx+hy*hy);
        wx=(q0*q0+q1*q1-q2*q2-q3*q3)*bx+2*(q1*q3-q0*q2)*bz;
        wy=2*(q1*q2-q0*q3)*bx+2*(q2*q3+q0*q1)*bz;
        wz=2*(q1*q3+q0*q2)*bx+(q0*q0-q1*q1-q2*q2+q3*q3)*bz;
        gx+=6*(my*wz-mz*wy);gy+=6*(mz*wx-mx*wz);gz+=6*(mx*wy-my*wx);
    }
    dt*=0.5f;
    s->q[0]=q0+(-q1*gx-q2*gy-q3*gz)*dt;s->q[1]=q1+(q0*gx+q2*gz-q3*gy)*dt;
    s->q[2]=q2+(q0*gy-q1*gz+q3*gx)*dt;s->q[3]=q3+(q0*gz+q1*gy-q2*gx)*dt;
    n=0;for(i=0;i<4;i++) n+=s->q[i]*s->q[i];
    if(!isfinite(n)||n<=1e-12f) {uav_attitude_init(s);return -1;}
    n=1.0f/sqrtf(n);for(i=0;i<4;i++) s->q[i]*=n;
    q0=s->q[0];q1=s->q[1];q2=s->q[2];q3=s->q[3];
    s->yaw_deg=-atan2f(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.295779578f;
    s->pitch_deg=asinf(clamp_unit(2*(q1*q3-q0*q2)))*57.295779578f;
    s->roll_deg=atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)*57.295779578f;
    return 0;
}
