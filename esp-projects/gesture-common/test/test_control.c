#include "gesture_control.h"
#include "control_config.h"
#include "FusionAhrs.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void near(float got,float expected,float tolerance) { assert(fabsf(got-expected)<=tolerance); }
static void packet(gesture_control_t *c,uint32_t now,uint8_t flags,int pitch,int roll) {
    gesture_frame_t f={.flags=flags,.sequence=(uint16_t)(c->frame.sequence+1),.pitch_cd=pitch,.roll_cd=roll,.yaw_cd=0,.uptime_ms=now};
    assert(gesture_control_receive(c,&f,now));
}
static void arm(gesture_control_t *c,uint32_t start) {
    for(uint32_t t=start;t<=start+300;t+=50) packet(c,t,GESTURE_VALID,0,0);
    assert(c->state==DRIVE_READY);
    packet(c,start+350,GESTURE_VALID|GESTURE_HELD,2500,0);
    assert(c->state==DRIVE_ACTIVE);
}
static void run_until(gesture_control_t *c,uint32_t start,uint32_t end,int pitch) {
    for(uint32_t t=start;t<=end;t+=10) {
        if(t%50==0) packet(c,t,GESTURE_VALID|GESTURE_HELD,pitch,0);
        gesture_control_step(c,t,0.01f);
        for(int i=0;i<3;++i) assert(fabsf(c->pwm[i])<=CONTROL_MAX_PWM+0.001f);
    }
}
static void protocol_test(void) {
    gesture_frame_t f={.flags=3,.sequence=65535,.pitch_cd=-1234,.roll_cd=2500,.yaw_cd=777,.uptime_ms=0x12345678};
    uint8_t data[14]; gesture_encode(&f,data);
    const uint8_t expected[]={1,3,255,255,0x2e,0xfb,0xc4,9,0x09,0x03,0x78,0x56,0x34,0x12};
    assert(!memcmp(data,expected,14));
    gesture_frame_t decoded; assert(gesture_decode(data,14,&decoded));
    assert(decoded.pitch_cd==-1234 && decoded.roll_cd==2500 && decoded.yaw_cd==777 && decoded.uptime_ms==0x12345678);
    assert(!gesture_decode(data,13,&decoded)); data[0]=2; assert(!gesture_decode(data,14,&decoded));
    data[0]=1; data[1]=128; assert(!gesture_decode(data,14,&decoded));
    assert(gesture_sequence_newer(0,65535)); assert(!gesture_sequence_newer(2,2));
    assert(!gesture_sequence_newer(65535,0)); assert(!gesture_sequence_newer(32768,0));
}
static void states_test(void) {
    gesture_control_t c; gesture_control_init(&c); gesture_control_link(&c,true);
    packet(&c,1,3,2500,0); assert(c.state!=DRIVE_ACTIVE);
    arm(&c,50); run_until(&c,410,1000,2500);
    near(c.pwm[0],-25.980762f,.01f); near(c.pwm[1],0,.01f); near(c.pwm[2],25.980762f,.01f);
    packet(&c,1010,GESTURE_VALID,2500,0); assert(c.state!=DRIVE_ACTIVE);
    for(int i=0;i<3;++i) near(c.pwm[i],0,.001f);
    packet(&c,1020,3,2500,0); assert(c.state!=DRIVE_ACTIVE);
    arm(&c,1050);
    gesture_control_step(&c,1650,.01f); assert(c.state==DRIVE_FAULT);
    packet(&c,1660,3,2500,0); assert(c.state!=DRIVE_ACTIVE);
    arm(&c,1700);
    gesture_frame_t repeated=c.frame;
    assert(!gesture_control_receive(&c,&repeated,2250));
    gesture_control_step(&c,2300,.01f); assert(c.state==DRIVE_FAULT);
    arm(&c,2350); gesture_control_fault(&c,true); assert(c.state==DRIVE_FAULT);
    gesture_control_fault(&c,false); packet(&c,2710,3,2500,0); assert(c.state!=DRIVE_ACTIVE);
    arm(&c,2750); packet(&c,3110,7,0,0); assert(c.state==DRIVE_FAULT);
    arm(&c,3150); packet(&c,3510,3,6100,0); assert(c.state==DRIVE_FAULT);
    arm(&c,3550); gesture_control_link(&c,false); assert(c.state==DRIVE_OFF);
    gesture_control_link(&c,true); packet(&c,3910,3,2500,0); assert(c.state!=DRIVE_ACTIVE);
    arm(&c,3950); gesture_control_step(&c,4310,.2f); assert(c.state==DRIVE_FAULT);
}
static void reverse_test(void) {
    gesture_control_t c; gesture_control_init(&c); gesture_control_link(&c,true); arm(&c,0);
    run_until(&c,360,1000,2500);
    bool reached_zero=false, reversed=false; uint32_t zero=0;
    for(uint32_t t=1010;t<2300;t+=10) {
        if(t%50==0) packet(&c,t,3,-2500,0);
        float previous=c.pwm[0]; gesture_control_step(&c,t,.01f);
        assert(fabsf(c.pwm[0]-previous)<=.601f);
        if(c.pwm[0]==0 && !reached_zero) { reached_zero=true; zero=t; }
        if(c.pwm[0]>0) { assert(reached_zero && t-zero>=CONTROL_REVERSE_MS); reversed=true; }
    }
    assert(reversed);
}
static void wrap_test(void) {
    gesture_control_t c; gesture_control_init(&c); gesture_control_link(&c,true);
    gesture_frame_t f={.flags=1,.sequence=65535,.uptime_ms=0xfffffff0u};
    assert(gesture_control_receive(&c,&f,0xfffffff0u));
    f.sequence=0; f.uptime_ms=34; assert(gesture_control_receive(&c,&f,34));
    f.sequence=1; f.uptime_ms=33; assert(!gesture_control_receive(&c,&f,44));
    gesture_control_step(&c,300,.01f); assert(c.state==DRIVE_FAULT);
}
static void mapping_test(void) {
    near(gesture_axis(5),0,.001f); near(gesture_axis(-5),0,.001f);
    near(gesture_axis(15),.5f,.001f); near(gesture_axis(-25),-1,.001f);
    float out[3]; gesture_mix(1,0,0,out); near(out[0],-25.980762f,.01f); near(out[1],0,.01f); near(out[2],25.980762f,.01f);
    gesture_mix(0,1,0,out); near(out[0],-15,.01f); near(out[1],-30,.01f); near(out[2],-15,.01f);
    gesture_mix(0,0,1,out); near(out[0],-30,.01f); near(out[1],30,.01f); near(out[2],-30,.01f);
}
static void fusion_test(void) {
    FusionAhrs a; FusionAhrsInitialise(&a);
    FusionAhrsSettings settings={.sampleRate=100,.convention=FusionConventionNwu,.gain=.5f,
        .gyroscopeRange=500,.accelerationRejection=10,.magneticRejection=0,.rejectionTimeout=5};
    FusionAhrsSetSettings(&a,&settings);
    FusionVector g={.array={0,0,0}}, gravity={.array={0,0,1}};
    for(int i=0;i<500;++i) FusionAhrsUpdateNoMagnetometer(&a,g,gravity);
    assert(!FusionAhrsGetFlags(&a).startup);
    FusionEuler e=FusionQuaternionToEuler(FusionAhrsGetQuaternion(&a));
    near(e.angle.pitch,0,.1f); near(e.angle.roll,0,.1f);
    /* Forward tilt +30deg => gravity in body coordinates [-sin30,0,cos30]. */
    gravity.axis.x=-.5f; gravity.axis.z=.8660254f; FusionAhrsRestart(&a);
    for(int i=0;i<500;++i) FusionAhrsUpdateNoMagnetometer(&a,g,gravity);
    e=FusionQuaternionToEuler(FusionAhrsGetQuaternion(&a)); near(e.angle.pitch,30,.2f);
}
static void yaw_safety_test(void) {
    gesture_control_t control; gesture_control_init(&control); gesture_control_link(&control,true);
    gesture_frame_t frame={.flags=GESTURE_VALID,.yaw_cd=2500};
    for(uint32_t now=50;now<=450;now+=50) {
        frame.sequence++; frame.uptime_ms=now;
        assert(gesture_control_receive(&control,&frame,now));
        assert(control.state==DRIVE_WAIT_NEUTRAL);
    }
    arm(&control,500);
    frame=control.frame; frame.sequence++; frame.uptime_ms=900; frame.pitch_cd=0; frame.yaw_cd=6100;
    assert(gesture_control_receive(&control,&frame,900)); assert(control.state==DRIVE_FAULT);
    frame.yaw_cd=18001;
    uint8_t data[GESTURE_FRAME_SIZE]; gesture_encode(&frame,data);
    gesture_frame_t decoded; assert(!gesture_decode(data,sizeof(data),&decoded));
    assert(!gesture_decode(data,12,&decoded));
}
static void mixer_geometry_test(void) {
    for(int forward=-1;forward<=1;++forward) for(int lateral=-1;lateral<=1;++lateral) for(int yaw=-1;yaw<=1;++yaw) {
        float output[3], opposite[3];
        gesture_mix(forward,lateral,yaw,output); gesture_mix(-forward,-lateral,-yaw,opposite);
        float scale=CONTROL_MAX_PWM/fmaxf(1,fmaxf(.8660254f*abs(forward)+fabsf(.5f*lateral+yaw),fabsf(-1.0f*lateral+yaw)));
        near((output[2]-output[0])/1.7320508f,forward*scale,.001f);
        near(-(output[0]+2*output[1]+output[2])/3,lateral*scale,.001f);
        near((output[1]-output[0]-output[2])/3,yaw*scale,.001f);
        for(int wheel=0;wheel<3;++wheel) {
            assert(fabsf(output[wheel])<=CONTROL_MAX_PWM+.001f);
            near(output[wheel],-opposite[wheel],.001f);
        }
    }
}
static void straight_slew_ratio_test(void) {
    for(int axis=0;axis<3;++axis) {
        gesture_control_t control; gesture_control_init(&control); gesture_control_link(&control,true); arm(&control,0);
        for(uint32_t now=360;now<=1000;now+=10) {
            if(now==360 || now%50==0) {
                gesture_frame_t frame={.flags=GESTURE_VALID|GESTURE_HELD,.sequence=control.frame.sequence+1,
                    .pitch_cd=axis==0?2500:0,.roll_cd=axis==1?2500:0,.yaw_cd=axis==2?2500:0,.uptime_ms=now};
                assert(gesture_control_receive(&control,&frame,now));
            }
            float previous[3]; memcpy(previous,control.pwm,sizeof(previous));
            gesture_control_step(&control,now,.01f);
            for(int wheel=0;wheel<3;++wheel) assert(fabsf(control.pwm[wheel]-previous[wheel])<=.601f);
            if(axis==0) { near(control.pwm[0],-control.pwm[2],.001f); near(control.pwm[1],0,.001f); }
            if(axis==1) { near(2*control.pwm[0],control.pwm[1],.001f); near(control.pwm[0],control.pwm[2],.001f); }
            if(axis==2) { near(control.pwm[0],-control.pwm[1],.001f); near(control.pwm[0],control.pwm[2],.001f); }
        }
    }
}
int main(void) {
    protocol_test(); states_test(); reverse_test(); wrap_test(); mapping_test(); fusion_test(); yaw_safety_test(); mixer_geometry_test(); straight_slew_ratio_test();
    puts("PASS: protocol, arming/recovery, expiry, estop, tilt, disconnect, reversal, wrapping, mixer, Fusion");
}
