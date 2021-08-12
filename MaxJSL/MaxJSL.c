// MaxJSL.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#define WIN_VERSION
#define WIN_EXT_VERSION

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "JSL/JoyShockLibrary.h"

#define MAXAPI_USE_MSCRT
#include "c74support/max-includes/ext.h"
#include "c74support/max-includes/ext_obex.h"

typedef struct _maxjsl {
    t_object x_obj;
    t_atom * buffer;
    int * deviceHandles;
    t_int numOfDevices;
    t_int maxDevices;
    uint8_t * poll;
    void * clock;
    t_int rate;
    void * outlet;
}t_maxjsl;

static t_class* maxjsl_class;

void clockHandler(t_maxjsl * x);
void maxjsl_connectDevices(t_maxjsl* x);
void maxjsl_disconnectDevices(t_maxjsl* x);
void maxjsl_controllerType(t_maxjsl* x, t_int devid);
void maxjsl_getState(t_maxjsl * x, t_int devid);
void maxjsl_getIMU(t_maxjsl * x, t_int devid);
void maxjsl_getMotion(t_maxjsl * x, t_int devid);
void maxjsl_getTouch(t_maxjsl * x, t_int devid);
void maxjsl_rumble(t_maxjsl * x, t_int devid, t_int smallRumble, t_int bigRumble);
void maxjsl_poll(t_maxjsl * x, t_int devid, t_int state);
void maxjsl_rate(t_maxjsl * x, t_int rate);
void maxjsl_calibrate(t_maxjsl * x, t_int devid, float xOffset, float yOffset, float zOffset);
void maxjsl_calibration(t_maxjsl * x, t_int devid, t_int state);
void maxjsl_dump(t_maxjsl* x);

void* maxjsl_new(t_int maxDevices) {
    t_maxjsl* x = (t_maxjsl*)object_alloc(maxjsl_class);
    x->maxDevices = maxDevices == 0 ? 4 : maxDevices;
    x->outlet = outlet_new((t_object*)x, NULL);
    x->buffer = (t_atom*)malloc(64 * sizeof(t_atom *));       // For responses
    x->deviceHandles = (int*)malloc(x->maxDevices * sizeof(int));
    x->poll = (uint8_t *)malloc(x->maxDevices * sizeof(uint8_t));
    size_t i;
    for (i = 0; i < x->maxDevices; i++)
    {
        x->poll[i] = 0;
    }
    x->clock = clock_new(x, clockHandler);
    x->rate = 0;
    return x;
}

void* maxjsl_free(t_maxjsl * x) {
    JslDisconnectAndDisposeAll();
    free(x->buffer);
    free(x->deviceHandles);
    free(x->poll);
    clock_free(x->clock);
    return;
}

void ext_main(void* r) {
    t_class* c;
    c = class_new("jsl", (method)maxjsl_new, (method)maxjsl_free, sizeof(t_maxjsl), NULL, A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_connectDevices, "connect", 0);
    class_addmethod(c, (method)maxjsl_disconnectDevices, "disconnect", 0);
    class_addmethod(c, (method)maxjsl_controllerType, "type", A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_getState, "state", A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_getIMU, "imu", A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_getMotion, "motion", A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_rumble, "rumble", A_DEFLONG, A_DEFLONG, A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_poll, "poll", A_DEFLONG, A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_rate, "rate", A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_calibrate, "calibrate", A_DEFLONG, A_DEFFLOAT, A_DEFFLOAT, A_DEFFLOAT, 0);
    class_addmethod(c, (method)maxjsl_calibration, "calibration", A_DEFLONG, A_DEFLONG, 0);
    class_addmethod(c, (method)maxjsl_dump, "dump", 0);
    class_register(CLASS_BOX, c);
    maxjsl_class = c;
}

void clockHandler(t_maxjsl * x){
    size_t i;
    for (i = 0; i < x->numOfDevices; i++)
    {
        if (x->poll[i] > 0)
        {
            maxjsl_getState(x, i);
            maxjsl_getIMU(x, i);
            maxjsl_getMotion(x, i);
            maxjsl_getTouch(x, i);
        }
    }
    if(x->rate > 0){
        clock_delay(x->clock, x->rate);
    }
}

void maxjsl_connectDevices(t_maxjsl* x) {
    x->numOfDevices = JslConnectDevices();
    if(x->numOfDevices > x->maxDevices){
        x->numOfDevices = x->maxDevices;
    }
    int result = JslGetConnectedDeviceHandles(x->deviceHandles, x->maxDevices);
    atom_setsym(x->buffer + 0, gensym("connect"));
    atom_setlong(x->buffer + 1, x->numOfDevices);
    outlet_list(x->outlet, NULL, 2, x->buffer);
    return;
}

void maxjsl_disconnectDevices(t_maxjsl* x) {
    JslDisconnectAndDisposeAll();
    x->numOfDevices = 0;
    atom_setsym(x->buffer + 0, gensym("disconnect"));
    outlet_list(x->outlet, NULL, 1, x->buffer);
    return;
}

void maxjsl_controllerType(t_maxjsl* x, t_int devid) {
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("type"));
    atom_setlong(x->buffer + 1, devid);
    int result = JslGetControllerType(x->deviceHandles[devid]);
    switch (result)
    {
        default:
            break;
        case JS_TYPE_JOYCON_LEFT:
            atom_setsym(x->buffer + 2, gensym("JoyconLeft"));
            break;
        case JS_TYPE_JOYCON_RIGHT:
            atom_setsym(x->buffer + 2, gensym("JoyconRight"));
            break;
        case JS_TYPE_PRO_CONTROLLER:
            atom_setsym(x->buffer + 2, gensym("ProController"));
            break;
        case JS_TYPE_DS4:
            atom_setsym(x->buffer + 2, gensym("DS4"));
            break;
        case JS_TYPE_DS:
            atom_setsym(x->buffer + 2, gensym("DS"));
            break;
    }
    outlet_list(x->outlet, NULL, 3, x->buffer);
    return;
}

void maxjsl_getState(t_maxjsl * x, t_int devid){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("state"));
    atom_setlong(x->buffer + 1, devid);
    JOY_SHOCK_STATE joyShockState = JslGetSimpleState(x->deviceHandles[devid]);
    atom_setlong(x->buffer + 2, (joyShockState.buttons & JSMASK_UP) >> JSOFFSET_UP);
    atom_setlong(x->buffer + 3, (joyShockState.buttons & JSMASK_DOWN) >> JSOFFSET_DOWN);
    atom_setlong(x->buffer + 4, (joyShockState.buttons & JSMASK_LEFT) >> JSOFFSET_LEFT);
    atom_setlong(x->buffer + 5, (joyShockState.buttons & JSMASK_RIGHT) >> JSOFFSET_RIGHT);
    atom_setlong(x->buffer + 6, (joyShockState.buttons & JSMASK_PLUS) >> JSOFFSET_PLUS);
    atom_setlong(x->buffer + 7, (joyShockState.buttons & JSMASK_MINUS) >> JSOFFSET_MINUS);
    atom_setlong(x->buffer + 8, (joyShockState.buttons & JSMASK_LCLICK) >> JSOFFSET_LCLICK);
    atom_setlong(x->buffer + 9, (joyShockState.buttons & JSMASK_RCLICK) >> JSOFFSET_RCLICK);
    atom_setlong(x->buffer + 10, (joyShockState.buttons & JSMASK_L) >> JSOFFSET_L);
    atom_setlong(x->buffer + 11, (joyShockState.buttons & JSMASK_R) >> JSOFFSET_R);
    atom_setlong(x->buffer + 12, (joyShockState.buttons & JSMASK_ZL) >> JSOFFSET_ZL);
    atom_setlong(x->buffer + 13, (joyShockState.buttons & JSMASK_ZR) >> JSOFFSET_ZR);
    atom_setlong(x->buffer + 14, (joyShockState.buttons & JSMASK_S) >> JSOFFSET_S);
    atom_setlong(x->buffer + 15, (joyShockState.buttons & JSMASK_E) >> JSOFFSET_E);
    atom_setlong(x->buffer + 16, (joyShockState.buttons & JSMASK_W) >> JSOFFSET_W);
    atom_setlong(x->buffer + 17, (joyShockState.buttons & JSMASK_N) >> JSOFFSET_N);
    atom_setlong(x->buffer + 18, (joyShockState.buttons & JSMASK_HOME) >> JSOFFSET_HOME);
    atom_setlong(x->buffer + 19, (joyShockState.buttons & JSMASK_CAPTURE) >> JSOFFSET_CAPTURE);
    atom_setlong(x->buffer + 20, (joyShockState.buttons & JSMASK_SL) >> JSOFFSET_SL);
    atom_setlong(x->buffer + 21, (joyShockState.buttons & JSMASK_SR) >> JSOFFSET_SR);
    atom_setfloat(x->buffer + 22, joyShockState.lTrigger);
    atom_setfloat(x->buffer + 23, joyShockState.rTrigger);
    atom_setfloat(x->buffer + 24, joyShockState.stickLX);
    atom_setfloat(x->buffer + 25, joyShockState.stickLY);
    atom_setfloat(x->buffer + 26, joyShockState.stickRX);
    atom_setfloat(x->buffer + 27, joyShockState.stickRY);
    outlet_list(x->outlet, NULL, 28, x->buffer);
}

void maxjsl_getIMU(t_maxjsl * x, t_int devid){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("imu"));
    atom_setlong(x->buffer + 1, devid);
    IMU_STATE imuState = JslGetIMUState(x->deviceHandles[devid]);
    atom_setfloat(x->buffer + 2, imuState.accelX);
    atom_setfloat(x->buffer + 3, imuState.accelY);
    atom_setfloat(x->buffer + 4, imuState.accelZ);
    atom_setfloat(x->buffer + 5, imuState.gyroX);
    atom_setfloat(x->buffer + 6, imuState.gyroY);
    atom_setfloat(x->buffer + 7, imuState.gyroZ);
    outlet_list(x->outlet, NULL, 8, x->buffer);
}

void maxjsl_getMotion(t_maxjsl * x, t_int devid){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("motion"));
    atom_setlong(x->buffer + 1, devid);
    MOTION_STATE motionState = JslGetMotionState(x->deviceHandles[devid]);
    atom_setfloat(x->buffer + 2, motionState.quatW);
    atom_setfloat(x->buffer + 3, motionState.quatX);
    atom_setfloat(x->buffer + 4, motionState.quatY);
    atom_setfloat(x->buffer + 5, motionState.quatZ);
    atom_setfloat(x->buffer + 6, motionState.accelX);
    atom_setfloat(x->buffer + 7, motionState.accelY);
    atom_setfloat(x->buffer + 8, motionState.accelZ);
    atom_setfloat(x->buffer + 9, motionState.gravX);
    atom_setfloat(x->buffer + 10, motionState.gravY);
    atom_setfloat(x->buffer + 11, motionState.gravZ);
    outlet_list(x->outlet, NULL, 12, x->buffer);
}

void maxjsl_getTouch(t_maxjsl * x, t_int devid){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("touch"));
    atom_setlong(x->buffer + 1, devid);
    TOUCH_STATE touchState = JslGetTouchState(x->deviceHandles[devid]);
    atom_setlong(x->buffer + 2, touchState.t0Id);
    atom_setlong(x->buffer + 3, touchState.t0Down);
    atom_setfloat(x->buffer + 4, touchState.t0X);
    atom_setfloat(x->buffer + 5, touchState.t0Y);
    atom_setlong(x->buffer + 6, touchState.t1Id);
    atom_setlong(x->buffer + 7, touchState.t1Down);
    atom_setfloat(x->buffer + 8, touchState.t1X);
    atom_setfloat(x->buffer + 9, touchState.t1Y);
    outlet_list(x->outlet, NULL, 10, x->buffer);
}

void maxjsl_rumble(t_maxjsl * x, t_int devid, t_int smallRumble, t_int bigRumble){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("rumble"));
    atom_setlong(x->buffer + 1, devid);
    JslSetRumble(x->deviceHandles[devid], smallRumble, bigRumble);
    atom_setlong(x->buffer + 2, smallRumble);
    atom_setlong(x->buffer + 3, bigRumble);
    outlet_list(x->outlet, NULL, 4, x->buffer);
}

void maxjsl_poll(t_maxjsl * x, t_int devid, t_int state){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    x->poll[devid] = state;
    atom_setsym(x->buffer + 0, gensym("poll"));
    atom_setlong(x->buffer + 1, devid);
    atom_setlong(x->buffer + 2, x->poll[devid]);
    outlet_list(x->outlet, NULL, 3, x->buffer);
}

void maxjsl_rate(t_maxjsl * x, t_int rate){
    x->rate = rate;
    if(x->rate > 0){
        clock_set(x->clock, x->rate);
    }
}

void maxjsl_calibrate(t_maxjsl * x, t_int devid, float xOffset, float yOffset, float zOffset){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("calibrate"));
    atom_setlong(x->buffer + 1, devid);
    float xGet, yGet, zGet;
    JslGetCalibrationOffset(x->deviceHandles[devid], &xGet, &yGet, &zGet);
    xOffset = xOffset == 0 ? xGet : xOffset;
    yOffset = yOffset == 0 ? yGet : yOffset;
    zOffset = zOffset == 0 ? zGet : zOffset;
    JslSetCalibrationOffset(x->deviceHandles[devid], xOffset, yOffset, zOffset);
    atom_setfloat(x->buffer + 2, xOffset);
    atom_setfloat(x->buffer + 3, yOffset);
    atom_setfloat(x->buffer + 4, zOffset);
    outlet_list(x->outlet, NULL, 5, x->buffer);
}

void maxjsl_calibration(t_maxjsl * x, t_int devid, t_int state){
    if (devid >= x->numOfDevices || devid < 0)
    {
        object_error((t_object *)x, "Unknown Device ID");
        return;
    }
    atom_setsym(x->buffer + 0, gensym("calibration"));
    atom_setlong(x->buffer + 1, devid);
    if(state > 0){
        JslStartContinuousCalibration(x->deviceHandles[devid]);
    }
    else if (state < 0){
        JslResetContinuousCalibration(x->deviceHandles[devid]);
    }
    else{
        JslPauseContinuousCalibration(x->deviceHandles[devid]);
    }
    atom_setlong(x->buffer + 2, state);
    outlet_list(x->outlet, NULL, 3, x->buffer);
}

void maxjsl_dump(t_maxjsl * x){
    atom_setsym(x->buffer + 0, gensym("dump"));
    size_t i;
    for (i = 0; i < x->numOfDevices; i++)
    {
        atom_setlong(x->buffer + 1, i);
        atom_setlong(x->buffer + 2, x->deviceHandles[i]);
        atom_setfloat(x->buffer + 3, JslGetStickStep(x->deviceHandles[i]));
        atom_setfloat(x->buffer + 4, JslGetTriggerStep(x->deviceHandles[i]));
        atom_setfloat(x->buffer + 5, JslGetPollRate(x->deviceHandles[i]));
        atom_setlong(x->buffer + 6, JslGetControllerType(x->deviceHandles[i]));
        atom_setlong(x->buffer + 7, JslGetControllerColour(x->deviceHandles[i]));
        outlet_list(x->outlet, NULL, 8, x->buffer);
    }
}