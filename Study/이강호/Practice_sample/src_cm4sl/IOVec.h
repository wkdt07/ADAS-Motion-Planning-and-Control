/*
 *****************************************************************************
 *  CarMaker - Version 14.0.1
 *  Virtual Test Driving Tool
 *
 *  Copyright ©1998-2025 IPG Automotive GmbH. All rights reserved.
 *  www.ipg-automotive.com
 *****************************************************************************
 */

#ifndef _IO_H__
#define _IO_H__

#ifdef __cplusplus
extern "C" {
#endif

struct tInfos;

/*** Input Vector, signals from hardware, ... */
typedef struct {
    double T;
    float  DeltaT; /* DeltaT of the last time step */
} tIOVec;

extern tIOVec IO;

/*** I/O configuration */

/* extern int IO_None; DON'T - Variable is predefined by CarMaker! */
extern int IO_CAN_IF;
extern int IO_FlexRay;

/*** I/O calibration */

typedef struct tCal {
    float Min;
    float Max;
    float LimitLow;
    float LimitHigh;
    float Factor;
    float Offset;
    int   Rezip;
} tCal;

void  iGetCal(struct tInfos *Inf, char const *key, tCal *cal, int optional);
float CalIn(tCal *cal, int Value);
float CalInF(tCal *cal, float Value);
int   CalOut(tCal *cal, float Value);
float CalOutF(tCal *cal, float Value);
int   LimitInt(float fValue, int Min, int Max);

int IO_Init_First(void);
int IO_Init(void);
int IO_Init_Finalize(void);

int  IO_Param_Get(struct tInfos *inf);
void IO_BeginCycle(void);
void IO_In(unsigned CycleNo);
void IO_Out(unsigned CycleNo);

void IO_Cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _IO_H__ */
