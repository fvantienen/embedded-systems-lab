#ifndef TIMER_H
#define TIMER_H

#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<assert.h>
#include<sys/time.h>

typedef struct
{
    struct timeval startTime;
    struct timeval stopTime;
    double elapsedTime;
    char nameTime[128];
}Timer;

void initTimer(Timer *t, const char *s);
void clearTimer(Timer *t);
void startTimer(Timer *t);
void restartTimer(Timer *t);
void stopTimer(Timer *t);
void printTimer(Timer *t);

#endif