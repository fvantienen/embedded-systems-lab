#include "Timer.h"

void initTimer(Timer *t, const char *s)
{
    t->elapsedTime = 0.0;
    strncpy(t->nameTime,s,128);
}

void clearTimer(Timer *t)
{
    t->elapsedTime = 0.0;
}

void startTimer(Timer *t)
{
    gettimeofday(&(t->startTime), NULL);
}

void restartTimer(Timer *t)
{
    t->elapsedTime = 0.0;
    gettimeofday(&(t->startTime), NULL);
}

void stopTimer(Timer *t)
{
    gettimeofday(&(t->stopTime), NULL);

    t->elapsedTime =  ( (t->stopTime).tv_sec  - (t->startTime).tv_sec) * 1000.0;      // sec to ms
    t->elapsedTime += ( (t->stopTime).tv_usec - (t->startTime).tv_usec) / 1000.0;   // us to ms
}

void printTimer(Timer *t)
{
    printf("%s = %g msec\n",t->nameTime, t->elapsedTime);
}
