#if !defined _WINDIOT_WIND_H
#define _WINDIOT_WIND_H

extern volatile unsigned long TimeAtWindStartCounting;


void countOneSpin(void);

void readWindIntoArray(bool notSkipDirection);

String WindDataJson(unsigned int);

#endif