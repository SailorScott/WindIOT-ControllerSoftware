#if !defined _WINDIOT_PRESSURE_WAVE_H
#define _WINDIOT_PRESSURE_WAVE_H

bool  WriteSinglePressureIntoArray(void);

bool setupPressureSensor();

void waveAnalysisAndStore(int);

String PressureWaveJson(unsigned int);


#endif