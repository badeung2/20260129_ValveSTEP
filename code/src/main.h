#ifndef MAIN_H
#define MAIN_H
#include "AppBuild.h"
#define C_FET_SETTING (uint16_t)((10UL * FPLL) / C_DELAY_CONST)
#define C_DELAY_3AXIS (uint16_t)((1250UL * FPLL) / C_DELAY_CONST)
extern void main_PeriodicTimerEvent(uint16_t u16Period);
extern void main_noinit_section_init(void);
extern int main(void);
#endif
