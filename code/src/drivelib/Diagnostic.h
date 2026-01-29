#ifndef DRIVE_LIB_DIAGNOSTIC_H
#define DRIVE_LIB_DIAGNOSTIC_H
#include "AppBuild.h"

#pragma space dp
#pragma space none
#pragma space nodp
#pragma space none
extern void DiagnosticInit(void);
extern void DiagnosticReset(void);
extern void DiagnosticPeriodicTimerEvent(void);
#endif
