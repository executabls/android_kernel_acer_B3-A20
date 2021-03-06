#ifndef _G3DKMD_HWDBG_H_
#define _G3DKMD_HWDBG_H_

#include "g3dkmd_define.h"
#include "yl_define.h"

#define G3DKMD_HWDBG_ALIGN         16
#define G3DKMD_HWDBG_BUF_SIZE      32 * 1024

#ifdef YL_HW_DEBUG_DUMP_TEMP

void g3dKmdHwDbgAllocMemory(unsigned int size, unsigned int align);
void g3dKmdHwDbgFreeMemory(void);
void g3dKmdHwDbgSetDumpFlushBase(void);
void g3dKmdHwDbgSetDumpReg( unsigned int value );
void g3dKmdHwDbgSetFrameID( unsigned int uBegin, unsigned int uEnd );
unsigned int g3dKmdHwDbgGetBeginFrameID(void);
unsigned int g3dKmdHwDbgGetEndFrameID(void);
void g3dKmdHwDbgSetRepeatTimes( unsigned int uTimes );
unsigned int g3dKmdHwDbgGetRepeatTimes(void);

#ifdef G3DKMD_SNAPSHOT_PATTERN
unsigned int g3dKmdHwDbgGetDumpFlushBase(void);
#endif
#endif
#endif //_G3DKMD_HWDBG_H_
