#ifndef __RTSPSERVER_H__
#define __RTSPSERVER_H__
//=====================   C   =====================
#include "system.h"
#include "config.h"

//=====================  SDK  =====================
#include "frame_queue.h"
#include "rtsp.h"

extern int init_rtsp_main_process();
extern int rtspServerInit(const char *moduleName);

#endif
