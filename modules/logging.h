/*
 * Nicole Swierstra
 * V67 Logging
 * 
 * Logging functions handled by the DMA mostly to avoid costly copying 
 * 
 */


#ifndef _MODULES_LOGGING_H_
#define _MODULES_LOGGING_H_

#include <stdint.h>
#include "printf/printf.h"

void logging_init();
void _update_logging();

#define LOGGING 1

#define _enable_logging(state) RTOS_scheduleTask(state, _update_logging, 71)
#define _enable_logging_all() RTOS_scheduleTask(-1, _update_logging, 71)
#if (LOGGING == 1)
#define LOG(msg, ...) printf(__FILE__ " line: %d [%d] ", __LINE__, RTOS_getMainTick()); \
                     printf(msg, ##__VA_ARGS__)
#define LOGLN(msg, ...) LOG(msg, ##__VA_ARGS__); _putchar('\n')
#else
#define LOG(msg, ...) (void)
#define LOGLN(msg, ...) (void)
#endif

#endif
