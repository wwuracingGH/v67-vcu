#ifndef _LOGGING_H_
#define _LOGGING_H_

#include <stdint.h>

void logging_init();
void _update_logging();

#define _enable_logging(state) RTOS_scheduleTask(state, _update_logging, 71)
#define _enable_logging_all() RTOS_scheduleTask(-1, _update_logging, 71)
#define LOG(msg, ...) printf(__FILE__ " line: %d [%d] ", __LINE__, rtos_scheduler.timestamp); printf(msg, ##__VA_ARGS__)
#define LOGLN(msg, ...) LOG(msg, ##__VA_ARGS__); _putchar('\n')
#endif
