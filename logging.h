#ifndef _LOGGING_
#define _LOGGING_

#include <stdint.h>

void _init_logging();
void _enable_logging(uint8_t state);
void _enable_logging_all();

#define LOG(msg, ...) printf(__FILE__ " line: %d [%d] ", __LINE__, rtos_scheduler.timestamp); printf(msg, ##__VA_ARGS__)
#define LOGLN(msg, ...) LOG(msg "\n", ##__VA_ARGS__) 
#endif
