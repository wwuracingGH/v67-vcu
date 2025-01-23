#include <stdint.h>
#ifndef _RTOS_
#define _RTOS_

#define RTOS_maxEventNum 16
#define RTOS_maxTaskNum  64
#define RTOS_maxStateNum 16

typedef struct {
    void (*callback)();
    uint16_t countdown;
    int8_t nextEvent;
} rtos_event;

typedef struct {
    void (*callback)();
    uint16_t reset_ms;
    uint16_t counter;
} rtos_task;

typedef struct {
    void (*entry)();
    void (*exit)();
    uint64_t taskMask;
} rtos_state;

typedef struct {
    uint16_t eventQue;
    int firstEventIndex;
    rtos_event eventHeap[RTOS_maxEventNum];

    uint64_t taskQue; 
    uint8_t numberOfTasks;
    rtos_task tasks[RTOS_maxTaskNum];

    uint8_t numberOfStates;
    uint8_t state;
    rtos_state states[RTOS_maxStateNum];
} kernel;

extern kernel rtos_scheduler;

/**
 *  Inits and zeros the RTOS, just in case
 */
int RTOS_init(void);

/**
 *  Adds a state to the car, and then returns the index of the state
 */
int RTOS_addState(void (*start), void (*stop));

/**
 * switches the cars state to the state in the pool at index of @state
 */
int RTOS_switchState(uint8_t state);

/**
 * Checks if you are in that state
 */
int RTOS_inState(uint8_t state);

/**
 * removes the first event in the queue
 */
int RTOS_removeFirstEvent(void);

/**
 * schedules an event to take place in the future
 */
int RTOS_scheduleEvent(void (*function)(), uint16_t countdown);

/**
 * schedules a task to be performed in the referenced state
 */
int RTOS_scheduleTask(uint8_t state, void (*function)(), uint16_t period);

/**
 * In interrupt, calls events and queues tasks
 */
int RTOS_Update(void);

/**
 * In non-rt context, calls tasks that have been queued
 */
int RTOS_ExecuteTasks(void);

#endif
