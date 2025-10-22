#include <stdint.h>
#ifndef _RTOS_
#define _RTOS_

/* must be defined right under the header file inclusion */
#define _RTOS_IMPLEMENTATION_ kernel rtos_scheduler = {0}; 
#define RTOS_ALL_STATES     -1
#define RTOS_NOT_IN_TASK     127


#ifndef RTOS_mainTick
#define RTOS_mainTick 1000
#endif

#ifndef RTOS_subTick_pow
#define RTOS_subTick_pow 5
#endif

/*
 * not cross platform start macros
 */
#define RTOS_start_armeabi(CLKSPEED) SysTick_Config(CLKSPEED / ((1 << RTOS_subTick_pow) * RTOS_mainTick)); __enable_irq();

/*
 * Some defaults, but the user can define their own
 */
#ifndef RTOS_maxEventNum
#define RTOS_maxEventNum 16
typedef uint16_t eventQue_t;
#else
#if (RTOS_maxEventNum <= 64 && RTOS_maxEventNum > 32)
typedef uint64_t eventQue_t;
#elif (RTOS_maxEventNum <= 32 && RTOS_maxEventNum > 16)
typedef uint32_t eventQue_t;
#elif (RTOS_maxEventNum <= 16 && RTOS_maxEventNum > 8)
typedef uint16_t eventQue_t;
#elif (RTOS_maxEventNum <= 8 && RTOS_maxEventNum >= 0)
typedef uint8_t eventQue_t;
#elif (RTOS_maxEventNum > 64 || RTOS_maxEventNum < 0)
#error Max events too large!
#endif
#endif

#ifndef RTOS_maxTaskNum
#define RTOS_maxTaskNum  64
typedef uint64_t taskQue_t;
#else
#if (RTOS_maxTaskNum <= 64 && RTOS_maxTaskNum > 32)
typedef uint64_t taskQue_t;
#elif (RTOS_maxTaskNum <= 32 && RTOS_maxTaskNum > 16)
typedef uint32_t taskQue_t;
#elif (RTOS_maxTaskNum <= 16 && RTOS_maxTaskNum > 8)
typedef uint16_t taskQue_t;
#elif (RTOS_maxTaskNum <= 8 && RTOS_maxTaskNum >= 0)
typedef uint8_t taskQue_t;
#elif (RTOS_maxTaskNum > 64 || RTOS_maxTaskNum < 0)
#error Max tasks too large!
#endif
#endif

#ifndef RTOS_maxStateNum
#define RTOS_maxStateNum 16
#endif

typedef struct {
    void (*callback)();
    uint32_t timestamp;
} rtos_event;

typedef struct {
    void (*callback)();
    uint16_t reset_ms;
    uint16_t counter;
    
    uint32_t lastStartTick;
    uint16_t lastTime;
    uint16_t priority; 
} rtos_task;

typedef struct {
    void (*entry)();
    void (*exit)();
    taskQue_t taskMask;
} rtos_state;

typedef struct {
    /* Timestamp in subticks */
    volatile uint32_t timestamp;
    
    volatile taskQue_t taskQue;
    
    volatile uint8_t currentTask    :  7;
    volatile uint8_t executing      :  1;
    
    uint8_t numberOfTasks;
    uint8_t numberOfEvents;
    uint8_t numberOfStates;
    uint8_t state;

    rtos_event eventHeap[RTOS_maxEventNum];
    rtos_task tasks[RTOS_maxTaskNum];
    rtos_state states[RTOS_maxStateNum];
} kernel;

extern kernel rtos_scheduler;
/**
 *  Inits and zeros the RTOS, just in case
 */
int RTOS_init(void);

/**
 * Returns main tick part of the tick var 
 */
uint32_t RTOS_getMainTick(void);

/**
 * Returns main tick part of the tick var 
 */
uint32_t RTOS_getSubTick(void);


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
int RTOS_scheduleTask(int state, void (*function)(), uint16_t period);

/**
 * In interrupt, calls events and queues tasks
 */
int RTOS_Update(void);

/**
 * In non-rt context, calls tasks that have been queued
 */
int RTOS_ExecuteTasks(void);


/**
 *
 */
rtos_task* RTOS_currentTask(){
    if(rtos_scheduler.currentTask == RTOS_NOT_IN_TASK) return NULL;

    return &rtos_scheduler.tasks[rtos_scheduler.currentTask];
}

inline int RTOS_init(){
    rtos_scheduler.timestamp = 0;
    rtos_scheduler.executing = 0;
    rtos_scheduler.currentTask = RTOS_NOT_IN_TASK;

    rtos_scheduler.numberOfStates = 0;
    rtos_scheduler.numberOfTasks = 0;
    rtos_scheduler.numberOfEvents = 0;
    rtos_scheduler.taskQue = 0;
    rtos_scheduler.state = 0;
    

    for(int i = 0; i < RTOS_maxEventNum; i++){
        rtos_scheduler.eventHeap[i].callback = 0;
        rtos_scheduler.eventHeap[i].timestamp = 0;
    }

    for(int i = 0; i < RTOS_maxStateNum; i++){
        rtos_scheduler.states[i].entry = 0;
        rtos_scheduler.states[i].exit = 0;
        rtos_scheduler.states[i].taskMask = 0;
        
    }

    for (int i = 0; i < RTOS_maxTaskNum; i++){
        rtos_scheduler.tasks[i].callback = 0;
        rtos_scheduler.tasks[i].counter = 0;
        rtos_scheduler.tasks[i].reset_ms = 0;
        rtos_scheduler.tasks[i].lastStartTick = 0;
        rtos_scheduler.tasks[i].lastTime = 0;
    }

    return 0;
}

/**
 * Returns main tick part of the tick var 
 */
uint32_t RTOS_getMainTick(void){ return rtos_scheduler.timestamp >> RTOS_subTick_pow; }

/**
 * Returns main tick part of the tick var 
 */
uint32_t RTOS_getSubTick(void) { return rtos_scheduler.timestamp & (1 << RTOS_subTick_pow) - 1; }


/**
 * adds state to the rtos
 */
inline int RTOS_addState(void (*start), void (*stop)){
    if (rtos_scheduler.numberOfStates >= RTOS_maxStateNum) return -1;

    rtos_scheduler.states[rtos_scheduler.numberOfStates].entry = start;
    rtos_scheduler.states[rtos_scheduler.numberOfStates].exit = stop;

    rtos_scheduler.numberOfStates++;

    return rtos_scheduler.numberOfStates - 1;
}

/**
 * Switches the current rtos state
 */
inline int RTOS_switchState(uint8_t state){
    if(rtos_scheduler.state == state) return state;

    if(rtos_scheduler.states[rtos_scheduler.state].exit != 0) 
        rtos_scheduler.states[rtos_scheduler.state].exit();

    if(rtos_scheduler.states[state].entry != 0)
        rtos_scheduler.states[state].entry();

    rtos_scheduler.state = state;
    return state;
}

inline int RTOS_inState(uint8_t state) { return state == rtos_scheduler.state; }

/**
 * Schedules a task to be performed on a state, making sure that duplicates are flagged 
 * returns the index of the task
 */
inline int RTOS_scheduleTask(int state, void (*function)(), uint16_t period){
    if(rtos_scheduler.numberOfTasks == RTOS_maxTaskNum) return -1;

    for(int i = 0; i < rtos_scheduler.numberOfTasks; i++){
        if (rtos_scheduler.tasks[i].callback == function &&
            rtos_scheduler.tasks[i].reset_ms == period)
        {
            rtos_scheduler.states[state].taskMask |= 1 << i;
            return i;
        }
    }

    rtos_scheduler.tasks[rtos_scheduler.numberOfTasks].callback = function;
    rtos_scheduler.tasks[rtos_scheduler.numberOfTasks].counter = period;
    rtos_scheduler.tasks[rtos_scheduler.numberOfTasks].reset_ms = period;
    
    if (state == RTOS_ALL_STATES) {
        for(int i = 0; i < RTOS_maxStateNum; i++){
            rtos_scheduler.states[i].taskMask |= 1 << rtos_scheduler.numberOfTasks;
        } 
    }
    else {
        rtos_scheduler.states[state].taskMask |= 1 << rtos_scheduler.numberOfTasks;
    }

    rtos_scheduler.numberOfTasks++;
    return rtos_scheduler.numberOfTasks - 1;
}

/**
 * schedules an event with a countdown, returns index in the heap
 */
inline int RTOS_scheduleEvent(void (*function)(), uint16_t countdown){
    int ti = rtos_scheduler.numberOfEvents;
    int parent = (ti - 1) >> 1;
    
    while(parent >= 0 && rtos_scheduler.eventHeap[parent].timestamp > rtos_scheduler.eventHeap[ti].timestamp){
        rtos_scheduler.eventHeap[ti].timestamp = rtos_scheduler.eventHeap[parent].timestamp;
        rtos_scheduler.eventHeap[ti].callback = rtos_scheduler.eventHeap[parent].callback;
        ti = parent;
        parent = (ti - 1) >> 1;
    }

    rtos_scheduler.eventHeap[ti].callback  = function;
    rtos_scheduler.eventHeap[ti].timestamp = rtos_scheduler.timestamp + (countdown << RTOS_subTick_pow);

    rtos_scheduler.numberOfEvents++;
    return ti;
}

int smallerChild(int index, int len){
    uint32_t tsi = rtos_scheduler.eventHeap[index].timestamp;
    uint32_t least, tsl, tsr;
    if      (((index << 1) + 1) >= len) return -1;
    else if (((index << 1) + 2) >= len) least = (index << 1) + 1; 
    else {
        tsl = rtos_scheduler.eventHeap[(index << 1) + 1].timestamp;
        tsr = rtos_scheduler.eventHeap[(index << 1) + 2].timestamp;
        least = tsl <= tsr ? (index << 1) + 1 : (index << 1) + 2;
    }
    return rtos_scheduler.eventHeap[least].timestamp < tsi ? least : -1;
}

/**
 * returns 0 if no remaining tasks
 */
inline int RTOS_removeFirstEvent(){
    int new_size = rtos_scheduler.numberOfEvents - 1;
    if(new_size == 0) { 
        rtos_scheduler.eventHeap[0].callback = 0;
        rtos_scheduler.eventHeap[0].timestamp = 0;
        return 0;
    }
    
    void (*callback)() = rtos_scheduler.eventHeap[new_size].callback;
    uint32_t timestamp = rtos_scheduler.eventHeap[new_size].timestamp;
    
    rtos_scheduler.eventHeap[0].callback = callback;
    rtos_scheduler.eventHeap[0].timestamp = timestamp;
    
    rtos_scheduler.eventHeap[new_size].callback = 0;
    rtos_scheduler.eventHeap[new_size].timestamp = 0;
    
    uint32_t i = 0, smc = 0;
    while((smc = smallerChild(i, new_size)) != -1) {
        rtos_scheduler.eventHeap[i].callback = rtos_scheduler.eventHeap[smc].callback;
        rtos_scheduler.eventHeap[i].timestamp = rtos_scheduler.eventHeap[smc].timestamp;
        i = smc;
    }
    
    rtos_scheduler.eventHeap[i].callback = callback;
    rtos_scheduler.eventHeap[i].timestamp = timestamp;
    
    return new_size;
}

/**
 * called in ms interrupt context to schedule the tasks and execute the events
 */
inline int RTOS_Update(){
    if(RTOS_getSubTick()){ 
        rtos_scheduler.timestamp++;
        return 1;
    }
    rtos_scheduler.timestamp++;
    
    for(int i = 0; i < rtos_scheduler.numberOfTasks; i++){ 
        if((rtos_scheduler.states[rtos_scheduler.state].taskMask >> i) & 0x01) {
            rtos_scheduler.tasks[i].counter--;
            if(rtos_scheduler.tasks[i].counter <= 0){
                rtos_scheduler.taskQue |= 1 << i;
                rtos_scheduler.tasks[i].counter = rtos_scheduler.tasks[i].reset_ms;
            }
        }
    }

    return 0;
}

/**
 * called in main context to execute all tasks and events
 */
inline int RTOS_ExecuteTasks(){
    rtos_scheduler.executing = 1;
    
    for(int i = 0; i < rtos_scheduler.numberOfTasks; i++){
        if ((rtos_scheduler.taskQue >> i) & 0x1){
            rtos_scheduler.currentTask = i;
            uint32_t startTick = rtos_scheduler.timestamp; 
            rtos_scheduler.tasks[i].callback();
            rtos_scheduler.taskQue &= ~(0x1 << i);
            rtos_scheduler.currentTask = RTOS_NOT_IN_TASK;
            rtos_scheduler.tasks[i].lastStartTick = startTick; 
            rtos_scheduler.tasks[i].lastTime = rtos_scheduler.timestamp - startTick; 
        }
    }

    if(rtos_scheduler.numberOfEvents > 0){
        if (rtos_scheduler.eventHeap[0].timestamp < rtos_scheduler.timestamp && rtos_scheduler.eventHeap[0].callback != 0){
            rtos_scheduler.eventHeap[0].callback();
            rtos_scheduler.numberOfEvents = RTOS_removeFirstEvent();
        }
    }
    rtos_scheduler.executing = 0;
    return 0;
}

#endif