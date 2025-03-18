#include "rtos.h"


extern int RTOS_init(){
    rtos_scheduler.firstEventIndex = -1;
    rtos_scheduler.numberOfStates = 0;
    rtos_scheduler.numberOfTasks = 0;
    rtos_scheduler.taskQue = 0;
    rtos_scheduler.eventQue = 0;
    rtos_scheduler.state = 0;
    

    for(int i = 0; i < RTOS_maxEventNum; i++){
        rtos_scheduler.eventHeap[i].nextEvent = -1;
        rtos_scheduler.eventHeap[i].callback = 0;
        rtos_scheduler.eventHeap[i].countdown = 0;
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
    }

    return 0;
}

/**
 * adds state to the rtos
 */
extern int RTOS_addState(void (*start), void (*stop)){
    if (rtos_scheduler.numberOfStates >= RTOS_maxStateNum) return -1;

    rtos_scheduler.states[rtos_scheduler.numberOfStates].entry = start;
    rtos_scheduler.states[rtos_scheduler.numberOfStates].exit = stop;
    rtos_scheduler.states[rtos_scheduler.numberOfStates].taskMask = 0;

    rtos_scheduler.numberOfStates++;

    return rtos_scheduler.numberOfStates - 1;
}

/**
 * Switches the current rtos state
 */
extern int RTOS_switchState(uint8_t state){
    if(rtos_scheduler.state == state) return state;

    if(rtos_scheduler.states[rtos_scheduler.state].exit != 0) 
        rtos_scheduler.states[rtos_scheduler.state].exit();

    if(rtos_scheduler.states[state].entry != 0)
        rtos_scheduler.states[state].entry();

    rtos_scheduler.state = state;
    return state;
}

extern int RTOS_inState(uint8_t state) { return state == rtos_scheduler.state; }

/**
 * Schedules a task to be performed on a state, making sure that duplicates are flagged 
 * returns the index of the task
 */
extern int RTOS_scheduleTask(uint8_t state, void (*function)(), uint16_t period){
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
    rtos_scheduler.states[state].taskMask |= 1 << rtos_scheduler.numberOfTasks;

    rtos_scheduler.numberOfTasks++;
    return rtos_scheduler.numberOfTasks - 1;
}

/**
 * schedules an event with a countdown, returns index in the linked list heap
 */
extern int RTOS_scheduleEvent(void (*function)(), uint16_t countdown){
    int newEventHandle = 0;
    while(rtos_scheduler.eventHeap[newEventHandle].callback != 0 && newEventHandle < RTOS_maxEventNum)
        newEventHandle++;
    
    if (newEventHandle == RTOS_maxEventNum) return -1;

    rtos_scheduler.eventHeap[newEventHandle].callback = function;
    rtos_scheduler.eventHeap[newEventHandle].countdown = countdown;
    rtos_scheduler.eventHeap[newEventHandle].nextEvent = -1;
    
    if(rtos_scheduler.firstEventIndex == -1){
        rtos_scheduler.firstEventIndex = newEventHandle;
        return newEventHandle;
    }

    int index = rtos_scheduler.firstEventIndex;

    while(rtos_scheduler.eventHeap[index].nextEvent != -1){
        if(rtos_scheduler.eventHeap[rtos_scheduler.eventHeap[index].nextEvent].countdown > countdown) break;

        index = rtos_scheduler.eventHeap[index].nextEvent;
    }

    rtos_scheduler.eventHeap[index].nextEvent = newEventHandle;
    rtos_scheduler.eventHeap[newEventHandle].nextEvent = rtos_scheduler.eventHeap[index].nextEvent;

    return newEventHandle;
}

/**
 * returns 0 if no remaining tasks
 */
extern int RTOS_removeFirstEvent(){
    uint8_t newFirst = rtos_scheduler.eventHeap[rtos_scheduler.firstEventIndex].nextEvent;
    
    rtos_scheduler.eventHeap[rtos_scheduler.firstEventIndex].callback = 0;
    rtos_scheduler.eventHeap[rtos_scheduler.firstEventIndex].nextEvent = -1;
    
    rtos_scheduler.firstEventIndex = newFirst;

    return rtos_scheduler.firstEventIndex == -1;
}

/**
 * called in ms interrupt context to schedule the tasks and execute the events
 */
extern int RTOS_Update(){
    for(int i = 0; i < rtos_scheduler.numberOfTasks; i++){ 
        rtos_scheduler.tasks[i].counter--;

        if((rtos_scheduler.states[rtos_scheduler.state].taskMask >> i) & 0x01) {
            if(rtos_scheduler.tasks[i].counter <= 0){
                rtos_scheduler.taskQue |= 1 << i;
                rtos_scheduler.tasks[i].counter = rtos_scheduler.tasks[i].reset_ms;
            }
        }
    }

    int eventpointer = rtos_scheduler.firstEventIndex;
    while (eventpointer != -1){
        if (rtos_scheduler.eventHeap[eventpointer].countdown > 0 || rtos_scheduler.eventHeap[eventpointer].callback == 0)
            break; //move on if no events to execute
        
        rtos_scheduler.eventQue |= (1 << eventpointer);
        eventpointer = rtos_scheduler.eventHeap[eventpointer].nextEvent;
    }
    while (eventpointer != -1){
        rtos_scheduler.eventHeap[eventpointer].countdown--;
        eventpointer = rtos_scheduler.eventHeap[eventpointer].nextEvent;
    }

    return 0;
}

/**
 * called in main context to execute all tasks and events
 */
extern int RTOS_ExecuteTasks(){
    for(int i = 0; i < rtos_scheduler.numberOfTasks; i++){
        if ((rtos_scheduler.taskQue >> i) & 0x1){
            rtos_scheduler.tasks[i].callback();
            rtos_scheduler.taskQue &= ~(0x1 << i);
        }
    }

    for(int i = 0; i < RTOS_maxEventNum; i++){
        if ((rtos_scheduler.eventQue >> i) & 0x1){
            rtos_scheduler.eventHeap[i].callback();
            rtos_scheduler.eventQue &= ~(0x1 << i);
            RTOS_removeFirstEvent();
        }
        else break;
    }
    
    return 0;
}
