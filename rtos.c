#include "rtos.h"

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
    if(rtos_scheduler.numberOfTasks == RTOS_maxStateNum) return -1;

    for(int i = 0; i < rtos_scheduler.numberOfTasks; i++){
        if (rtos_scheduler.tasks[i].callback == function &&
            rtos_scheduler.tasks[i].reset_ms == period)
        {
            rtos_scheduler.states[state].taskMask |= 1 << i;
            return i;
        }
    }

    rtos_scheduler.tasks[rtos_scheduler.numberOfTasks].callback = function;
    rtos_scheduler.tasks[rtos_scheduler.numberOfTasks].counter = 0;
    rtos_scheduler.tasks[rtos_scheduler.numberOfTasks].reset_ms = period;
    rtos_scheduler.states[state].taskMask |= 1 << rtos_scheduler.numberOfTasks;

    rtos_scheduler.numberOfTasks++;
    return rtos_scheduler.numberOfTasks - 1;
}

/**
 * schedules an event with a countdown, returns index in the linked list heap
 */
extern int RTOS_scheduleEvent(void (*function)(), uint16_t countdown){
    int pointer = 0;
    while(rtos_scheduler.eventHeap[pointer].callback == 0)
        pointer++;

    rtos_scheduler.eventHeap[pointer].callback = function;
    rtos_scheduler.eventHeap[pointer].countdown = countdown;
    
    int index = rtos_scheduler.firstEventIndex;
    int lastIndex = -1;    
    int firstevent = 1;
    while(rtos_scheduler.eventHeap[index].nextEvent != -1){
        if(rtos_scheduler.eventHeap[index].countdown > countdown) break;

        firstevent = 0;
        lastIndex = index;
        index = rtos_scheduler.eventHeap[index].nextEvent;
    }

    if(!firstevent){
        rtos_scheduler.eventHeap[lastIndex].nextEvent = pointer; 
    }
    else rtos_scheduler.firstEventIndex = pointer;

    if(rtos_scheduler.eventHeap[lastIndex].nextEvent != -1)
        rtos_scheduler.eventHeap[pointer].nextEvent = index;

    return pointer;
}

/**
 * returns 0 if no remaining tasks
 */
extern int RTOS_removeFirstEvent(){
    uint8_t newFirst = rtos_scheduler.eventHeap[rtos_scheduler.firstEventIndex].nextEvent;
    
    rtos_scheduler.eventHeap[rtos_scheduler.firstEventIndex].callback = 0;
    rtos_scheduler.eventHeap[rtos_scheduler.firstEventIndex].nextEvent = -1;
    
    rtos_scheduler.firstEventIndex = newFirst;

    return rtos_scheduler.firstEventIndex;
}

/**
 * called in ms interrupt context to schedule the tasks and execute the events
 */
extern int RTOS_Update(){
    for(int i = 0; i < rtos_scheduler.numberOfTasks; i++){
        if(!((rtos_scheduler.states[rtos_scheduler.state].taskMask >> i) & 1)) 
            continue;
        if(rtos_scheduler.tasks[i].counter <= 0){
            rtos_scheduler.taskQue |= 1 << i;
            rtos_scheduler.tasks[i].counter = rtos_scheduler.tasks[i].reset_ms;
        }
        rtos_scheduler.tasks[i].counter--;
    }

    int eventpointer = rtos_scheduler.firstEventIndex;
    while (eventpointer != -1){
        if (rtos_scheduler.eventHeap[eventpointer].countdown > 0)
            break; //move on if no events to execute
        
        rtos_scheduler.eventQue |= (1 << eventpointer);
        eventpointer = rtos_scheduler.eventHeap[eventpointer].nextEvent;
    }
    /* Two loops might seem weird but the lack of that if statement in a 16 full queue saves 
     * ~50-100 clock cycles between the branch prediction and the cmp instruction, and 
     * this function is as uncluttered as possible on purpose. */
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
        if (rtos_scheduler.taskQue >> i & 0x1){
            rtos_scheduler.tasks[i].callback();
            rtos_scheduler.taskQue &= ~(0x1 << i);
        }
    }

    for(int i = 0; i != -1;){
        if (rtos_scheduler.eventQue >> i & 0x1){
            rtos_scheduler.tasks[i].callback();
            i = rtos_scheduler.eventHeap[i].nextEvent;
            RTOS_removeFirstEvent();
        }
        else break;
    }
    
    return 0;
}
