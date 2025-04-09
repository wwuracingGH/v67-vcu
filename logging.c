#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"
#include "llrttsos.h"

#define UPDATE_MS 4
#define PRINT_BUFFER_LEN 4096 /* 4kb buffer should be enough? */
#include <stdint.h>

struct __printbuffer {
    char buffer[PRINT_BUFFER_LEN];
    
    /* Index of where the putc function has last put a character */
    uint16_t append_head;
    
    /* Index of where the last character written to uart is */
    uint16_t put_head;
} _print_buffer = {{0}, 0, 0};

/* returns length of the unwritten portion of buffer */
uint32_t _buffer_len(){
    return ((_print_buffer.append_head - _print_buffer.put_head) + PRINT_BUFFER_LEN) % PRINT_BUFFER_LEN;
}

/* put char into buffer */
void _putchar(char character) {
    _print_buffer.append_head++;
    
    /* faster than mod i think*/
    if(_print_buffer.append_head == PRINT_BUFFER_LEN) 
        _print_buffer.append_head = 0;
    
    _print_buffer.buffer[_print_buffer.append_head] = character;
}

void _init_logging() {
    /* TODO: start uart functionality, idealy with a dma */ 
}

void _flush() {
    uint8_t* source = (uint8_t*)&_print_buffer.buffer[_print_buffer.put_head];
    int length = _buffer_len();
    if(_print_buffer.append_head < _print_buffer.put_head) 
        length = PRINT_BUFFER_LEN - _print_buffer.put_head;

    /* TODO: start the dma burst or manually log all characters (i think you can do this?) */

    /* Some rough ideas for what the register settings should look like:
     *
     * DMA source = source
     * DMA size = 8 bit
     * DMA dest = uart
     * DMA length = length  
     */

    _print_buffer.put_head += length;
    
    if (_print_buffer.put_head == PRINT_BUFFER_LEN)
        _print_buffer.put_head = 0;
}

void _updateLogging() {
    if(_buffer_len() > 0) _flush();
}

/* enables logging on an rtos state */
void _enable_logging(uint8_t state){
    RTOS_scheduleTask(state, _updateLogging, UPDATE_MS);
}

void _enable_logging_all(){
    RTOS_scheduleTask(RTOS_ALL_STATES, _updateLogging, UPDATE_MS);
}
