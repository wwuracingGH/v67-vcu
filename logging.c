#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "logging.h"
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"

#define UPDATE_MS 4
#define PRINT_BUFFER_LEN 4096 /* 4kb buffer should be enough? */

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
extern void _putchar(char character) {
    _print_buffer.append_head++;
    
    /* faster than mod i think*/
    if(_print_buffer.append_head == PRINT_BUFFER_LEN) 
        _print_buffer.append_head = 0;
    
    _print_buffer.buffer[_print_buffer.append_head] = character;
}

void _init_logging() {
    /* Enable USART2 clock */
    RCC->APB1LENR |= (1 << RCC_APB1LENR_USART2EN_Pos);
    // do two dummy reads after enabling the peripheral clock, as per the errata
    volatile uint32_t dummy;
    dummy = RCC->APB1LENR;
    dummy = RCC->APB1LENR;

    /* Enable GPIOA clock*/
    RCC->AHB1ENR |= (1 << RCC_AHB2ENR_GPIOAEN);
    // do two dummy reads after enabling the peripheral clock, as per the errata
    dummy = RCC->AHB1ENR;
    dummy = RCC->AHB1ENR;

    /* Set PA2 and PA3 to alternate function */
    GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODE2_Pos) | (0b10 << GPIO_MODER_MODE3_Pos);

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2_3 | GPIO_AFRL_AFSEL2_3);
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);

    /* Configure and enable USART2 */
    USART2->BRR = 0x087a; // 115200 baud @ 250 MHz APB1 clock and 16x oversampling
    USART2->CR3 |= USART_CR3_DMAT;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA2EN;

    GPDMA2_Channel5->CTR1 |= DMA_CTR1_SINC | DMA_CTR1_SAP; /* set size of data to transfer */
    GPDMA2_Channel5->CTR2 |= 0b01 << DMA_CTR2_TRIGPOL_Pos | 24 << DMA_CTR2_REQSEL_Pos 
                            | DMA_CTR2_DREQ | (0b11 << DMA_CTR2_TRIGM_Pos);
    
    GPDMA2_Channel5->CDAR = (uint32_t)(&(USART2->TDR)); /* sets destination of dma transfer */
   
    /* sets size of burst at 1 */
    GPDMA2_Channel5->CTR1 &= ~(63 << DMA_CTR1_DBL_1_Pos) | ~(63 << DMA_CTR1_SBL_1_Pos);

    GPDMA2_Channel5->CTR3 |= 1 << DMA_CTR3_SAO_Pos;

    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

void _flush() {
    uint8_t* source = (uint8_t*)&_print_buffer.buffer[_print_buffer.put_head];
    int length = _buffer_len();
    if(_print_buffer.append_head < _print_buffer.put_head) 
        length = PRINT_BUFFER_LEN - _print_buffer.put_head;

    GPDMA2_Channel5->CSAR = (uint32_t)source; /* sets source of dma transfer */

    /* sets size of dma transfer */
    GPDMA2_Channel5->CBR1 |= length << DMA_CBR1_BNDT_Pos;
    
    GPDMA2_Channel5->CCR |= DMA_CCR_EN;
    USART2->ICR |= 0b1111111111;

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
    RTOS_scheduleTask(-1, _updateLogging, UPDATE_MS);
}
