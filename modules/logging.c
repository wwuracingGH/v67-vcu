#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "logging.h"
#include "stm32h5xx.h"

#define UPDATE_MS 4
#define PRINT_BUFFER_LEN 4096 /* 4kb buffer should be enough? */

struct __printbuffer {
    uint8_t buffer[PRINT_BUFFER_LEN];
    
    /* Index of where the putc function has last put a character */
    uint16_t append_head;
    
    /* Index of where the last character written to uart is */
    uint16_t put_head;
} _print_buffer = {{0}, 0, 0};

/* returns length of the unwritten portion of buffer */
uint32_t _buffer_len(){
    int l = _print_buffer.append_head - _print_buffer.put_head;
    if (l < 0) l += PRINT_BUFFER_LEN;
    return l;
}

/* put char into buffer */
extern void _putchar(char character) {
    _print_buffer.buffer[_print_buffer.append_head] = character;
    
    _print_buffer.append_head++;
    _print_buffer.append_head %= PRINT_BUFFER_LEN;
}

void logging_init() {
    /* Enable USART2 clock */
    RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos);
    /* small read just to check */
    volatile uint32_t dummy;
    dummy = RCC->APB1LENR;
    dummy = RCC->APB1LENR;
    (void)dummy;

    /* small read just to check */
    dummy = RCC->AHB1ENR;
    dummy = RCC->AHB1ENR;
    (void)dummy;

    /* Configure and enable USART2 */
    USART1->BRR = 0x087a; // 115200 baud @ 250 MHz APB1 clock and 16x oversampling
    USART1->CR3 |= USART_CR3_DMAT;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA2EN;

    GPDMA2_Channel5->CTR1 |= DMA_CTR1_SINC | DMA_CTR1_SAP; /* set size of data to transfer */
    GPDMA2_Channel5->CTR2 |= (22 << DMA_CTR2_REQSEL_Pos) | DMA_CTR2_PFREQ | DMA_CTR2_DREQ;
    
    GPDMA2_Channel5->CDAR = (uint32_t)(&(USART1->TDR)); /* sets destination of dma transfer */
   
    /* sets size of burst at 1 */
    GPDMA2_Channel5->CTR1 &= ~(63 << DMA_CTR1_DBL_1_Pos) | ~(63 << DMA_CTR1_SBL_1_Pos);

    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

void _flush() {
	if(!(USART1->ISR & USART_ISR_TC)) return;

    uint8_t* source = (uint8_t*)&_print_buffer.buffer[_print_buffer.put_head];
    int length = _print_buffer.append_head - _print_buffer.put_head;
    if(length < 0)
        length = PRINT_BUFFER_LEN - _print_buffer.put_head; /* make it just one dma transfer */

    GPDMA2_Channel5->CSAR = (uint32_t)source; /* sets source of dma transfer */

    /* sets size of dma transfer */
    GPDMA2_Channel5->CBR1 &= ~DMA_CBR1_BNDT_Msk;
    GPDMA2_Channel5->CBR1 |= length << DMA_CBR1_BNDT_Pos;
    
    GPDMA2_Channel5->CCR |= DMA_CCR_EN;

    _print_buffer.put_head += length;
    
    if (_print_buffer.put_head == PRINT_BUFFER_LEN)
        _print_buffer.put_head = 0;
}

void _update_logging() {
    if(_print_buffer.put_head != _print_buffer.append_head) _flush();
}
