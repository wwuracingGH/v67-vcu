//this is not my code
//i will rewrite it at some point
#include <stdint.h>

#define SRAM_START (0x20000000U)
#define SRAM_SIZE (6U * 1024U)
#define SRAM_END (SRAM_START + SRAM_SIZE)
#define STACK_POINTER_INIT_ADDRESS (SRAM_END)
#define ISR_VECTOR_SIZE_WORDS 48

void reset_handler(void);
void default_handler(void);

// Cortex-M system exceptions
void nmi_handler(void) __attribute__((weak, alias("default_handler")));
void hard_fault_handler(void) __attribute__((weak, alias("default_handler")));
void svcall_handler(void) __attribute__((weak, alias("default_handler")));
void debug_monitor_handler(void) __attribute__((weak, alias("default_handler")));
void pendsv_handler(void) __attribute__((weak, alias("default_handler")));
void systick_handler(void) __attribute__((weak, alias("default_handler")));

// STM32F410RB interrupt handlers
void wwdg_handler(void) __attribute__((weak, alias("default_handler")));
void pvd_handler(void) __attribute__((weak, alias("default_handler")));
void rtc_handler(void) __attribute__((weak, alias("default_handler")));
void flash_handler(void) __attribute__((weak, alias("default_handler")));
void rcc_crs_handler(void) __attribute__((weak, alias("default_handler")));
void exti0_1_handler(void) __attribute__((weak, alias("default_handler")));
void exti2_3_handler(void) __attribute__((weak, alias("default_handler")));
void exti4_F_handler(void) __attribute__((weak, alias("default_handler")));
void tsc_handler(void) __attribute__((weak, alias("default_handler")));
void dma_ch1_handler(void) __attribute__((weak, alias("default_handler")));
void dma_ch2_3_handler(void) __attribute__((weak, alias("default_handler")));
void dma_ch4_7_handler(void) __attribute__((weak, alias("default_handler")));
void adc_comp_handler(void) __attribute__((weak, alias("default_handler")));
void tim1_butc_handler(void) __attribute__((weak, alias("default_handler")));
void tim1_cc_handler(void) __attribute__((weak, alias("default_handler")));
void tim2_handler(void) __attribute__((weak, alias("default_handler")));
void tim3_handler(void) __attribute__((weak, alias("default_handler")));
void tim6_DAC_handler(void) __attribute__((weak, alias("default_handler")));
void tim7_handler(void) __attribute__((weak, alias("default_handler")));
void tim14_handler(void) __attribute__((weak, alias("default_handler")));
void tim15_handler(void) __attribute__((weak, alias("default_handler")));
void tim16_handler(void) __attribute__((weak, alias("default_handler")));
void tim17_handler(void) __attribute__((weak, alias("default_handler")));
void i2c1_handler(void) __attribute__((weak, alias("default_handler")));
void i2c2_handler(void) __attribute__((weak, alias("default_handler")));
void spi1_handler(void) __attribute__((weak, alias("default_handler")));
void spi2_handler(void) __attribute__((weak, alias("default_handler")));
void usart1_handler(void) __attribute__((weak, alias("default_handler")));
void usart2_handler(void) __attribute__((weak, alias("default_handler")));
void usart3_8_handler(void) __attribute__((weak, alias("default_handler")));
void cec_can_handler(void) __attribute__((weak, alias("default_handler")));
void usb_handler(void) __attribute__((weak, alias("default_handler")));

uint32_t isr_vector[ISR_VECTOR_SIZE_WORDS] __attribute__((section(".isr_vector"))) = {
    STACK_POINTER_INIT_ADDRESS,         /* 0x00 */
    // Cortex-M system exceptions
    (uint32_t)&reset_handler,           /* 0x04 */
    (uint32_t)&nmi_handler,             /* 0x08 */
    (uint32_t)&hard_fault_handler,      /* 0x0C */
    0,                                  /* 0x10 */
    0,                                  /* 0x14 */
    0,                                  /* 0x18 */
    0,                                  /* 0x1C */
    0,                                  /* 0x20 */
    0,                                  /* 0x24 */
    0,                                  /* 0x28 */
    (uint32_t)&svcall_handler,          /* 0x2C */
    0,                                  /* 0x30 */
    0,                                  /* 0x34 */
    (uint32_t)&pendsv_handler,          /* 0x38 */
    (uint32_t)&systick_handler,         /* 0x3C */
    // STM32F410 interrupt handlers
    (uint32_t)&wwdg_handler,            /* 0x40 */
    (uint32_t)&pvd_handler,             /* 0x44 */
    (uint32_t)&rtc_handler,             /* 0x48 */
    (uint32_t)&flash_handler,           /* 0x4C */
    (uint32_t)&rcc_crs_handler,         /* 0x50 */
    (uint32_t)&exti0_1_handler,         /* 0x54 */
    (uint32_t)&exti2_3_handler,         /* 0x58 */
    (uint32_t)&exti4_F_handler,         /* 0x5C */
    (uint32_t)&tsc_handler,             /* 0x60 */
    (uint32_t)&dma_ch1_handler,         /* 0x64 */
    (uint32_t)&dma_ch2_3_handler,       /* 0x68 */
    (uint32_t)&dma_ch4_7_handler,       /* 0x6C */
    (uint32_t)&adc_comp_handler,        /* 0x70 */
    (uint32_t)&tim1_butc_handler,       /* 0x74 */
    (uint32_t)&tim1_cc_handler,         /* 0x78 */
    (uint32_t)&tim2_handler,            /* 0x7C */
    (uint32_t)&tim3_handler,            /* 0x80 */
    (uint32_t)&tim6_DAC_handler,        /* 0x84 */
    (uint32_t)&tim7_handler,            /* 0x88 */
    (uint32_t)&tim14_handler,           /* 0x8C */
    (uint32_t)&tim15_handler,           /* 0x90 */
    (uint32_t)&tim16_handler,           /* 0x94 */
    (uint32_t)&tim17_handler,           /* 0x98 */
    (uint32_t)&i2c1_handler,            /* 0x9C */
    (uint32_t)&i2c2_handler,            /* 0xA0 */
    (uint32_t)&spi1_handler,            /* 0xA4 */
    (uint32_t)&spi2_handler,            /* 0xA8 */
    (uint32_t)&usart1_handler,          /* 0xAC */
    (uint32_t)&usart2_handler,          /* 0xB0 */
    (uint32_t)&usart3_8_handler,        /* 0xB4 */
    (uint32_t)&cec_can_handler,         /* 0xB8 */
    (uint32_t)&usb_handler,             /* 0xBC */
};

extern uint32_t _etext, _sdata, _edata, _sbss, _ebss, _sidata;
void main(void);
void kill_car();

void reset_handler(void)
{
    // Copy .data from FLASH to SRAM
    uint32_t data_size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t *flash_data = (uint8_t*) &_sidata; // Data load address (in flash)
    uint8_t *sram_data = (uint8_t*) &_sdata; // Data virtual address (in sram)
    
    for (uint32_t i = 0; i < data_size; i++)
    {
        sram_data[i] = flash_data[i];
    }

    // Zero-fill .bss section in SRAM
    uint32_t bss_size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    uint8_t *bss = (uint8_t*) &_sbss;

    for (uint32_t i = 0; i < bss_size; i++)
    {
        bss[i] = 0;
    }
    
    main();
}

void default_handler(void)
{
    while(1);
}

