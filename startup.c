/*
 * Nicole Swierstra, Renee Anderson
 *
 * Boilerplate NVIC and memory initialization
 * 
 * Might include RAM execution in the future but the H533's bus isn't really setup for that
 * Should at least include Instruction Cacheing tho
 * 
 * Based on STM32 Without CubeIDE's startup.c
 */
#include <stdint.h>
#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h533xx.h"

#define SRAM_START (0x20000000U)
#define SRAM_SIZE (272UL * 1024UL)
#define SRAM_END (SRAM_START + SRAM_SIZE)
#define STACK_POINTER_INIT_ADDRESS (SRAM_END)
#define ISR_VECTOR_SIZE_WORDS 150

void reset_handler(void);
void default_handler(void);

/* Cortex-M system exceptions */
void nmi_handler(void) __attribute__((weak, alias("default_handler")));
void hard_fault_handler(void) __attribute__((weak, alias("default_handler")));
void mem_manage(void) __attribute__((weak, alias("default_handler")));
void bus_fault(void) __attribute__((weak, alias("default_handler")));
void usage_fault(void) __attribute__((weak, alias("default_handler")));
void secure_fault(void) __attribute__((weak, alias("default_handler")));
void svcall_handler(void) __attribute__((weak, alias("default_handler")));
void dbg_monitor_handler(void) __attribute__((weak, alias("default_handler")));
void pendsv_handler(void) __attribute__((weak, alias("default_handler")));
void systick_handler(void) __attribute__((weak, alias("default_handler")));

/* STM32H533 Interrupts */
void wwdg_handler(void) __attribute__((weak, alias("default_handler")));
void pvd_handler(void) __attribute__((weak, alias("default_handler")));
void rtc_handler(void) __attribute__((weak, alias("default_handler")));
void rtc_s_handler(void) __attribute__((weak, alias("default_handler")));
void tamper_handler(void) __attribute__((weak, alias("default_handler")));
void ramcfg_handler(void) __attribute__((weak, alias("default_handler")));
void flash_handler(void) __attribute__((weak, alias("default_handler")));
void gtcz_handler(void) __attribute__((weak, alias("default_handler")));
void rcc_handler(void) __attribute__((weak, alias("default_handler")));
void exti0_handler(void) __attribute__((weak, alias("default_handler")));
void exti1_handler(void) __attribute__((weak, alias("default_handler")));
void exti2_handler(void) __attribute__((weak, alias("default_handler")));
void exti3_handler(void) __attribute__((weak, alias("default_handler")));
void exti4_handler(void) __attribute__((weak, alias("default_handler")));
void exti5_handler(void) __attribute__((weak, alias("default_handler")));
void exti6_handler(void) __attribute__((weak, alias("default_handler")));
void exti7_handler(void) __attribute__((weak, alias("default_handler")));
void exti8_handler(void) __attribute__((weak, alias("default_handler")));
void exti9_handler(void) __attribute__((weak, alias("default_handler")));
void exti10_handler(void) __attribute__((weak, alias("default_handler")));
void exti11_handler(void) __attribute__((weak, alias("default_handler")));
void exti12_handler(void) __attribute__((weak, alias("default_handler")));
void exti13_handler(void) __attribute__((weak, alias("default_handler")));
void exti14_handler(void) __attribute__((weak, alias("default_handler")));
void exti15_handler(void) __attribute__((weak, alias("default_handler")));
void tsc_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch0_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch1_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch2_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch3_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch4_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch5_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch6_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma1_ch7_handler(void) __attribute__((weak, alias("default_handler")));
void iwdg_handler(void) __attribute__((weak, alias("default_handler")));
void adc1_handler(void) __attribute__((weak, alias("default_handler")));
void dac1_handler(void) __attribute__((weak, alias("default_handler")));
void fdcan_it0_handler(void) __attribute__((weak, alias("default_handler")));
void fdcan_it1_handler(void) __attribute__((weak, alias("default_handler")));
void tim1_brk_handler(void) __attribute__((weak, alias("default_handler")));
void tim1_up_handler(void) __attribute__((weak, alias("default_handler")));
void tim1_trg_com_handler(void) __attribute__((weak, alias("default_handler")));
void tim1_cc_handler(void) __attribute__((weak, alias("default_handler")));
void tim2_handler(void) __attribute__((weak, alias("default_handler")));
void tim3_handler(void) __attribute__((weak, alias("default_handler")));
void tim4_handler(void) __attribute__((weak, alias("default_handler")));
void tim5_handler(void) __attribute__((weak, alias("default_handler")));
void tim6_handler(void) __attribute__((weak, alias("default_handler")));
void tim7_handler(void) __attribute__((weak, alias("default_handler")));
void i2c1_ev_handler(void) __attribute__((weak, alias("default_handler")));
void i2c1_er_handler(void) __attribute__((weak, alias("default_handler")));
void i2c2_ev_handler(void) __attribute__((weak, alias("default_handler")));
void i2c2_er_handler(void) __attribute__((weak, alias("default_handler")));
void spi1_handler(void) __attribute__((weak, alias("default_handler")));
void spi2_handler(void) __attribute__((weak, alias("default_handler")));
void spi3_handler(void) __attribute__((weak, alias("default_handler")));
void usart1_handler(void) __attribute__((weak, alias("default_handler")));
void usart2_handler(void) __attribute__((weak, alias("default_handler")));
void usart3_handler(void) __attribute__((weak, alias("default_handler")));
void uart4_handler(void) __attribute__((weak, alias("default_handler")));
void uart5_handler(void) __attribute__((weak, alias("default_handler")));
void lpuart1_handler(void) __attribute__((weak, alias("default_handler")));
void lptim1_handler(void) __attribute__((weak, alias("default_handler")));
void tim8_brk_handler(void) __attribute__((weak, alias("default_handler")));
void tim8_up_handler(void) __attribute__((weak, alias("default_handler")));
void tim8_trg_com_handler(void) __attribute__((weak, alias("default_handler")));
void tim8_cc_handler(void) __attribute__((weak, alias("default_handler")));
void adc2_handler(void) __attribute__((weak, alias("default_handler")));
void lptim2_handler(void) __attribute__((weak, alias("default_handler")));
void tim15_handler(void) __attribute__((weak, alias("default_handler")));
void usb_fs_handler(void) __attribute__((weak, alias("default_handler")));
void crs_handler(void) __attribute__((weak, alias("default_handler")));
void ucpd1_handler(void) __attribute__((weak, alias("default_handler")));
void fmc_handler(void) __attribute__((weak, alias("default_handler")));
void octospi1_handler(void) __attribute__((weak, alias("default_handler")));
void sdmmc1_handler(void) __attribute__((weak, alias("default_handler")));
void i2c3_ev_handler(void) __attribute__((weak, alias("default_handler")));
void i2c3_er_handler(void) __attribute__((weak, alias("default_handler")));
void spi4_handler(void) __attribute__((weak, alias("default_handler")));
void usart6_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch0_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch1_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch2_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch3_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch4_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch5_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch6_handler(void) __attribute__((weak, alias("default_handler")));
void gpdma2_ch7_handler(void) __attribute__((weak, alias("default_handler")));
void fpu_handler(void) __attribute__((weak, alias("default_handler")));
void icache_handler(void) __attribute__((weak, alias("default_handler")));
void dcache_handler(void) __attribute__((weak, alias("default_handler")));
void dcmi_pssi_handler(void) __attribute__((weak, alias("default_handler")));
void fdcan2_it0_handler(void) __attribute__((weak, alias("default_handler")));
void fdcan2_it1_handler(void) __attribute__((weak, alias("default_handler")));
void dts_handler(void) __attribute__((weak, alias("default_handler")));
void rng_handler(void) __attribute__((weak, alias("default_handler")));
void otfdec1_handler(void) __attribute__((weak, alias("default_handler")));
void aes_handler(void) __attribute__((weak, alias("default_handler")));
void hash_handler(void) __attribute__((weak, alias("default_handler")));
void pka_handler(void) __attribute__((weak, alias("default_handler")));
void cec_handler(void) __attribute__((weak, alias("default_handler")));
void tim12_handler(void) __attribute__((weak, alias("default_handler")));
void i3c1_ev(void) __attribute__((weak, alias("default_handler")));
void i3c1_er(void) __attribute__((weak, alias("default_handler")));
void i3c2_ev(void) __attribute__((weak, alias("default_handler")));
void i3c2_er(void) __attribute__((weak, alias("default_handler")));

uint32_t isr_vector[ISR_VECTOR_SIZE_WORDS] __attribute__((section(".isr_vector"))) = {
    STACK_POINTER_INIT_ADDRESS,         /* 0x00 */
    /* Cortex-M system exceptions */
    (uint32_t)&reset_handler,           /* 0x04 */
    (uint32_t)&nmi_handler,             /* 0x08 */
    (uint32_t)&hard_fault_handler,      /* 0x0C */
    (uint32_t)&mem_manage,              /* 0x10 */
    (uint32_t)&bus_fault,               /* 0x14 */
    (uint32_t)&usage_fault,             /* 0x18 */
    (uint32_t)&secure_fault,            /* 0x1C */
    0,                                  /* 0x20 */
    0,                                  /* 0x24 */
    0,                                  /* 0x28 */
    (uint32_t)&svcall_handler,          /* 0x2C */
    (uint32_t)&dbg_monitor_handler,     /* 0x30 */
    0,                                  /* 0x34 */
    (uint32_t)&pendsv_handler,          /* 0x38 */
    (uint32_t)&systick_handler,         /* 0x3C */
    /* STM32H533 interrupt handlers */
    (uint32_t)&wwdg_handler,            /* 0x40 */
    (uint32_t)&pvd_handler,             /* 0x44 */
    (uint32_t)&rtc_handler,             /* 0x48 */
    (uint32_t)&rtc_s_handler,           /* 0x4C */
    (uint32_t)&tamper_handler,          /* 0x50 */
    (uint32_t)&ramcfg_handler,          /* 0x54 */
    (uint32_t)&flash_handler,           /* 0x58 */
    0,                                  /* 0x5C */
    (uint32_t)&gtcz_handler,            /* 0x60 */
    (uint32_t)&rcc_handler,             /* 0x64 */
    0,                                  /* 0x68 */
    (uint32_t)&exti0_handler,           /* 0x6C */
    (uint32_t)&exti1_handler,           /* 0x70 */
    (uint32_t)&exti2_handler,           /* 0x74 */
    (uint32_t)&exti3_handler,           /* 0x78 */
    (uint32_t)&exti4_handler,           /* 0x7C */
    (uint32_t)&exti5_handler,           /* 0x80 */
    (uint32_t)&exti6_handler,           /* 0x84 */
    (uint32_t)&exti7_handler,           /* 0x88 */
    (uint32_t)&exti8_handler,           /* 0x8C */
    (uint32_t)&exti9_handler,           /* 0x90 */
    (uint32_t)&exti10_handler,          /* 0x94 */
    (uint32_t)&exti11_handler,          /* 0x98 */
    (uint32_t)&exti12_handler,          /* 0x9C */
    (uint32_t)&exti13_handler,          /* 0xA0 */
    (uint32_t)&exti14_handler,          /* 0xA4 */
    (uint32_t)&exti15_handler,          /* 0xA8 */
    (uint32_t)&gpdma1_ch0_handler,      /* 0xAC */
    (uint32_t)&gpdma1_ch1_handler,      /* 0xB0 */
    (uint32_t)&gpdma1_ch2_handler,      /* 0xB4 */
    (uint32_t)&gpdma1_ch3_handler,      /* 0xB8 */
    (uint32_t)&gpdma1_ch4_handler,      /* 0xBC */
    (uint32_t)&gpdma1_ch5_handler,      /* 0xC0 */
    (uint32_t)&gpdma1_ch6_handler,      /* 0xC4 */
    (uint32_t)&gpdma1_ch7_handler,      /* 0xC8 */
    (uint32_t)&iwdg_handler,            /* 0xCC */
    0,                                  /* 0xD0 */
    (uint32_t)&adc1_handler,            /* 0xD4 */
    (uint32_t)&dac1_handler,            /* 0xD8 */
    (uint32_t)&fdcan_it0_handler,       /* 0xDC */
    (uint32_t)&fdcan_it1_handler,       /* 0xE0 */
    (uint32_t)&tim1_brk_handler,        /* 0xE4 */
    (uint32_t)&tim1_up_handler,         /* 0xE8 */
    (uint32_t)&tim1_trg_com_handler,    /* 0xEC */
    (uint32_t)&tim1_cc_handler,         /* 0xF0 */
    (uint32_t)&tim2_handler,            /* 0xF4 */
    (uint32_t)&tim3_handler,            /* 0xF8 */
    (uint32_t)&tim4_handler,            /* 0xFC */
    (uint32_t)&tim5_handler,            /* 0x100 */
    (uint32_t)&tim6_handler,            /* 0x104 */
    (uint32_t)&tim7_handler,            /* 0x108 */
    (uint32_t)&i2c1_ev_handler,         /* 0x10C */
    (uint32_t)&i2c1_er_handler,         /* 0x110 */
    (uint32_t)&i2c2_ev_handler,         /* 0x114 */
    (uint32_t)&i2c2_er_handler,         /* 0x118 */
    (uint32_t)&spi1_handler,            /* 0x11C */
    (uint32_t)&spi2_handler,            /* 0x120 */
    (uint32_t)&spi3_handler,            /* 0x124 */
    (uint32_t)&usart1_handler,          /* 0x128 */
    (uint32_t)&usart2_handler,          /* 0x12C */
    (uint32_t)&usart3_handler,          /* 0x130 */
    (uint32_t)&uart4_handler,           /* 0x134 */
    (uint32_t)&uart5_handler,           /* 0x138 */
    (uint32_t)&lpuart1_handler,         /* 0x13C */
    (uint32_t)&lptim1_handler,          /* 0x140 */
    (uint32_t)&tim8_brk_handler,        /* 0x144 */
    (uint32_t)&tim8_up_handler,         /* 0x148 */
    (uint32_t)&tim8_trg_com_handler,    /* 0x14C */
    (uint32_t)&tim8_cc_handler,         /* 0x150 */
    (uint32_t)&adc2_handler,            /* 0x154 */
    (uint32_t)&lptim2_handler,          /* 0x158 */
    (uint32_t)&tim15_handler,           /* 0x15C */
    0,                                  /* 0x160 */
    0,                                  /* 0x164 */
    (uint32_t)&usb_fs_handler,          /* 0x168 */
    (uint32_t)&crs_handler,             /* 0x16C */
    (uint32_t)&ucpd1_handler,           /* 0x170 */
    (uint32_t)&fmc_handler,             /* 0x174 */
    (uint32_t)&octospi1_handler,        /* 0x178 */
    (uint32_t)&sdmmc1_handler,          /* 0x17C */
    (uint32_t)&i2c3_ev_handler,         /* 0x180 */
    (uint32_t)&i2c3_er_handler,         /* 0x184 */
    (uint32_t)&spi4_handler,            /* 0x188 */
    0,                                  /* 0x18C */
    0,                                  /* 0x190 */
    (uint32_t)&usart6_handler,          /* 0x194 */
    0,                                  /* 0x198 */
    0,                                  /* 0x19C */
    0,                                  /* 0x1A0 */
    0,                                  /* 0x1A4 */
    (uint32_t)&gpdma2_ch0_handler,      /* 0x1A8 */
    (uint32_t)&gpdma2_ch1_handler,      /* 0x1AC */
    (uint32_t)&gpdma2_ch2_handler,      /* 0x1B0 */
    (uint32_t)&gpdma2_ch3_handler,      /* 0x1B4 */
    (uint32_t)&gpdma2_ch4_handler,      /* 0x1B8 */
    (uint32_t)&gpdma2_ch5_handler,      /* 0x1BC */
    (uint32_t)&gpdma2_ch6_handler,      /* 0x1C0 */
    (uint32_t)&gpdma2_ch7_handler,      /* 0x1C4 */
    0,                                  /* 0x1C8 */
    0,                                  /* 0x1CC */
    0,                                  /* 0x1D0 */
    0,                                  /* 0x1D4 */
    0,                                  /* 0x1D8 */
    (uint32_t)&fpu_handler,             /* 0x1DC */
    (uint32_t)&icache_handler,          /* 0x1E0 */
    (uint32_t)&dcache_handler,          /* 0x1E4 */
    0,                                  /* 0x1E8 */
    0,                                  /* 0x1EC */
    (uint32_t)&dcmi_pssi_handler,       /* 0x1F0 */
    (uint32_t)&fdcan2_it0_handler,      /* 0x1F4 */
    (uint32_t)&fdcan2_it1_handler,      /* 0x1F8 */
    0,                                  /* 0x1FC */
    0,                                  /* 0x200 */
    (uint32_t)&dts_handler,             /* 0x204 */
    (uint32_t)&rng_handler,             /* 0x208 */
    (uint32_t)&otfdec1_handler,         /* 0x20C */
    (uint32_t)&aes_handler,             /* 0x210 */
    (uint32_t)&hash_handler,            /* 0x214 */
    (uint32_t)&pka_handler,             /* 0x218 */
    (uint32_t)&cec_handler,             /* 0x21C */
    (uint32_t)&tim12_handler,           /* 0x220 */
    0,                                  /* 0x224 */
    0,                                  /* 0x228 */
    (uint32_t)&i3c1_ev,                 /* 0x22C */
    (uint32_t)&i3c1_er,                 /* 0x230 */
    0,                                  /* 0x234 */
    0,                                  /* 0x238 */
    0,                                  /* 0x23C */
    0,                                  /* 0x240 */
    0,                                  /* 0x244 */
    0,                                  /* 0x248 */
    (uint32_t)&i3c2_ev,                 /* 0x24C */
    (uint32_t)&i3c2_er,                 /* 0x250 */
    0                                   /* 0x254 */
};

extern volatile uint32_t _etext, _sdata, _edata, _sbss, _ebss, _sidata;
void main(void);

void reset_handler(void) {
    /* SCB->CPACR, should enable hardware floating point */
    *(uint32_t*)(0xE000ED88UL) |= ((3UL << 20U)|(3UL << 22U));

    /* Copy .data from FLASH to SRAM */
    volatile uint32_t data_size = (uint32_t)&_edata - (uint32_t)&_sdata;
    volatile uint8_t *flash_data = (uint8_t*) &_sidata; /* Data load address (in flash) */
    volatile uint8_t *sram_data = (uint8_t*) &_sdata; /* Data virtual address (in sram) */

    for (uint32_t i = 0; i < data_size; i++) {
        sram_data[i] = flash_data[i];
    }

    /* Zero-fill .bss section in SRAM */
    volatile uint32_t bss_size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    volatile uint8_t *bss = (uint8_t*) &_sbss;

    for (volatile uint32_t i = 0; i < bss_size; i++) {
        bss[i] = 0;
    }

    /* Call main processor logic */
    main();
}

void default_handler(void) {
    /* TODO: error handling? */
    while (1);
}

extern void* memset(void* dest, int c, unsigned int len) {
    uint8_t *p = dest;

    while (len > 0) {
        *p = c;
        p++;
        len--;
    }

    return dest;
}

extern void * memcpy(void *dest, const void *src, unsigned int len) {
    uint8_t * p = dest;
    uint8_t * s = src;

    while (len > 0) {
        *p = *s;
        p++;
        s++;
        len--;
    }

    return dest;
}


/* shit function lol (decommissioned but still in our hearts) */
/*
extern uint32_t __aeabi_uidivmod(uint32_t u, uint32_t v){
    uint32_t div = u;
    while(div > v) div -= v;
    return div;
}
*/ 
