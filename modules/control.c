#include "control.h"
#include <stdint.h>
#include "stm32h533xx.h"
#include "logging.h"

uint16_t raw_vals[ROLLING_ADC_VALS];

void waitus(volatile uint32_t us){
	/* waits the correct number of clock cycles for 72mhz
	 * Each loop takes exactly 3 clock cycles
	 * So 72000/3 = 24000 loop cycles per ms */
	asm(	"ldr r1, =#13\n"
	        "mul r1, r0, r1\n"
	        "waitLoop:\n"
	        "subs r1, r1, #1\n"
	        "bne waitLoop\n"
	);
}

/* copy of the DAR */
uint32_t gpdma_linked_list = (uint32_t)(&raw_vals);
uint32_t gpdma_ll_loc = (uint32_t)(&gpdma_linked_list);


/* init adc1 to work with the dma in the background */
void ADC_Init(){
    /* adc1 sequence -> 0 1 14 15 18 19 */

    /* enable the adc */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    /* make sure it's on */
    ADC1->CR &= ~ADC_CR_DEEPPWD;

    /* TODO: enable adc clock -> make sure it's < 125 MHZ */
    ADC12_COMMON->CCR |= 2UL << ADC_CCR_CKMODE_Pos;

    /* start the voltage regulator */
    ADC1->CR |= ADC_CR_ADVREGEN;
    waitus(10); /* wait 10us as specified on the datasheet */

    /* calibrate the adc */
    ADC1->CR |= 1UL << ADC_CR_ADCAL_Pos;   
    while(ADC1->CR & ADC_CR_ADCAL);
    ADC1->CR |= ADC_CR_ADEN;

    /* ensure injection disabled, continuous, dma enabled */
    ADC1->CFGR |= ADC_CFGR_JQDIS | ADC_CFGR_CONT | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;

    /* NO OVERSAMPLING, can be changed later */
    uint32_t t_smp = 4UL;
    ADC1->SMPR1 |= (t_smp << ADC_SMPR1_SMP0_Pos) | (t_smp << ADC_SMPR1_SMP1_Pos);
    ADC1->SMPR2 |= (t_smp << ADC_SMPR2_SMP14_Pos) | (t_smp << ADC_SMPR2_SMP15_Pos)
                | (t_smp << ADC_SMPR2_SMP18_Pos) | (t_smp << 27UL);

    /* 16 bit adc value, oversampled */
    ADC1->CFGR2 |= (3UL << ADC_CFGR2_OVSR_Pos) | (4UL << ADC_CFGR2_OVSS_Pos) | ADC_CFGR2_ROVSE;

    ADC1->OR |= 1;
    /* set sequence */
    ADC1->SQR1 |= (ADC_CHANNELS - 1) << ADC_SQR1_L_Pos;
    ADC1->SQR1 |= (0 << ADC_SQR1_SQ1_Pos) | (1 << ADC_SQR1_SQ2_Pos) | (14 << ADC_SQR1_SQ3_Pos) |
                (15 << ADC_SQR1_SQ4_Pos);
    ADC1->SQR2 |= (19 << ADC_SQR2_SQ6_Pos) | (18 << ADC_SQR2_SQ5_Pos);

    ADC1->CR |= ADC_CR_ADEN;

    /* it already is but whatever */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPDMA2EN;
    __DSB();

	GPDMA2_Channel0->CTR1 |= DMA_CTR1_DINC | DMA_CTR1_DAP; /* set dest inc burst and port of destination to memory */
	GPDMA2_Channel0->CTR1 |= (0b01 << DMA_CTR1_DDW_LOG2_Pos) | (0b01 << DMA_CTR1_SDW_LOG2_Pos); /* data width */
	GPDMA2_Channel0->CTR2 |= (0 << DMA_CTR2_REQSEL_Pos) | DMA_CTR2_PFREQ; /* source to adc 1, triggered correctly */

	GPDMA2_Channel0->CDAR = (uint32_t)(&raw_vals[0]); /* sets destination of dma transfer */
	GPDMA2_Channel0->CSAR = (uint32_t)(&(ADC1->DR)); /* sets source of dma transfer */

	/* sets size of burst at 1 */
	GPDMA2_Channel0->CTR1 &= ~(63 << DMA_CTR1_DBL_1_Pos) | ~(63 << DMA_CTR1_SBL_1_Pos);

	/* sets size of dma transfer */
	GPDMA2_Channel0->CBR1 &= ~DMA_CBR1_BNDT_Msk;
	GPDMA2_Channel0->CBR1 |= (ROLLING_ADC_VALS * 2) << DMA_CBR1_BNDT_Pos;

	gpdma_ll_loc = (uint32_t)&gpdma_linked_list;
	GPDMA2_Channel0->CLLR |= DMA_CLLR_UDA | (0xFFFF & gpdma_ll_loc);
	GPDMA2_Channel0->CLBAR |= gpdma_ll_loc & (0xFFFF << 16);

	GPDMA2_Channel0->CCR |= DMA_CCR_EN | DMA_CCR_LAP | (3UL << DMA_CCR_PRIO_Pos); /* enable the dma */

    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1->CR |= ADC_CR_ADSTART; /* start the adc */
}


/* internal function that condenses and writes apps values */
/* compiler should use SIMD instructions for this - maybe check? */
ADC_Block_t condense() {
	if(ADC1->ISR & ADC_ISR_OVR)
		ADC1->ISR |= ADC_ISR_OVR;
	uint32_t aaps1 = 0, aaps2 = 0, aaps3 = 0, aaps4 = 0, afbps = 0, arbps = 0;
    for(int i = 0; i < ROLLING_ADC_VALS; i += 6){
        aaps1 += raw_vals[i    ];
        aaps2 += raw_vals[i + 1];
        aaps3 += raw_vals[i + 2];
        aaps4 += raw_vals[i + 3];
        afbps += raw_vals[i + 4];
        arbps += raw_vals[i + 5];
    }

    ADC_Block_t block = { aaps1 >> ROLLING_ADC_FR_POW, aaps2 >> ROLLING_ADC_FR_POW,
        aaps3 >> ROLLING_ADC_FR_POW, aaps4 >> ROLLING_ADC_FR_POW, afbps >> ROLLING_ADC_FR_POW,
             arbps >> ROLLING_ADC_FR_POW};

    return block;
}

float find_avg_4(float * vals, int min, int max){
    float avg = 0;
    for(int i = min; i <= max; i++)
        avg += vals[i];
    avg /= max - min;
    return avg;
}

float find_avg_4_fst(float * vals, int min, int max){
    float avg = 0;
    switch(min){ /* hoping for a faster function on average */
        case 0: 
            avg += vals[0]; 
            if(0 >= max) break;
            // fall through
        case 1: 
            avg += vals[1]; 
            if(1 >= max) break;
            // fall through
        case 2: 
            avg += vals[2]; 
            if(2 >= max) break;
            // fall through
        case 3: 
            avg += vals[3]; 
            if(3 >= max) break;
    }
        
    avg /= max - min;
    return avg;
}

/* CAS instruction... wish you were here... */
void sort_dat_4(float * vals){
    if(vals[0] > vals[2]){
        int tmp = vals[0];
        vals[0] = vals[2];
        vals[2] = tmp;
    }

    if(vals[1] > vals[3]){
        int tmp = vals[1];
        vals[1] = vals[3];
        vals[3] = tmp;
    }

    if(vals[0] > vals[1]){
        int tmp = vals[0];
        vals[0] = vals[1];
        vals[1] = tmp;
    }

    if(vals[2] > vals[3]){
        int tmp = vals[2];
        vals[2] = vals[3];
        vals[3] = tmp;
    }

    if(vals[1] > vals[2]){
        int tmp = vals[1];
        vals[1] = vals[2];
        vals[2] = tmp;
    }
}

/*  Internal torque request calculation
    returns a mess: 
    low "word": 16 bit unsigned integer 
    high "word": sign (if doing something like regen), and 3 error bits at the top
    */
TorqueReq_t calc_torque_request(ADC_Bounds_t bounds, ControlParams_t params) {
    ADC_Block_t vals = condense();

    TorqueReq_t torque_request = {0, 0};

    int braking_pressure = 0;
    int bse_data_err = (vals.FBPS < bounds.BPS_min) << 1 | (vals.RBPS < bounds.BPS_min);
    switch(bse_data_err){
    case 2:
        braking_pressure = vals.RBPS; break;
    case 1:
        braking_pressure = vals.FBPS; break;
    case 0:
        braking_pressure = (vals.RBPS + vals.FBPS) >> 1; break;
    default:
        torque_request.flags |= APPS_FAULT_BSE; break;
    }

    float apps[4];
    apps[0] = (float)(vals.APPS1 - bounds.APPS1_l) / (float)(bounds.APPS1_h - bounds.APPS1_l);
    apps[1] = (float)(vals.APPS2 - bounds.APPS2_l) / (float)(bounds.APPS2_h - bounds.APPS2_l);
    apps[2] = (float)(vals.APPS3 - bounds.APPS3_l) / (float)(bounds.APPS3_h - bounds.APPS3_l);
    apps[3] = (float)(vals.APPS4 - bounds.APPS4_l) / (float)(bounds.APPS4_h - bounds.APPS4_l);

    LOG("%f %f %f %f \n", apps[0], apps[1], apps[2], apps[3]);

    sort_dat_4(apps);

    int mindex = 3;
    int maxdex = 1;
    
    for(int i = 0; i < 4; i++){
        int oob = (apps[i] > (APPS_OUT_OF_BOUNDS + 1)) || (apps[i] < -(APPS_OUT_OF_BOUNDS));
        if(!oob) {
            if(i < mindex)
                mindex = i;
            if(i > maxdex)
                maxdex = i;
        }
    }

    if(mindex == maxdex){
        torque_request.flags |= APPS_FAULT_BOUNDS;
    }

    float avg = find_avg_4_fst(apps, mindex, maxdex);

    while(apps[maxdex] - apps[mindex] > 0.1f){
        if(apps[maxdex] + apps[mindex] > avg * 2){
            maxdex--;
        }
        else {
            mindex++;
        }
        avg = find_avg_4(apps, mindex, maxdex);
    }

    if(mindex == maxdex){
        torque_request.flags |= APPS_FAULT_DELTA;
    }

    if(avg < 0.0f) avg = 0.0f;
    if(avg > 1.0f) avg = 1.0f;

    if(avg > 0.25f && braking_pressure > params.brake_threashold) {
        torque_request.flags |= APPS_FAULT_PLAUS;
    }

    torque_request.torque = avg * params.max_torque;

    return torque_request;
}

