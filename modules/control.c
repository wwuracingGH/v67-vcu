#include "control.h"
#include <stdint.h>
#include "stm32h533xx.h"
#include "logging.h"

uint16_t raw_vals[ROLLING_ADC_VALS];

void waitus(volatile uint32_t us){
    for(volatile int i = us * 10; i > 0; i--);
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
    
    GPIOA->ODR |= 1 << 6;
    waitus(10); /* wait 10us as specified on the datasheet */
    GPIOA->ODR = ~(1 << 6);

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
	uint32_t apps12 = 0, apps34 = 0, afrbps = 0;
    uint32_t* raw2_vals = (uint32_t*)&raw_vals[0];
    for(; raw2_vals < (raw_vals + ROLLING_ADC_VALS); raw2_vals += 3){
        apps12 = __UADD16(apps12, raw2_vals[0]);
        apps34 = __UADD16(apps34, raw2_vals[1]);
        afrbps = __UADD16(afrbps, raw2_vals[2]);
    }

    ADC_Block_t block = { 
        (apps12 & 0xFFFF) >> ROLLING_ADC_FR_POW, 
        apps12 >> (ROLLING_ADC_FR_POW + 16),
        (apps34 & 0xFFFF) >> ROLLING_ADC_FR_POW, 
        apps34 >> (ROLLING_ADC_FR_POW + 16),
        (afrbps & 0xFFFF) >> ROLLING_ADC_FR_POW, 
        afrbps >> (ROLLING_ADC_FR_POW + 16),
    };

    return block;
}

int find_avg_4(int * vals, int min, int max){
    int avg = 0;
    for(int i = min; i <= max; i++)
        avg += vals[i];
    avg /= max - min;
    return avg;
}

float find_avg_4_fst(int * vals, int min, int max){
    int avg = 0;
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
void sort_dat_4(int32_t * vals){
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

ADC_Mult_t get_adc_multiplers(ADC_Bounds_t* bounds) {
    ADC_Mult_t mult = {0};
    mult.APPS1_strt = bounds->APPS1_l;
    mult.APPS2_strt = bounds->APPS2_l;
    mult.APPS3_strt = bounds->APPS3_l;
    mult.APPS4_strt = bounds->APPS4_l;

    mult.APPS1_mult = 65536L / ((int32_t)bounds->APPS1_h - bounds->APPS1_l);
    mult.APPS2_mult = 65536L / ((int32_t)bounds->APPS2_h - bounds->APPS2_l);
    mult.APPS3_mult = 65536L / ((int32_t)bounds->APPS3_h - bounds->APPS3_l);
    mult.APPS4_mult = 65536L / ((int32_t)bounds->APPS4_h - bounds->APPS4_l);
    
    mult.BPS_f_min = bounds->BPS_f_min;
    mult.BPS_r_min = bounds->BPS_r_min;

    return mult;
}

/*  Internal torque request calculation
    returns a mess: 
    low "word": 16 bit unsigned integer 
    high "word": sign (if doing something like regen), and 3 error bits at the top
    */
TorqueReq_t calc_torque_request(ADC_Mult_t* mult, ControlParams_t* params) {
    GPIOA->ODR &= ~(1 << 6);
    ADC_Block_t vals = condense();
    GPIOA->ODR |= 1 << 6;

    TorqueReq_t torque_request = {0, 0};

    int braking_pressure = 0;
    int bse_data_err = (vals.FBPS < mult->BPS_f_min) << 1 | (vals.RBPS < mult->BPS_r_min);
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

    int32_t apps[4];
    apps[0] = ((int32_t)vals.APPS1 - (int32_t)mult->APPS1_strt) * mult->APPS1_mult;
    apps[1] = ((int32_t)vals.APPS2 - (int32_t)mult->APPS2_strt) * mult->APPS2_mult;
    apps[2] = ((int32_t)vals.APPS3 - (int32_t)mult->APPS3_strt) * mult->APPS3_mult;
    apps[3] = ((int32_t)vals.APPS4 - (int32_t)mult->APPS4_strt) * mult->APPS4_mult;

    //LOG("%d %d %d %d \n", apps[0], apps[1], apps[2], apps[3]);

    GPIOA->ODR &= ~(1 << 6);
    sort_dat_4(apps);
    GPIOA->ODR |= (1 << 6);

    int mindex = 3;
    int maxdex = 1;
    
    for(int i = 0; i < 4; i++){
        int oob = (apps[i] > (APPS_OUT_OF_BOUNDS + (1 << 16))) || (apps[i] < -(APPS_OUT_OF_BOUNDS));
        if(!oob) {
            if(i < mindex)
                mindex = i;
            if(i > maxdex)
                maxdex = i;
        }
    }

    if(mindex >= maxdex){
        torque_request.flags |= APPS_FAULT_BOUNDS;
    }

    int avg = find_avg_4(apps, mindex, maxdex);

    while(apps[maxdex] - apps[mindex] > 6553 && (maxdex >= mindex)){ /* 10% in */
        if(apps[maxdex] + apps[mindex] > avg * 2){
            maxdex--;
        }
        else {
            mindex++;
        }
        avg = find_avg_4(apps, mindex, maxdex);
    }

    //LOG("%d %d %d %d %d %d \n", apps[0], apps[1], apps[2], apps[3], maxdex, mindex);

    if(mindex == maxdex){
        torque_request.flags |= APPS_FAULT_DELTA;
    }

    if(avg < 0) avg = 0;
    if(avg > 65536) avg = 65536;

    if(avg > APPS_BPS_PLAUS && braking_pressure > params->brake_threashold) {
        torque_request.flags |= APPS_FAULT_PLAUS;
    }

    torque_request.torque = (avg * params->max_torque) >> 16;

    //LOG("%d, %d\n", torque_request.torque, torque_request.flags);

    return torque_request;
}

