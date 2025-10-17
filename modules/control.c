#include "Control.h"
#include <stdint.h>
#include "stm32h533xx.h"

/* init adc1 to work with the dma in the background */
void ADC_init(){
    /* adc1 sequence -> 0 1 14 15 18 19 */

    /* enable the adc */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    /* TODO: enable adc clock -> make sure it's < 125 MHZ */
    // CKMODE[1:0] of ADC_CCR must be different from 00
    // Divides the 250 MHz Clock by 2 to match 125 MHz
    ADC->CCR |= 2UL << ADC_CCR_CKMODE;

    /* make sure it's not on */
    ADC1->CR &= ~1UL;

    /* calibrate the adc */
    ADC1->CR |= 1UL << ADC_CR_ADCAL_Pos;   
    while(ADC1->CR & ADC_CR_ADCAL);

    /* ensure injection disabled, continuous, dma enabled */
    ADC1->CFGR |= ADC_CFGR_JQDIS | ADC_CFGR_CONT | ADC_CFGR_DMAEN;

    /* NO OVERSAMPLING, can be changed later */
    uint32_t oversampling = 0UL;
    ADC1->SMPR1 |= (oversampling << ADC_SMPR1_SMP0_Pos) | (oversampling << ADC_SMPR1_SMP1_Pos);
    ADC1->SMPR2 |= (oversampling << ADC_SMPR2_SMP14_Pos) | (oversampling << ADC_SMPR2_SMP15_Pos)
                | (oversampling << ADC_SMPR2_SMP18_Pos) | (oversampling << 27UL);

    /* set sequence */
    ADC1->SQR1 |= 5 << ADC_SQR1_L_Pos;
    ADC1->SQR1 |= (0 << ADC_SQR1_SQ1_Pos) | (1 << ADC_SQR1_SQ2_Pos) | (14 << ADC_SQR1_SQ3_Pos) |
                (15 << ADC_SQR1_SQ4_Pos);
    ADC1->SQR2 |= (19 << ADC_SQR2_SQ6_Pos) | (18 << ADC_SQR2_SQ5_Pos);



    /* enable the adc DONT UNCOMMENT UNTIL A BETTER CLOCK SOLUTION HAS BEEN FOUND */
    //ADC1->CR |= 1UL;

}


/* internal function that condenses and writes apps values */
/* compiler should use SIMD instructions for this - maybe check? */
ADC_Block_t condense(uint16_t * raw_vals) {
    ADC_Block_t b = {0, 0, 0, 0, 0, 0};
    for(int i = 0; i < ROLLING_ADC_VALS; i += 6){
        b.APPS1 += raw_vals[i + 1];
        b.APPS2 += raw_vals[i + 2];
        b.APPS3 += raw_vals[i + 3];
        b.APPS4 += raw_vals[i + 4];
        b.FBPS  += raw_vals[i + 5];
        b.RBPS  += raw_vals[i + 6];
    }
    b.APPS1 >>= ROLLING_ADC_FR_POW;
    b.APPS2 >>= ROLLING_ADC_FR_POW;
    b.APPS3 >>= ROLLING_ADC_FR_POW;
    b.APPS4 >>= ROLLING_ADC_FR_POW;
    b.FBPS >>= ROLLING_ADC_FR_POW;
    b.RBPS >>= ROLLING_ADC_FR_POW;
    return b;
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
    ADC_Block_t vals = condense(raw_vals);

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

