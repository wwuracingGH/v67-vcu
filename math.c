unsigned int clz (unsigned int v) {
    unsigned int bits = 16;
    unsigned int count = 32;
    do {
        unsigned int t = v >> bits;
        if (t != 0) {
            count -= bits;
            v = t;
        }
        bits >>= 1;
    } while (bits != 0);
    count -= v;
    return count;
}

uint32_t __aeabi_uidiv(uint32_t u, uint32_t v) {
    uint32_t q;
    uint32_t k;
    //
    q = 0;
    k = clz(v) - clz(u);  // Note: don't add 1, adjust case values
    //
    switch (k) {
    case 31: if (v <= (u >> 31)) { u -= v << 31; q += 1 << 31; }
    // fall through
    case 30: if (v <= (u >> 30)) { u -= v << 30; q += 1 << 30; }
    // fall through
    case 29: if (v <= (u >> 29)) { u -= v << 29; q += 1 << 29; }
    // fall through
    case 28: if (v <= (u >> 28)) { u -= v << 28; q += 1 << 28; }
    // fall through
    case 27: if (v <= (u >> 27)) { u -= v << 27; q += 1 << 27; }
    // fall through
    case 26: if (v <= (u >> 26)) { u -= v << 26; q += 1 << 26; }
    // fall through
    case 25: if (v <= (u >> 25)) { u -= v << 25; q += 1 << 25; }
    // fall through
    case 24: if (v <= (u >> 24)) { u -= v << 24; q += 1 << 24; }
    // fall through
    case 23: if (v <= (u >> 23)) { u -= v << 23; q += 1 << 23; }
    // fall through
    case 22: if (v <= (u >> 22)) { u -= v << 22; q += 1 << 22; }
    // fall through
    case 21: if (v <= (u >> 21)) { u -= v << 21; q += 1 << 21; }
    // fall through
    case 20: if (v <= (u >> 20)) { u -= v << 20; q += 1 << 20; }
    // fall through
    case 19: if (v <= (u >> 19)) { u -= v << 19; q += 1 << 19; }
    // fall through
    case 18: if (v <= (u >> 18)) { u -= v << 18; q += 1 << 18; }
    // fall through
    case 17: if (v <= (u >> 17)) { u -= v << 17; q += 1 << 17; }
    // fall through
    case 16: if (v <= (u >> 16)) { u -= v << 16; q += 1 << 16; }
    // fall through
    case 15: if (v <= (u >> 15)) { u -= v << 15; q += 1 << 15; }
    // fall through
    case 14: if (v <= (u >> 14)) { u -= v << 14; q += 1 << 14; }
    // fall through
    case 13: if (v <= (u >> 13)) { u -= v << 13; q += 1 << 13; }
    // fall through
    case 12: if (v <= (u >> 12)) { u -= v << 12; q += 1 << 12; }
    // fall through
    case 11: if (v <= (u >> 11)) { u -= v << 11; q += 1 << 11; }
    // fall through
    case 10: if (v <= (u >> 10)) { u -= v << 10; q += 1 << 10; }
    // fall through
    case  9: if (v <= (u >>  9)) { u -= v <<  9; q += 1 <<  9; }
    // fall through
    case  8: if (v <= (u >>  8)) { u -= v <<  8; q += 1 <<  8; }
    // fall through
    case  7: if (v <= (u >>  7)) { u -= v <<  7; q += 1 <<  7; }
    // fall through
    case  6: if (v <= (u >>  6)) { u -= v <<  6; q += 1 <<  6; }
    // fall through
    case  5: if (v <= (u >>  5)) { u -= v <<  5; q += 1 <<  5; }
    // fall through
    case  4: if (v <= (u >>  4)) { u -= v <<  4; q += 1 <<  4; }
    // fall through
    case  3: if (v <= (u >>  3)) { u -= v <<  3; q += 1 <<  3; }
    // fall through
    case  2: if (v <= (u >>  2)) { u -= v <<  2; q += 1 <<  2; }
    // fall through
    case  1: if (v <= (u >>  1)) { u -= v <<  1; q += 1 <<  1; }
    // fall through
    case  0: if (v <= (u >>  0)) { u -= v <<  0; q += 1 <<  0; }
    // fall through
    default: break;
    }
    //
    return q;
}

int main(){
    //setup
    clock_init();
    SystemCoreClockUpdate();
    
    GPIO_Init(); //must be called first
    ADC_DMA_Init(&ADC_Vars.APPS2, 4); //Bad practice 🤷‍♀️
    CAN_Init();

    __enable_irq(); //enable interrupts

    uint32_t canTimer = 500000;

    for(;;) {
        APPS_calc(&car_state.torque_req);

        while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
            uint8_t  can_len    = CAN->sFIFOMailBox[0].RDTR & 0xF;
            uint64_t can_data   = CAN->sFIFOMailBox[0].RDLR + ((uint64_t)CAN->sFIFOMailBox[0].RDHR << 32);
            uint16_t can_id     = CAN->sFIFOMailBox[0].RIR >> CAN_RI0R_STID_Pos;
            CAN->RF0R |= CAN_RF0R_RFOM0; //release mailbox

            CAN_msg canrx = {can_id, can_len, can_data};
            process_can(canrx);
        }

        if(canTimer--);
        else {
            canTimer = 500000;
            send_CAN(0b0001, 8, (uint8_t*)&ADC_Vars.APPS2);
            
        }
    }
}   