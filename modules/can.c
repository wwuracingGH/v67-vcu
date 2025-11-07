#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "can.h"
#include "stm32h5xx.h"

void CAN_init() {
    /* Enable FDCAN clock */
    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    __DSB(); /* data sync -> wait for mem update stage in pipeline */

    /* clear sleep bit */
    FDCAN1->CCCR &= ~(FDCAN_CCCR_CSR);
    while (FDCAN1->CCCR & FDCAN_CCCR_CSA);

    /* Enter initialization mode */
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0) {
        /* Wait for INIT to set */
    }

    /* Enable the Configuration Change Enable (CCE) bit */
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    while ((FDCAN1->CCCR & FDCAN_CCCR_CCE) == 0);  /* wait for cce */

    /* TODO: get rid of this */
    FDCAN1->CCCR |= FDCAN_CCCR_DAR; /* disable automatic retransmission */

    /* clock in is at 125mhz (set in clock config), there are 5 divs and the smp is 75% */
    FDCAN1->NBTP = (CAN1_BAUD_PRES << FDCAN_NBTP_NBRP_Pos) |
                   (2 << FDCAN_NBTP_NTSEG1_Pos) |
                   (0 << FDCAN_NBTP_NTSEG2_Pos) |
                   (0 << FDCAN_NBTP_NSJW_Pos);

    /* Configure Global Acceptance Filtering to set blank filter */
    FDCAN1->RXGFC |= (0 << FDCAN_RXGFC_ANFS_Pos) |
                     (0 << FDCAN_RXGFC_ANFE_Pos) |
                     FDCAN_RXGFC_F0OM | FDCAN_RXGFC_F1OM;

    FDCAN1->TXBC &= ~(1 << FDCAN_TXBC_TFQM_Pos); /* clear queue bit */

    /* clear sleep bit */
    FDCAN2->CCCR &= ~(FDCAN_CCCR_CSR);
    while (FDCAN2->CCCR & FDCAN_CCCR_CSA);

    /* Enter initialization mode */
    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    while ((FDCAN2->CCCR & FDCAN_CCCR_INIT) == 0);

    /* Enable the Configuration Change Enable (CCE) bit */
    FDCAN2->CCCR |= FDCAN_CCCR_CCE;
    while ((FDCAN2->CCCR & FDCAN_CCCR_CCE) == 0);  /* wait for cce */

    /* TODO: get rid of this */
    FDCAN2->CCCR |= FDCAN_CCCR_DAR; /* disable automatic retransmission */

    /* clock in is at 125mhz (set in clock config), there are 4 divs and the first seg is 2 long */
    FDCAN2->NBTP = (CAN2_BAUD_PRES << FDCAN_NBTP_NBRP_Pos) |
                   (2 << FDCAN_NBTP_NTSEG1_Pos) |
                   (0 << FDCAN_NBTP_NTSEG2_Pos) |
                   (0 << FDCAN_NBTP_NSJW_Pos);

    /* Configure Global Acceptance Filtering to set blank filter */
    FDCAN2->RXGFC |= (0 << FDCAN_RXGFC_ANFS_Pos) |
                     (0 << FDCAN_RXGFC_ANFE_Pos) |
                     FDCAN_RXGFC_F0OM | FDCAN_RXGFC_F1OM;

    FDCAN2->TXBC &= ~(1 << FDCAN_TXBC_TFQM_Pos); /* clear queue bit */

    FDCAN1->CCCR |= FDCAN_CCCR_FDOE;
    FDCAN2->CCCR |= FDCAN_CCCR_FDOE;

    /* Exit Initialize Mode */
    FDCAN1->CCCR &= ~(FDCAN_CCCR_INIT);
    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) {
        /* Wait */
    }

    /* Exit Initialize Mode */
    FDCAN2->CCCR &= ~(FDCAN_CCCR_INIT);
    while (FDCAN2->CCCR & FDCAN_CCCR_INIT) {
        /* Wait */
    }
}

void CAN_reset() {
    RCC->APB1HENR &= ~RCC_APB1HENR_FDCANEN;
    __DSB();
    RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
    __DSB();
    RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
    __DSB();

    CAN_init();
}

void CAN_sendmessage(FDCAN_GlobalTypeDef* FDCAN, uint16_t id, uint8_t length, uint8_t* data) {
    /* wait while buffer is full, if it doesn't become unfull, return */
    int timeout = 1024;
    while ((FDCAN->TXFQS & FDCAN_TXFQS_TFQF_Msk) && timeout) timeout--;
    /* Maybe reenable? 
     * if(!timeout) return; 
     */

    /* first open buffer slot */
    int j = (FDCAN->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;

    /* Find the indexed buffer */
    FDCAN_SRAM_Typedef * fd_ram = (FDCAN == FDCAN1) ? FDCAN1_RAM : FDCAN2_RAM;
    FDCAN_Tx_FIFO_Element_Typedef *msg = &fd_ram->tx_buffers[j];

    msg->H0 = 0; /* 11 bit ID goes to T0 */
    msg->H0 |= ((uint32_t)id << FDCAN_TXBH0_STDID_Pos); /* STDID */

    msg->H1  = 0;
    msg->H1 |= ((uint32_t)length << FDCAN_TXBH1_DLC_Pos); /* DLC */

    msg->data[0] = *((uint32_t*)data);
    msg->data[1] = *(((uint32_t*)data) + 1);

    FDCAN->TXBAR |= (1UL << j); /* Trigger message by writing to TX buffer */
}

uint32_t _get_id(FDCAN_Rx_FIFO_Element_Typedef* msg) {
    return (msg->H0 & FDCAN_RXBH0_XTD_Msk) ? (msg->H0 & FDCAN_RXBH0_EXTID_Msk) :
            (msg->H0 & FDCAN_RXBH0_STDID_Msk) >> FDCAN_RXBH0_STDID_Pos;
}

uint8_t _get_dlc(FDCAN_Rx_FIFO_Element_Typedef* msg) {
    return (msg->H1 & FDCAN_RXBH1_DLC_Msk) >> FDCAN_RXBH1_DLC_Pos;
}

uint32_t* _get_data(FDCAN_Rx_FIFO_Element_Typedef* msg) {
    return msg->data;
}

void CAN_recieveMessages(void (*callback)(uint8_t bus, uint32_t id, uint8_t dlc, uint32_t* data)) {
    while (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL_Msk) {
        int j = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
        FDCAN_Rx_FIFO_Element_Typedef* msg = FDCAN1_RAM->rx_fifo0 + j;
        callback(0, _get_id(msg), _get_dlc(msg), _get_data(msg));
        FDCAN1->RXF0A |= j;
        __DSB();
    }
    while (FDCAN1->RXF1S & FDCAN_RXF1S_F1FL_Msk) {
        int j = (FDCAN1->RXF1S & FDCAN_RXF1S_F1GI_Msk) >> FDCAN_RXF1S_F1GI_Pos;
        FDCAN_Rx_FIFO_Element_Typedef* msg = FDCAN1_RAM->rx_fifo1 + j;
        callback(0, _get_id(msg), _get_dlc(msg), _get_data(msg));
        FDCAN1->RXF1A |= j;
        __DSB();
    }
    while (FDCAN2->RXF0S & FDCAN_RXF0S_F0FL_Msk) {
        int j = (FDCAN2->RXF0S & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
        FDCAN_Rx_FIFO_Element_Typedef* msg = FDCAN2_RAM->rx_fifo0 + j;
        callback(1, _get_id(msg), _get_dlc(msg), _get_data(msg));
        FDCAN2->RXF0A |= j;
        __DSB();
    }
    while (FDCAN2->RXF1S & FDCAN_RXF1S_F1FL_Msk) {
        int j = (FDCAN2->RXF1S & FDCAN_RXF1S_F1GI_Msk) >> FDCAN_RXF1S_F1GI_Pos;
        FDCAN_Rx_FIFO_Element_Typedef* msg = FDCAN2_RAM->rx_fifo1 + j;
        callback(1, _get_id(msg), _get_dlc(msg), _get_data(msg));
        FDCAN2->RXF1A |= j;
        __DSB();
    }
}

int CAN_bytesFromDLC(int dlc) {
    const int bytes[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    return bytes[dlc]; 
}

int CAN_DLCFromBytes(int bytes) {
    /* TODO */
}