#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "stm32h5xx.h"
#include "stm32h5xxCANSRAM.h"

void CAN_Init(){

	/* Enable FDCAN clock */
	RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
	__DSB(); /* data sync -> wait for mem update stage in pipeline */

	/* clear sleep bit */
	FDCAN1->CCCR &= ~(FDCAN_CCCR_CSR);
	while (FDCAN1->CCCR & FDCAN_CCCR_CSA){};

	/* Enter initialization mode */
	FDCAN1->CCCR |= FDCAN_CCCR_INIT;
	while((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0) {
        /* Wait for INIT to set */
    }

	/* Enable the Configuration Change Enable (CCE) bit */
	FDCAN1->CCCR |= FDCAN_CCCR_CCE;
	while ((FDCAN1->CCCR & FDCAN_CCCR_CCE) == 0);  /* wait for cce */

	/* TODO: get rid of this */
	FDCAN1->CCCR |= FDCAN_CCCR_DAR; /* disable automatic retransmission */

	/* clock in is at 125mhz (set in clock config), there are 4 divs and the first seg is 2 long */
    FDCAN1->NBTP = (124 << FDCAN_NBTP_NBRP_Pos) |
                   (1 << FDCAN_NBTP_NTSEG1_Pos) |		
                   (0 << FDCAN_NBTP_NTSEG2_Pos) |		
				   (0 << FDCAN_NBTP_NSJW_Pos);

    /* Configure Global Acceptance Filtering to set blank filter */
    FDCAN1->RXGFC |= (0 << FDCAN_RXGFC_ANFS_Pos)|
    				(0 << FDCAN_RXGFC_ANFE_Pos);

    FDCAN1->TXBC &= ~(1 << FDCAN_TXBC_TFQM_Pos); // clear queue bit

    /* Exit Initialize Mode */
	FDCAN1->CCCR &= ~(FDCAN_CCCR_INIT);
    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) {
    	/* Wait */
    }
}

void CAN_sendmessage(FDCAN_GlobalTypeDef* FDCAN, uint16_t id, uint8_t length, uint8_t* data){
	/* wait while buffer is full, if it doesn't become unfull, return */
	int timeout = 1024;
	while((FDCAN->TXFQS & FDCAN_TXFQS_TFQF_Msk) && timeout) timeout--;
	//if(!timeout) return;

	/* first open buffer slot */
	int j = (FDCAN->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;

	/* Find the indexed buffer */
	FDCAN_Tx_FIFO_Element_Typedef *msg = FDCAN == FDCAN1 ? &FDCAN1_RAM->tx_buffers[j] : &FDCAN2_RAM->tx_buffers[j];								// use nicoles buffers from header file

	msg->H0 = 0; // 11 bit ID goes to T0
    msg->H0 |= ((uint32_t)id << FDCAN_TXBH0_STDID_Pos);     /* STDID   */

    msg->H1  = 0;
    msg->H1 |= ((uint32_t)length << FDCAN_TXBH1_DLC_Pos);      /* DLC */

    for(uint8_t i = 0; i < length; i++)	// Copy data to buffer
    	msg->data[i] = data[i];

    FDCAN1->TXBAR |= (1UL << j); // Trigger message by writing to TX buffer

    (void)id;
}
