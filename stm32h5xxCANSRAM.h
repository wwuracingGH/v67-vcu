#ifndef __STM32_H5_CANSRAM__
#define __STM32_H5_CANSRAM__

typedef struct __attribute__((packed)){
	uint32_t H0;
	uint32_t H1;
	uint8_t data[64]; /* only the first two words of these are set in CAN 2.0 mode */
} FDCAN_Rx_FIFO_Element_Typedef;

typedef struct {
	uint32_t E0;
	uint32_t E1;
} FDCAN_Tx_FIFO_Event_Typedef;

typedef struct __attribute__((packed)){
	uint32_t H0;
	uint32_t H1;
	uint8_t data[64]; /* only the first two words of these are set in CAN 2.0 mode */
} FDCAN_Tx_FIFO_Element_Typedef;

typedef struct {
	uint32_t 			sid_filter[28];
	uint64_t 			exid_filter[8];
	FDCAN_Rx_FIFO_Element_Typedef 	rx_fifo0[3];
	FDCAN_Rx_FIFO_Element_Typedef 	rx_fifo1[3];
	FDCAN_Tx_FIFO_Event_Typedef	tx_event_fifo[3];
	FDCAN_Tx_FIFO_Element_Typedef 	tx_buffers[3];
} FDCAN_SRAM_Typedef;



#define FDCAN_RAM_SIZE 		0x0350

#define FDCAN1_RAM_BASE_S 	(0x4000AC00)
#define FDCAN2_RAM_BASE_S 	(FDCAN1_RAM_BASE_S + FDCAN_RAM_SIZE)

#define FDCAN1_RAM	 	((FDCAN_SRAM_Typedef*)FDCAN1_RAM_BASE_S)
#define FDCAN2_RAM 		((FDCAN_SRAM_Typedef*)FDCAN2_RAM_BASE_S)


/********************** FDCAN transmit buffer element / event bit definitions ********************/

/* HEADER 0 DEFINITIONS */
#define FDCAN_TXBH0_EXTID_Pos 		(0UL)
#define FDCAN_TXBH0_EXTID_Msk   	(0x0FFFFFFFUL << FDCAN_TXBH0_EXTID_Pos) 
#define FDCAN_TXBH0_EXTID        	FDCAN_TXBH0_EXTID_Msk
#define FDCAN_TXBH0_STDID_Pos 		(18UL)
#define FDCAN_TXBH0_STDID_Msk  		(0x7FFUL << FDCAN_TXBH0_STDID_Pos) 
#define FDCAN_TXBH0_STDID          	FDCAN_TXBH0_STDID_Msk
#define FDCAN_TXBH0_RTR_Pos 		(29UL)
#define FDCAN_TXBH0_RTR_Msk   		(0x1UL << FDCAN_TXBH0_RTR_Pos) 
#define FDCAN_TXBH0_RTR        		FDCAN_TXBH0_RTR_Msk
#define FDCAN_TXBH1_XTD_Pos 		(30UL)
#define FDCAN_TXBH1_XTD_Msk  		(0x1UL << FDCAN_TXBH0_EXT_Pos) 
#define FDCAN_TXBH1_XTD          	FDCAN_TXBH1_EXT_Msk
#define FDCAN_TXBH1_ESI_Pos 		(31UL)
#define FDCAN_TXBH1_ESI_Msk  		(0x1UL << FDCAN_TXBH0_ESI_Pos) 
#define FDCAN_TXBH1_ESI          	FDCAN_TXBH1_ESI_Msk

/* HEADER 1 DEFINITIONS */
#define FDCAN_TXBH1_DLC_Pos			(16UL)
#define FDCAN_TXBH1_DLC_Msk			(0xFUL << FDCAN_TXBH1_DLC_Pos)
#define FDCAN_TXBH1_DLC             FDCAN_TXBH1_DLC_Msk
#define FDCAN_TXBH1_BRS_Pos			(20UL)
#define FDCAN_TXBH1_BRS_Msk			(0x1UL << FDCAN_TXBH1_BRS_Pos)
#define FDCAN_TXBH1_BRS				FDCAN_TXBH1_BRS_Msk
#define FDCAN_TXBH1_FDF_Pos			(21UL)
#define FDCAN_TXBH1_FDF_Msk			(0x1UL << FDCAN_TXBH1_FDF_Pos)
#define FDCAN_TXBH1_FDF				FDCAN_TXBH1_FDF_Msk
#define FDCAN_TXBH1_EFC_Pos			(23UL)
#define FDCAN_TXBH1_EFC_Msk			(0x1UL << FDCAN_TXBH1_EFC_Pos)
#define FDCAN_TXBH1_EFC				FDCAN_TXBH1_EFC_Msk
#define FDCAN_TXBH1_MM_Pos			(24UL)
#define FDCAN_TXBH1_MM_Msk			(0xFFUL << FDCAN_TXBH1_MM_Pos)
#define FDCAN_TXBH1_MM				FDCAN_TXBH1_MM_Mask

/********************** FDCAN receive buffer element bit definitions *****************************/

/* HEADER 0 DEFINITIONS */
#define FDCAN_RXBH0_EXTID_Pos 		(0UL)
#define FDCAN_RXBH0_EXTID_Msk   	(0x0FFFFFFFUL << FDCAN_RXBH0_EXTID_Pos) 
#define FDCAN_RXBH0_EXTID        	FDCAN_RXBH0_EXTID_Msk
#define FDCAN_RXBH0_STDID_Pos 		(18UL)
#define FDCAN_RXBH0_STDID_Msk  		(0x7FFUL << FDCAN_RXBH0_STDID_Pos) 
#define FDCAN_RXBH0_STDID          	FDCAN_RXBH0_STDID_Msk
#define FDCAN_RXBH0_RTR_Pos 		(29UL)
#define FDCAN_RXBH0_RTR_Msk   		(0x1UL << FDCAN_RXBH0_RTR_Pos) 
#define FDCAN_RXBH0_RTR        		FDCAN_RXBH0_RTR_Msk
#define FDCAN_RXBH1_XTD_Pos 		(30UL)
#define FDCAN_RXBH1_XTD_Msk  		(0x1UL << FDCAN_RXBH0_EXT_Pos) 
#define FDCAN_RXBH1_XTD          	FDCAN_RXBH1_EXT_Msk
#define FDCAN_RXBH1_ESI_Pos 		(31UL)
#define FDCAN_RXBH1_ESI_Msk  		(0x1UL << FDCAN_RXBH0_ESI_Pos) 
#define FDCAN_RXBH1_ESI          	FDCAN_RXBH1_ESI_Msk

/* HEADER 1 DEFINITIONS */
#define FDCAN_RXBH1_TS_Pos			(0UL)
#define FDCAN_RXBH1_TS_Msk			(0xFFFFUL << FDCAN_RXBH1_TS_Pos)
#define FDCAN_RXBH1_TS				FDCAN_RXBH1_TS_Msk
#define FDCAN_RXBH1_DLC_Pos			(16UL)
#define FDCAN_RXBH1_DLC_Msk			(0xFUL << FDCAN_RXBH1_DLC_Pos)
#define FDCAN_RXBH1_DLC        		FDCAN_RXBH1_DLC_Msk
#define FDCAN_RXBH1_BRS_Pos			(20UL)
#define FDCAN_RXBH1_BRS_Msk			(0x1UL << FDCAN_RXBH1_BRS_Pos)
#define FDCAN_RXBH1_BRS				FDCAN_RXBH1_BRS_Msk
#define FDCAN_RXBH1_FDF_Pos			(21UL)
#define FDCAN_RXBH1_FDF_Msk			(0x1UL << FDCAN_RXBH1_FDF_Pos)
#define FDCAN_RXBH1_FDF				FDCAN_RXBH1_FDF_Msk
#define FDCAN_RXBH1_FIDX_Pos		(24UL)
#define FDCAN_RXBH1_FIDX_Msk		(0x3FUL << FDCAN_RXBH1_EFC_Pos)
#define FDCAN_RXBH1_FIDX			FDCAN_RXBH1_EFC_Msk
#define FDCAN_RXBH1_ANMF_Pos		(31UL)
#define FDCAN_RXBH1_ANMF_Msk		(0x1UL << FDCAN_RXBH1_ANMF_Pos)
#define FDCAN_RXBH1_ANMF			FDCAN_TXBH1_ANMF_Mask

#endif
