#ifndef SERIAL_NEXELL_H
#define SERIAL_NEXELL_H 

#define	CFG_SYS_PLL_PMSFUNC(_FIN_, _P_, _M_, _S_)			\
	((_M_) * ((_FIN_)/((_P_)*(1UL<<(_S_)))))
#define	CFG_SYS_PLLFIN		12000000UL
#define	CFG_SYS_PLL1_P		2		//  1 <=, <= 63
#define	CFG_SYS_PLL1_M		256		// 13 <=, <= 255
#define	CFG_SYS_PLL1_S		3		// 0, 1, 2, 3
#define	CFG_SYS_PLL1_FREQ	CFG_SYS_PLL_PMSFUNC(CFG_SYS_PLLFIN, CFG_SYS_PLL1_P, CFG_SYS_PLL1_M, CFG_SYS_PLL1_S)
#define	CFG_UART_DEBUG_CLKDIV					(CFG_SYS_PLL1_FREQ/7372800) 		// PLL1 147,500,000 Hz = 40 / PLL1 192,000,000 Hz = 26
#define	CFG_UART_DEBUG_CLKFREQ					(CFG_SYS_PLL1_FREQ/CFG_UART_DEBUG_CLKDIV)
/*
 * nexell uart struct
 */
struct	NX_UART_RegisterSet
{
	volatile unsigned short	LCON;			///< 0x00 : Line Control Register
	volatile unsigned short	UCON;			///< 0x02 : Control Register
	volatile unsigned short	FCON;			///< 0x04 : FIFO Control Register
	volatile unsigned short	MCON;			///< 0x06 : Modem Control Register
	volatile unsigned short	TRSTATUS;		///< 0x08 : Tx/Rx Status Register
	volatile unsigned short	ESTATUS;		///< 0x0a : Error Status Register
	volatile unsigned short	FSTATUS;		///< 0x0c : FIFO Status Register
	volatile unsigned short	MSTATUS;		///< 0x0e : Modem Status Register
	volatile unsigned short	THB;			///< 0x10 : Transmit Buffer Register
	volatile unsigned short	RHB;			///< 0x12 : Receive Buffer Register
	volatile unsigned short	BRD;			///< 0x14 : Baud Rate Divisor Register
	volatile unsigned short	TIMEOUT;		///< 0x16 : Receive TimeOut Register
	volatile unsigned short	INTCON;			///< 0x18 : Interrupt Control Register
	volatile unsigned short	__Reserved[0x13];///< 0x1A ~ 0x3E : Reserved Region
	volatile unsigned short	CLKENB;			///< 0x40 : Clock Enable Register
	volatile unsigned short	CLKGEN;			///< 0x44 : Clock Generate Register
};
	


///	@brief Tx/Rx Status
typedef enum
{
	NX_UART_RX_BUFFER_READY		= 0x01,			///< Rx buffer contains valid data
	NX_UART_TX_BUFFER_EMPTY		= 0x02,			///< Tx buffer is empty
	NX_UART_TRANSMITTER_EMPTY	= 0x04			///< Tx buffer and the transmit shifter are empty

}	NX_UART_TXRXSTATUS ;















#endif