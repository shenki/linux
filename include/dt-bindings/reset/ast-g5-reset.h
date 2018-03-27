/*
 * This header provides ASPEED AST SOC constants for binding
 */

#ifndef _DT_BINDINGS_RESET_AST_G5_H
#define _DT_BINDINGS_RESET_AST_G5_H

/* AST_SCU_RESET : 0x04	 - system reset control register */
#define SCU_RESET_H264				(26) 
#define SCU_RESET_XDMA				(25)
#define SCU_RESET_MCTP				(24)
#define SCU_RESET_P2X				(24)
#define SCU_RESET_ADC				(23)
#define SCU_RESET_JTAG				(22)
#define SCU_RESET_PCIE_DIR			(21)
#define SCU_RESET_PCIE				(19)
#define SCU_RESET_MIC				(18)
#define SCU_RESET_RFX				(17) 
#define SCU_RESET_SDHCI				(16)
#define SCU_RESET_USB11				(15)
#define SCU_RESET_USB20				(14)
#define SCU_RESET_CRT				(13)
#define SCU_RESET_MAC1				(12)
#define SCU_RESET_MAC0				(11)
#define SCU_RESET_PECI				(10)
#define SCU_RESET_PWM				(9)
#define SCU_PCI_VGA_DIS				(8)
#define SCU_RESET_2D				(7)
#define SCU_RESET_VIDEO				(6)
#define SCU_RESET_LPC				(5)
#define SCU_RESET_ESPI				(5)
#define SCU_RESET_HACE				(4)
#define SCU_RESET_USB_P1			(3)
#define SCU_RESET_I2C				(2)
#define SCU_RESET_AHB				(1)
#define SCU_RESET_SRAM_CTRL			(0)

/* AST_SCU_RESET2 : 0xD4 - Reset Control register set 2	*/
#define SCU_RESET_CRT3				(40)
#define SCU_RESET_CRT2				(39)
#define SCU_RESET_CRT1				(38)
#define SCU_RESET_CRT0				(37)
#define SCU_RESET_NIC1				(36)
#define SCU_RESET_NIC0				(35)
#define SCU_RESET_RFXDEC			(34)
#define SCU_RESET_BITBLT			(33)
#define SCU_RESET_RFXCMQ			(32)


#endif	/* _DT_BINDINGS_RESET_AST_G5_H */
