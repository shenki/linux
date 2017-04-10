#
# Makefile for the linux kernel.
#

obj-y 			+= timer.o devs.o ast-ahbc.o

obj-$(CONFIG_IRMP)	+= ast-bmc-scu.o

obj-$(CONFIG_RT360_CAM) += ast-cam-scu.o

obj-$(CONFIG_ARCH_AST1070) += ast1070-irq.o ast1070-scu.o ast1070-i2c-irq.o dev-ci2c.o dev-cuart.o dev-clpc.o 

obj-$(CONFIG_AST2400_BMC) += ast2400-irq.o ast2400-scu.o dev-ast2400-uart.o #dev-ast2400-i2c.o

#obj-n   := dummy.o
#platform
obj-y			+= dev-uart.o

#Storage
obj-y 	                += dev-nor.o dev-nand.o 

#bus
obj-y                   += dev-i2c.o dev-spi.o dev-ehci.o dev-uhci.o 

#video
obj-y			+= dev-fb.o dev-rfx.o 
