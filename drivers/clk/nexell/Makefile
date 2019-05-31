ifeq ($(CONFIG_ARCH_NXP5540), y)
obj-y   += clk-nxp5540-axi.o clk-nxp5540-core.o clk-nxp5540-pll.o
else
obj-$(CONFIG_ARCH_S5P4418)	+= clk-s5pxx18.o clk-s5pxx18-pll.o
obj-$(CONFIG_ARCH_NXP2120)	+= clk-nxp2120-pll.o clk-nxp2120.o
endif
