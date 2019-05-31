ccflags-$(CONFIG_ARCH_MULTIPLATFORM) += -I$(srctree)/$(src)/include
ccflags-$(CONFIG_ARCH_NXP2120) += -I$(srctree)/arch/arm/mach-nxp2120/include
#ccflags-$(CONFIG_NXP4330_MCU_S) += -I$(srctree)/arch/arm/mach-s5p4418/include

obj-$(CONFIG_ARCH_NXP2120) := cpu.o  #nxp2120.o timer.o irq.o
obj-$(CONFIG_ARCH_NXP2120) += soc_module.o nx_clkpwr.o nx_mcud.o nx_mcus.o #mcus_module.o nx_mcus.o 
#obj-$(CONFIG_DEBUG_NXP2120_UART)	+= lldebug.o
obj-$(CONFIG_SMP) += platsmp.o #headsmp.o
#obj-$(CONFIG_SECURE_REG_ACCESS) += sec_reg.o reg-call.o
#obj-$(CONFIG_NXP4330_MCU_S) += mcus_module.o nx_mcus.o 