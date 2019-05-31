ifeq	($(CONFIG_PM_SLEEP),y)
	zreladdr-y	:= 0x81008000
params_phys-y	:= 0x81000100
else
	zreladdr-y	:= 0x80008000
params_phys-y	:= 0x80000100
endif
