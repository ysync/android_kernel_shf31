# miyabi
ifeq ($(CONFIG_MACH_EBZ),y)
obj-$(CONFIG_SECURITY_MIYABI)		+= miyabi.o mnhelper.o
else ifeq ($(CONFIG_MACH_PA25),y)
obj-$(CONFIG_SECURITY_MIYABI)		+= miyabi.o mnhelper.o
else
obj-$(CONFIG_SECURITY_MIYABI)		+= miyabi.o
endif

ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DLOCAL_MIYABI_FBUILD
endif

