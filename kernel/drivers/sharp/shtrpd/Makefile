ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHTRPD_FACTORY_MODE_ENABLE
endif

obj-$(CONFIG_TRACKPAD_SHTRPD) += shtrpd.o
