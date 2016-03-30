# Kernel makefile for motions sensors
#
# 

ifeq ($(CONFIG_MPU_SENSORS_MPU6515), y)
obj-$(CONFIG_MPU_SENSORS_MPU6515)	+= mpu6515/
else
obj-$(CONFIG_SHMDS)	+= motionsensor/
endif
