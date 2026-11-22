#
# Makefile for the MiraMEMS MIR3DA / DA217 3-axis accelerometer driver.
#
# The module is called mir3da.ko so that /sys/module/mir3da/version reports
# MODULE_VERSION("1.0"), matching the factory kernel.
#
obj-$(CONFIG_SENSORS_MIR3DA)		+= mir3da.o
mir3da-objs				:= mir3da_core.o mir3da_cust.o
CFLAGS_mir3da_cust.o += -Wno-error=unused-variable -Wno-error=unused-but-set-variable
