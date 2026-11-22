# ---------------------------------------------------------------------------
# Hynitron touch driver -- MUTCAP/SELCAP family (hynitron_core.*)
#
# Device DT node:  i2c@70600000/hynitron@1a
#     compatible = "hynitron,hyn_ts"      reg = <0x1a>
# reg 0x1a means "mutcap" (see docs/hynitron-ts.txt).
#
# This is the family the FACTORY kernel used: its .rodata contains
#   "Hynitron MUTCAP touchscreen driver 1.0."
#   "Hynitron SELCAP touchscreen driver 1.0."
#   capacitive_hynitron_cst{0xx,1xxse,2xxse,3xx,3xxse,3240,6xx,8xx,9xx}_update
# and hynitron,have-key / key-number / key-code / key-x-coord / key-y-coord.
#
# The previous Makefile built the V2.23 family (hyn_core.o + hyn_lib/ +
# hyn_chips/).  That family's of_match_table has NO "hynitron,hyn_ts" entry
# and it has no CST9xx chip backend at all, so it registered an i2c driver
# named "hyn_ts" that could never bind -> touchscreen dead, with no error in
# the log (a failed of-match is silent).
#
# Both families register an i2c driver literally named "hyn_ts" and share
# helper symbol names, so they must NOT be built together.
# ---------------------------------------------------------------------------

obj-y += hynitron_ts.o

hynitron_ts-objs += hynitron_core.o
hynitron_ts-objs += hynitron_update_firmware.o
hynitron_ts-objs += hynitron_i2c.o
hynitron_ts-objs += hynitron_tool_debug.o
hynitron_ts-objs += hynitron_esd_check.o
hynitron_ts-objs += hynitron_gesture.o
hynitron_ts-objs += hynitron_proximity.o
