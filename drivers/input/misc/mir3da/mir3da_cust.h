/* For PLATFORM STANDARD
 *
 * mir3da_cust.h - Linux kernel modules for 3-Axis Accelerometer
 *
 * Copyright (C) 2011-2013 MiraMEMS Sensing Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MIR3DA_STANDARD_H__
#define __MIR3DA_STANDARD_H__

#include <linux/ioctl.h>

/* driver version info*/
#define DRI_VER                                 "1.0"

#define MIR3DA_I2C_ADDR		                    0x26//0x26<-> SD0=GND;0x27<-> SD0=High
/* Fallback axis remap when the device tree has no "layout" property.
 * The board's DT carries layout = <7>; the factory kernel blob hard-coded 6,
 * so the DT value always wins when it is present.
 */
/* nekoboot: the factory kernel hard-codes 6 for this panel; 7 makes
 * framework auto-rotate land 90 degrees off.  See HARDWARE_DIAG.md. */
#define MIR3DA_DIRECTION_NUM					6   //0 ~ 7

#define MIR3DA_ACC_IOCTL_BASE                   88
#define IOCTL_INDEX_BASE                        0x00

#define MIR3DA_ACC_IOCTL_SET_DELAY              _IOW(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE, int)
#define MIR3DA_ACC_IOCTL_GET_DELAY              _IOR(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+1, int)
#define MIR3DA_ACC_IOCTL_SET_ENABLE             _IOW(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+2, int)
#define MIR3DA_ACC_IOCTL_GET_ENABLE             _IOR(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+3, int)
#define MIR3DA_ACC_IOCTL_SET_G_RANGE            _IOW(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+4, int)
#define MIR3DA_ACC_IOCTL_GET_G_RANGE            _IOR(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+5, int)

#define MIR3DA_ACC_IOCTL_GET_COOR_XYZ           _IOW(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+22, int)
#define MIR3DA_ACC_IOCTL_CALIBRATION            _IOW(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+23, int)
#define MIR3DA_ACC_IOCTL_UPDATE_OFFSET     	    _IOW(MIR3DA_ACC_IOCTL_BASE, IOCTL_INDEX_BASE+24, int)

/* ---------------------------------------------------------------------------
 * DA217 pedometer / step-counter registers.
 *
 * Taken from the vendor register header da217.h (shipped alongside this file
 * for reference; it is NOT included from kernel code because it pulls in
 * <stdint.h>/<stdbool.h> which do not exist in the kernel).
 *
 * The register map overlaps the NSA_REG_* defines in mir3da_core.h:
 *   0x0D/0x0E  step total (MSB/LSB)
 *   0x2E       step counter reset (write 0x80)
 *   0x2F..0x33 pedometer configuration / filter / enable
 * ------------------------------------------------------------------------- */
#define DA217_REG_STEPS_MSB                     0x0D
#define DA217_REG_STEPS_LSB                     0x0E
#define DA217_REG_RESET_STEP                    0x2E
#define DA217_REG_STEP_CFG0                     0x2F
#define DA217_REG_STEP_CFG1                     0x30
#define DA217_REG_STEP_CFG2                     0x31
#define DA217_REG_STEP_CFG3                     0x32
#define DA217_REG_STEP_FILTER                   0x33

/* Pedometer setup reproduced from the factory kernel image for this board. */
#define DA217_STEP_CFG0_VALUE                   0x74
#define DA217_STEP_CFG1_VALUE                   0xE2
#define DA217_STEP_CFG2_VALUE                   0x46
#define DA217_STEP_CFG3_VALUE                   0x33
#define DA217_STEP_FILTER_VALUE                 0xA2
#define DA217_STEP_RESET_BIT                    0x80
#define DA217_STEP_ENABLE_BIT                   0x80

#endif /* !__MIR3DA_STANDARD_H__ */
