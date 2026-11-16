/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _RIHWACCESS_H
#define _RIHWACCESS_H

#include <zephyr/arch/cpu.h>

#define READ8(address)           sys_read8(address)
#define READ16(address)          sys_read16(address)
#define READ32(address)          sys_read32(address)
#define READ64(address)          sys_read64(address)
#define WRITE8(address,data)     sys_write8(data, address)
#define WRITE16(address,data)    sys_write16(data, address)
#define WRITE32(address,data)    sys_write32(data, address)
#define WRITE64(address,data)    sys_write64(data, address)

#define OR8(address,data)        WRITE8(address, READ8(address) | (data))
#define AND8(address,data)       WRITE8(address, READ8(address) & (data))
#define OR16(address,data)       WRITE16(address, READ16(address) | (data))
#define AND16(address,data)      WRITE16(address, READ16(address) & (data))
#define OR32(address,data)       WRITE32(address, READ32(address) | (data))
#define AND32(address,data)      WRITE32(address, READ32(address) & (data))

// replace a bit field, where the field is specified by field containing '1's
// E.g. MODIFYREG(address, field=000 111 000, data=000 101 000);
//      this replaces bits 5:3 with <data> : assuming current=110 001 011, field=000 111 000, data=000 101 000, then the result=110 101 011
#define MODIFYREG(address,field,data) WRITEREG(address, (READREG(address) & ~(field)) | (data))
#define MODIFY16(address,field,data) WRITE16(address, (READ16(address) & ~(field)) | (data))
// register access may be 16bit or 32bit (depending on the CPU or bus), so we define an independent wrapper
#define READREG        READ32
#define WRITEREG       WRITE32
// #  define WRITEREG       WRITEREG2
#define ORREG          OR32
#define ANDREG         AND32
#define READREG_DEV    READ32_DEV
#define WRITEREG_DEV   WRITE32_DEV


#define WRITEREG_LOG(reg_name, write_reg, reg_value)                                        \
    WRITEREG(write_reg, reg_value);                                                         \
    LOG_DBG("%s\t0x%08X\t->\t0x%08X", reg_name, write_reg, reg_value);                      \

#endif // _RIHWACCESS_H