#define USE_WR_DELAY
/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016-2017 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <stdio.h>
#include <string.h>

#include "py/obj.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/machine_qspi.h"
#include "drivers/memory/qspiflash.h"
#include "led.h"
#include "lib/oofatfs/ff.h"
#include "extmod/vfs_fat.h"
#include "genhdr/pins.h"
#include "irq.h"

#ifdef MICROPY_HW_QSPIFLASH_SIZE_BITS
//#define CMD_WRITE (0x02)
//#define CMD_READ (0x03)
//#define CMD_WRDI (0x04)
#define CMD_READ (0x03)
#define CMD_QREAD (0x6b)
#define CMD_C4READ (0xeb)
#define CMD_SDIOR (0xbb)
#define CMD_SDOR (0x3b)
#define CMD_DREAD (0x3b)
#define CMD_PP (0x02)
#define CMD_QPP (0x32)
#define CMD_RDSR (0x05)
#if defined(MICROPY_HW_QSPIFLASH_CR16) && (MICROPY_HW_QSPIFLASH_CR16 == 1)
#define CMD_RDCR (0x15)
#else
#define CMD_RDCR (0x35)
#endif
#define CMD_WRSR (0x01)
#define CMD_WREN (0x06)
#define CMD_SEC_ERASE (0x20)
#define CMD_CHIP_ERASE (0xc7)
#define CMD_RDSFDP (0x5a)

#define CMD_RBPR (0x72)
#define CMD_WBPR (0x42)
#define CMD_EQIO (0x38)
#define CMD_RSTQIO (0xff)

#define CMD_RD_SECID (0x88)
#define CMD_ULBPR (0x98)

#define CMD_RDSCUR (0x2b)

#define WAIT_SR_TIMEOUT (1000000)

#define PAGE_SIZE (256) // maximum bytes we can write in one QSPI transfer
#define SECTOR_SIZE (4096) // size of erase sector

// Note: this code is not reentrant with this shared buffer
STATIC uint8_t buf[SECTOR_SIZE];

#if DO_VERIFY
STATIC uint8_t rbuf[SECTOR_SIZE];
#endif
STATIC uint32_t bufsec = 0xffffffff;

STATIC void mp_qspiflash_transfer(mp_qspiflash_t *self, size_t len, const uint8_t *src, uint8_t *dest);
STATIC void mp_qspiflash_write_cmd(mp_qspiflash_t *self, uint8_t cmd);
#ifdef xMICROPY_HW_QSPI_QE_MASK
STATIC void mp_qspiflash_write_cmd1(mp_qspiflash_t *self, uint8_t cmd);
#endif
STATIC int mp_qspiflash_erase_sector(mp_qspiflash_t *self, uint32_t addr);
STATIC int mp_qspiflash_set_QE(mp_qspiflash_t *self);
STATIC int mp_qspiflash_get_JEDEC(mp_qspiflash_t *self, void *buf, int offset, int n);
STATIC uint16_t mp_qspiflash_get_SR(mp_qspiflash_t *self);
STATIC uint16_t mp_qspiflash_get_CR(mp_qspiflash_t *self);
#ifdef MICROPY_HW_QSPIFLASH_SIO3
STATIC int mp_qspiflash_set_CR(mp_qspiflash_t *self, uint32_t cr);
#endif
#if defined(MICROPY_HW_QSPIFLASH_NEED_WBPR) && (MICROPY_HW_QSPIFLASH_NEED_WBPR == 1)
STATIC void mp_qspiflash_wbpr(mp_qspiflash_t *self);
#endif

#define CS_LOW mp_hal_pin_write(self->qspi.cs, 0)
#define CS_HIGH mp_hal_pin_write(self->qspi.cs, 1)

STATIC /*const*/ mp_qspiflash_t qspiflash = {
    .qspi = {
        .base = {&mp_machine_soft_qspi_type},
        .cs = &MICROPY_HW_QSPIFLASH_CS,
        .sck = &MICROPY_HW_QSPIFLASH_SCK,
        .sio0 = &MICROPY_HW_QSPIFLASH_SIO0,
        .sio1 = &MICROPY_HW_QSPIFLASH_SIO1,
#if defined(MICROPY_HW_QSPIFLASH_SIO2)
        .sio2 = &MICROPY_HW_QSPIFLASH_SIO2,
        .sio3 = &MICROPY_HW_QSPIFLASH_SIO3,
#endif
    },
};

STATIC uint32_t dummies = 6;

void mp_qspiflash_init(mp_qspiflash_t *self) {
    //printf("%s\n", __FUNCTION__);
    CS_HIGH;
    mp_hal_pin_output(self->qspi.cs);
    mp_hal_pin_write(self->qspi.sck, 0);
    mp_hal_pin_output(self->qspi.sck);

    mp_hal_pin_output(self->qspi.sio0);
    mp_hal_pin_input(self->qspi.sio1);
    mp_hal_pin_write(self->qspi.sio2, 1);
    mp_hal_pin_output(self->qspi.sio2);
    mp_hal_pin_write(self->qspi.sio3, 1);
    mp_hal_pin_output(self->qspi.sio3);

#if defined(MICROPY_HW_QSPIFLASH_NEED_WBPR) && (MICROPY_HW_QSPIFLASH_NEED_WBPR == 1)
    mp_qspiflash_write_cmd(self, CMD_RSTQIO);
    mp_qspiflash_write_cmd(self, CMD_RSTQIO);
    self->flags &= ~0x80;
    //mp_qspiflash_write_cmd1(self, CMD_EQIO);
    //mp_qspiflash_wbpr(self);
#endif
#ifndef MICROPY_HW_QSPIFLASH_SIO3
    self->flags |= 0x40;
#endif
    // try to get JEDEC description
    uint32_t buf[4];
    mp_qspiflash_get_JEDEC(self, buf, 0x30, 16);

#if defined(MICROPY_HW_QSPIFLASH_NEED_WBPR) && (MICROPY_HW_QSPIFLASH_NEED_WBPR == 1)
    //mp_qspiflash_write_cmd1(self, CMD_EQIO);
    //self->flags |= 0x80;
    //mp_qspiflash_wbpr(self);
    printf("%02x %02x\n", mp_qspiflash_get_SR(self), mp_qspiflash_get_CR(self));
#ifdef MICROPY_HW_QSPIFLASH_SIO3
    mp_qspiflash_set_QE(self);
#endif
#else
#ifdef xMICROPY_HW_QSPIFLASH_SIO3
    // make sure QE flag is set!
    if (mp_qspiflash_set_QE(self)) {
        printf("No QSPI device attached!\n");
        //return -MP_EIO;
    }
#else
    if (1 == 0) {
        mp_qspiflash_set_QE(self);
    }
#endif
#endif
    if (1) {
        uint16_t sr = mp_qspiflash_get_SR(self);
        uint16_t cr = mp_qspiflash_get_CR(self);
        #ifdef MICROPY_HW_QSPIFLASH_SIO3
        mp_qspiflash_set_CR(self, MICROPY_HW_QSPIFLASH_CR);
        #endif
        uint16_t crn = mp_qspiflash_get_CR(self);
        printf("QSPI device found! %ld kB %04x %04x %04x\n", (buf[1] + 1) >> 13, sr, cr, crn);
        dummies = 6 + ((crn >> 5) & 2);
    }

#if defined(MICROPY_HW_QSPIFLASH_NEED_WBPR) && (MICROPY_HW_QSPIFLASH_NEED_WBPR == 1)
    mp_qspiflash_wbpr(self);
    //mp_qspiflash_write_cmd(self, CMD_EQIO);
#else
    //mp_qspiflash_wbpr(self);
#endif
    self->flags &= ~1;
    // indicate a clean cache with LED off
    led_state(PYB_LED_RED, 0);
}

volatile uint32_t qspi_sema = 0;
volatile uint32_t qspi_sema_block_cnt = 0;
#define SECTOR_CNT 512
volatile uint32_t erase_histo[SECTOR_CNT];
volatile uint32_t basepri;

STATIC void mp_qspiflash_acquire_bus(mp_qspiflash_t *self) {
    // can be used for actions needed to acquire bus
    (void)self;

    __disable_irq();
    if (qspi_sema & 1) {
        qspi_sema_block_cnt += 1;
        while (1) {
            __enable_irq();
            __WFI();
            __disable_irq();
            if ((qspi_sema & 1) == 0) {
                break;
            }
        }
    }
    // we must disable USB irqs to prevent MSC contention with QSPI flash
    basepri = raise_irq_pri(IRQ_PRI_OTG_FS);

    qspi_sema |= 1;
    __enable_irq();
    // enable writes
}

STATIC void mp_qspiflash_release_bus(mp_qspiflash_t *self) {
    // can be used for actions needed to release bus
    (void)self;

    __disable_irq();
    qspi_sema &= ~1;
    restore_irq_pri(basepri);
    __enable_irq();
}

STATIC void mp_qspiflash_transfer(mp_qspiflash_t *self, size_t len, const uint8_t *src, uint8_t *dest) {
    mp_machine_soft_qspi_transfer(&self->qspi.base, len, src, dest);
}

STATIC void mp_qspiflash_qread(mp_qspiflash_t *self, size_t len, uint8_t *dest) {
    mp_machine_soft_qspi_qread(&self->qspi.base, len, dest);
}

STATIC void mp_qspiflash_dread(mp_qspiflash_t *self, size_t len, uint8_t *dest) {
    mp_machine_soft_qspi_dread(&self->qspi.base, len, dest);
}

STATIC void mp_qspiflash_qwrite(mp_qspiflash_t *self, size_t len, const uint8_t *src) {
    mp_machine_soft_qspi_qwrite(&self->qspi.base, len, src);
}

STATIC int mp_qspiflash_wait_sr(mp_qspiflash_t *self, uint8_t mask, uint8_t val, uint32_t timeout) {
    uint32_t cmd = CMD_RDSR;
    CS_LOW;
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        mp_qspiflash_qwrite(self, 2, (uint8_t *)&cmd);
        for (; timeout; --timeout) {
            mp_qspiflash_qread(self, 1, (uint8_t *)&cmd);
            if ((cmd & mask) == val) {
                break;
            }
        }
    } else {
        mp_qspiflash_transfer(self, 1, (uint8_t *)&cmd, NULL);
        for (; timeout; --timeout) {
            mp_qspiflash_transfer(self, 1, NULL, (uint8_t *)&cmd);
            if ((cmd & mask) == val) {
                break;
            }
        }
    }
#else
    //mp_qspiflash_transfer(self, 1, cmd, NULL);
    mp_qspiflash_transfer(self, 2, (uint8_t *)&cmd, NULL);
    for (; timeout; --timeout) {
        mp_qspiflash_transfer(self, 1, (uint8_t *)&cmd, (uint8_t *)&cmd);
        if ((cmd & mask) == val) {
            break;
        }
    }
#endif
    CS_HIGH;
    if ((cmd & mask) == val) {
        return 0; // success
    } else if (timeout == 0) {
        return -MP_ETIMEDOUT;
    } else {
        return -MP_EIO;
    }
}

STATIC int mp_qspiflash_wait_wel1(mp_qspiflash_t *self) {
    return mp_qspiflash_wait_sr(self, 2, 2, WAIT_SR_TIMEOUT);
}

STATIC int mp_qspiflash_wait_wip0(mp_qspiflash_t *self) {
    return mp_qspiflash_wait_sr(self, 1, 0, WAIT_SR_TIMEOUT);
}

#ifdef xMICROPY_HW_QSPI_QE_MASK
STATIC void mp_qspiflash_write_cmd1(mp_qspiflash_t *self, uint8_t cmd) {
    CS_LOW;
    mp_qspiflash_transfer(self, 1, &cmd, NULL);
    CS_HIGH;
}
#endif

STATIC void mp_qspiflash_write_cmd(mp_qspiflash_t *self, uint8_t cmd) {
    CS_LOW;
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        mp_qspiflash_qwrite(self, 1, &cmd);
    } else {
        mp_qspiflash_transfer(self, 1, &cmd, NULL);
    }
#else
    mp_qspiflash_transfer(self, 1, &cmd, NULL);
#endif
    CS_HIGH;
}

#ifdef MICROPY_HW_QSPI_QE_MASK
#define QSPI_QE_MASK MICROPY_HW_QSPI_QE_MASK
#else
#define QSPI_QE_MASK 0x40
#endif

STATIC uint16_t mp_qspiflash_get_CR(mp_qspiflash_t *self) {
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        uint32_t cmd = CMD_RDCR;
        CS_LOW;
        mp_qspiflash_qwrite(self, 1, (uint8_t *)&cmd);
        mp_qspiflash_qread(self, 2, (uint8_t *)&cmd);
        CS_HIGH;
        return (cmd >> 8) & 0xff;
    } else {
        uint32_t cmd = CMD_RDCR;
        CS_LOW;
        mp_qspiflash_transfer(self, 2, (uint8_t *)&cmd, (uint8_t *)&cmd);
        CS_HIGH;
        return (cmd >> 8) & 0xff;
    }
#else
    uint32_t cmd = CMD_RDCR;
    CS_LOW;
    mp_qspiflash_transfer(self, 3, (uint8_t *)&cmd, (uint8_t *)&cmd);
    CS_HIGH;
    return (cmd >> 8) & 0xffff;
#endif
}

STATIC uint16_t mp_qspiflash_get_SR(mp_qspiflash_t *self) {
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        uint32_t cmd = CMD_RDSR;
        CS_LOW;
        mp_qspiflash_qwrite(self, 2, (uint8_t *)&cmd);
        mp_qspiflash_qread(self, 1, (uint8_t *)&cmd);
        CS_HIGH;
        return cmd & 0xff;
    } else {
        uint32_t cmd = CMD_RDSR;
        CS_LOW;
        mp_qspiflash_transfer(self, 2, (uint8_t *)&cmd, (uint8_t *)&cmd);
        CS_HIGH;
        return (cmd >> 8) & 0xff;
    }
#else
    uint32_t cmd = CMD_RDSR;
    CS_LOW;
    mp_qspiflash_transfer(self, 3, (uint8_t *)&cmd, (uint8_t *)&cmd);
    CS_HIGH;
    return (cmd >> 8) & 0xffff;
#endif
}

STATIC int mp_qspiflash_get_JEDEC(mp_qspiflash_t *self, void *buf, int offset, int n) {
    uint8_t cmd[5] = {CMD_RDSFDP, 0, 0, 0, 0};
    cmd[1] = (offset >> 16) & 0xff;
    cmd[2] = (offset >> 8) & 0xff;
    cmd[3] = offset & 0xff;

    CS_LOW;
    mp_qspiflash_transfer(self, 5, (uint8_t *)cmd, (uint8_t *)cmd);
    mp_qspiflash_transfer(self, n, (uint8_t *)buf, (uint8_t *)buf);
    CS_HIGH;
    printf("JEDEC[%x]:", offset);
    for (int i = 0; i < n; i++) {
        printf(" %02x", ((uint8_t *)buf)[i]);
    }
    printf("\n");
    return 1;
}

#ifdef MICROPY_HW_QSPIFLASH_SIO3
STATIC int mp_qspiflash_set_QE(mp_qspiflash_t *self) {
#ifdef MICROPY_HW_QSPI_QE_MASK
    return 0;
    uint32_t cmd = CMD_WRSR;
    cmd |= mp_qspiflash_get_SR(self) << 8;
    cmd |= mp_qspiflash_get_CR(self) << 16;
    if (cmd & (QSPI_QE_MASK << 16)) {
        return 0;
    } else {
        cmd |= QSPI_QE_MASK << 16;
        mp_qspiflash_write_cmd(self, CMD_WREN);
        CS_LOW;
        if (self->flags & 0x80) {
            mp_qspiflash_qwrite(self, 3, (uint8_t *)&cmd);
        } else if (self->flags & 0x40) {
            // dual mode
            cmd &= ~(QSPI_QE_MASK << 16);
            mp_qspiflash_transfer(self, 3, (uint8_t *)&cmd, NULL);
        } else {
            mp_qspiflash_transfer(self, 3, (uint8_t *)&cmd, NULL);
        }
        CS_HIGH;
        return mp_qspiflash_wait_wip0(self);
    }
#else
    uint32_t cmd = CMD_RDSR;
    CS_LOW;
    mp_qspiflash_transfer(self, 3, (uint8_t *)&cmd, (uint8_t *)&cmd);
    CS_HIGH;
    if (cmd & (QSPI_QE_MASK << 8)) {
        return 0;
    } else {
        mp_qspiflash_write_cmd(self, CMD_WREN);
        cmd |= QSPI_QE_MASK << 8;
        cmd = (cmd & ~0xff) | CMD_WRSR;
        CS_LOW;
        mp_qspiflash_transfer(self, 3, (uint8_t *)&cmd, (uint8_t *)&cmd);
        CS_HIGH;
        return mp_qspiflash_wait_wip0(self);
    }
#endif
}
#endif

#if defined(MICROPY_HW_QSPIFLASH_NEED_WBPR) && (MICROPY_HW_QSPIFLASH_NEED_WBPR == 1)
STATIC void mp_qspiflash_wbpr(mp_qspiflash_t *self) {
    uint8_t cmd[7] = {CMD_RBPR, 0, 0, 0, 0, 0, 0};
    //uint8_t cmd2[7] = {CMD_WBPR, 1, 2, 3, 4, 0x55, 0xaa};
    uint8_t cmd2[7] = {CMD_WBPR};
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        CS_LOW;
        mp_qspiflash_qwrite(self, 2, (uint8_t *)cmd);
        mp_qspiflash_qread(self, 6, (uint8_t *)&cmd[1]);
        CS_HIGH;
        mp_qspiflash_write_cmd(self, CMD_WREN);
        CS_LOW;
        mp_qspiflash_qwrite(self, 7, (uint8_t *)cmd2);
        CS_HIGH;
        CS_LOW;
        mp_qspiflash_qwrite(self, 2, (uint8_t *)cmd);
        mp_qspiflash_qread(self, 6, (uint8_t *)&cmd2[1]);
        CS_HIGH;
    } else {
        CS_LOW;
        mp_qspiflash_transfer(self, 7, (uint8_t *)cmd, (uint8_t *)cmd);
        CS_HIGH;
        mp_qspiflash_write_cmd(self, CMD_WREN);
        CS_LOW;
        mp_qspiflash_transfer(self, 7, (uint8_t *)cmd2, (uint8_t *)cmd2);
        CS_HIGH;
        cmd[0] = CMD_RBPR;
        CS_LOW;
        mp_qspiflash_transfer(self, 7, (uint8_t *)cmd, (uint8_t *)cmd2);
        CS_HIGH;
    }
#else
    CS_LOW;
    mp_qspiflash_transfer(self, 7, (uint8_t *)cmd, (uint8_t *)cmd);
    CS_HIGH;
#endif
    printf("BPR:");
    for (int i = 0; i < sizeof(cmd); i++) {
        printf(" %02x", cmd[i]);
    }
    printf("\n");
    printf("    ");
    for (int i = 0; i < sizeof(cmd2); i++) {
        printf(" %02x", cmd2[i]);
    }
    printf("\n");
}
#endif

#ifdef MICROPY_HW_QSPIFLASH_SIO3
STATIC int mp_qspiflash_set_CR(mp_qspiflash_t *self, uint32_t cr) {
#ifdef MICROPY_HW_QSPI_QE_MASK
    uint32_t sr = CMD_WRSR | (mp_qspiflash_get_SR(self) << 8) | (cr << 16);
    mp_qspiflash_write_cmd(self, CMD_WREN);
    CS_LOW;
    if (self->flags & 0x80) {
        mp_qspiflash_qwrite(self, 3, (uint8_t *)&sr);
    } else {
        mp_qspiflash_transfer(self, 3, (uint8_t *)&sr, NULL);
    }
    CS_HIGH;
#else
    uint32_t cmd = CMD_RDSR;
    mp_qspiflash_write_cmd(self, CMD_WREN);
    int ret = mp_qspiflash_wait_wel1(self);
    if (ret != 0) {
        printf("ERR %s %d\n", __func__, ret);
        return ret;
    }
    CS_LOW;
    mp_qspiflash_transfer(self, 3, (uint8_t *)&cmd, (uint8_t *)&cmd);
    CS_HIGH;
    cmd = CMD_WRSR | ((cmd >> 8) & 0xff00) | (cr << 16);
    CS_LOW;
    mp_qspiflash_transfer(self, 4, (uint8_t *)&cmd, NULL);
    CS_HIGH;
#endif
    return mp_qspiflash_wait_wip0(self);
}
#endif

STATIC int mp_qspiflash_erase_sector(mp_qspiflash_t *self, uint32_t addr) {
    // enable writes
    mp_qspiflash_write_cmd(self, CMD_WREN);

    // wait WEL=1
    int ret = mp_qspiflash_wait_wel1(self);
    if (ret != 0) {
        printf("ERR %s %d\n", __func__, ret);
        return ret;
    }

    if ((addr >> 12) < SECTOR_CNT) {
        erase_histo[addr >> 12] += 1;
    }

    uint8_t cmd[4] = {CMD_SEC_ERASE, addr >> 16, addr >> 8, addr};
    // erase the sector
    CS_LOW;
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        mp_qspiflash_qwrite(self, 4, cmd);
    } else {
        mp_qspiflash_transfer(self, 4, cmd, NULL);
    }
#else
    mp_qspiflash_transfer(self, 4, cmd, NULL);
#endif
    CS_HIGH;
    // wait WIP=0
    return mp_qspiflash_wait_wip0(self);
}

STATIC int mp_qspiflash_erase_all(mp_qspiflash_t *self) {
    // enable writes
    mp_qspiflash_write_cmd(self, CMD_WREN);

    // wait WEL=1
    int ret = mp_qspiflash_wait_wel1(self);
    if (ret != 0) {
        printf("ERR %s %d\n", __func__, ret);
        return ret;
    }

    mp_qspiflash_write_cmd(self, CMD_CHIP_ERASE);

    // wait WIP=0
    return mp_qspiflash_wait_wip0(self);
}

STATIC int mp_qspiflash_sec_id(mp_qspiflash_t *self) {
#ifdef MICROPY_HW_QSPI_QE_MASK
    uint8_t cmd[16] = {CMD_RD_SECID, 0, 0, 0, 0, 0 };
    // erase the sector
    CS_LOW;
    if (self->flags & 0x80) {
        mp_qspiflash_qwrite(self, 6, cmd);
        mp_qspiflash_qread(self, 16, cmd);
    } else {
        mp_qspiflash_transfer(self, 6, cmd, NULL);
        mp_qspiflash_transfer(self, 16, NULL, cmd);
    }
    CS_HIGH;
    printf("SEC_ID:");
    for (int i = 0; i < 16; i++) {
        printf(" %02x", cmd[i]);
    }
    printf("\n");
#else
    uint32_t cmd = CMD_RDSCUR;
    mp_qspiflash_transfer(self, 2, (uint8_t *)&cmd, (uint8_t *)&cmd);
    printf("SEC_REG: %02lx\n", (cmd >> 8) & 0xff);
#endif
    return 0;
}

STATIC int mp_qspiflash_ulbpr(mp_qspiflash_t *self) {
#ifdef MICROPY_HW_QSPI_QE_MASK
    uint8_t cmd[1] = {CMD_ULBPR};
    mp_qspiflash_write_cmd(self, CMD_WREN);
    CS_LOW;
    mp_qspiflash_qwrite(self, 1, cmd);
    CS_HIGH;
#endif
    return 0;
}

STATIC int mp_qspiflash_write_page(mp_qspiflash_t *self, uint32_t addr, const uint8_t *src) {
    // enable writes
    mp_qspiflash_write_cmd(self, CMD_WREN);

    // wait WEL=1
    int ret = mp_qspiflash_wait_wel1(self);
    if (ret != 0) {
        return ret;
    }

    // write the page
    CS_LOW;
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        uint8_t cmd[4] = {CMD_PP, addr >> 16, addr >> 8, addr};
        mp_qspiflash_qwrite(self, 4, cmd);
        mp_qspiflash_qwrite(self, PAGE_SIZE, src);
    } else if (self->flags & 0x40) {
        // dual mode
        uint8_t cmd[4] = {CMD_PP, addr >> 16, addr >> 8, addr};
        mp_qspiflash_transfer(self, 4, cmd, NULL);
        mp_qspiflash_transfer(self, PAGE_SIZE, src, NULL);
    } else {
        uint8_t cmd[4] = {CMD_QPP, addr >> 16, addr >> 8, addr};
        mp_qspiflash_transfer(self, 1, cmd, NULL);
        mp_qspiflash_qwrite(self, 3, &cmd[1]);
        mp_qspiflash_qwrite(self, PAGE_SIZE, src);
    }
#else
    #if defined(MICROPY_HW_QSPIFLASH_QSPI) && (MICROPY_HW_QSPIFLASH_QSPI == 4)
    uint8_t cmd[4] = {CMD_QPP, addr >> 16, addr >> 8, addr};
    mp_qspiflash_transfer(self, 1, cmd, NULL);
    mp_qspiflash_qwrite(self, 3, &cmd[1]);
    mp_qspiflash_qwrite(self, PAGE_SIZE, src);
    #else
    if (1 == 0) {
        mp_qspiflash_qwrite(self, PAGE_SIZE, src);
    }
    uint8_t cmd[4] = {CMD_PP, addr >> 16, addr >> 8, addr};
    mp_qspiflash_transfer(self, 4, cmd, NULL);
    mp_qspiflash_transfer(self, PAGE_SIZE, src, NULL);
    #endif
#endif
    CS_HIGH;

    // wait WIP=0
    return mp_qspiflash_wait_wip0(self);
}

void mp_qspiflash_read_raw(mp_qspiflash_t *self, uint32_t addr, size_t len, uint8_t *dest) {
    //mp_qspiflash_acquire_bus(self);
#ifdef MICROPY_HW_QSPI_QE_MASK
    if (self->flags & 0x80) {
        uint8_t cmd[7] = {CMD_C4READ, addr >> 16, addr >> 8, addr};
        CS_LOW;
        mp_qspiflash_transfer(self, 1, cmd, NULL);
        mp_qspiflash_qwrite(self, 6, &cmd[1]);
        mp_qspiflash_qread(self, len, dest);
        CS_HIGH;
    } else if (self->flags & 0x40) {
        // dual mode
        uint8_t cmd[5] = {CMD_SDOR, addr >> 16, addr >> 8, addr};
        CS_LOW;
        mp_qspiflash_transfer(self, 5, cmd, NULL);
        mp_qspiflash_dread(self, len, dest);
        CS_HIGH;
    } else {
        uint8_t cmd[5] = {CMD_QREAD, addr >> 16, addr >> 8, addr};
        CS_LOW;
        mp_qspiflash_transfer(self, 5, cmd, NULL);
        mp_qspiflash_qread(self, len, dest);
        CS_HIGH;
    }
#else
    if (1 == 0) {
        mp_qspiflash_dread(self, len, dest);
        mp_qspiflash_qread(self, len, dest);
    }
    CS_LOW;
    #if defined(MICROPY_HW_QSPIFLASH_QSPI) && (MICROPY_HW_QSPIFLASH_QSPI == 4)
    //uint8_t cmd[9] = {CMD_C4READ, addr >> 16, addr >> 8, addr, 0, 0, 0, 0, 0};
    uint8_t cmd[5] = {CMD_QREAD, addr >> 16, addr >> 8, addr, 0};
    CS_LOW;
    mp_qspiflash_transfer(self, 5, cmd, NULL);
    mp_qspiflash_qread(self, len, dest);
    CS_HIGH;
    #elif defined(MICROPY_HW_QSPIFLASH_QSPI) && (MICROPY_HW_QSPIFLASH_QSPI == 2)
    uint8_t cmd[5] = {CMD_DREAD, addr >> 16, addr >> 8, addr, 0};
    CS_LOW;
    mp_qspiflash_transfer(self, 5, cmd, NULL);
    mp_qspiflash_dread(self, len, dest);
    CS_HIGH;
    #else
    uint8_t cmd[4] = {CMD_READ, addr >> 16, addr >> 8, addr};
    mp_qspiflash_transfer(self, 4, cmd, NULL);
    mp_qspiflash_transfer(self, len, NULL, dest);
    CS_HIGH;
    #endif
#endif
}

void mp_qspiflash_read(mp_qspiflash_t *self, uint32_t addr, size_t len, uint8_t *dest) {
    if (len <= 0) {
        return;
    }
    mp_qspiflash_acquire_bus(self);
    if (bufsec != 0xffffffff) {
        uint32_t bis = addr / SECTOR_SIZE;
        int rest = 0;
        if (bis < bufsec) {
            rest = bufsec*SECTOR_SIZE - addr;
            if (rest > len) {
                rest = len;
            }
            mp_qspiflash_read_raw(self, addr, rest, dest);
            len -= rest;
            if (len <= 0) {
                mp_qspiflash_release_bus(self);
                return;
            } else {
                // something from buffer...
                addr = bufsec*SECTOR_SIZE;
                dest += rest;
                if (len > SECTOR_SIZE) {
                    rest = SECTOR_SIZE;
                } else {
                    rest = len;
                }
                memcpy(dest, buf, rest);
                len -= rest;
                if (len <= 0) {
                    mp_qspiflash_release_bus(self);
                    return;
                }
                dest += rest;
                addr += rest;
            }
        } else if (bis == bufsec) {
            uint32_t offset = addr & (SECTOR_SIZE-1);
            rest = SECTOR_SIZE - offset;
            if (rest > len) {
                rest = len;
            }
            memcpy(dest, &buf[offset], rest);
            len -= rest;
            if (len <= 0) {
                mp_qspiflash_release_bus(self);
                return;
            }
        }
        dest += rest;
        addr += rest;
    }
    // read rest direct from flash
    mp_qspiflash_read_raw(self, addr, len, dest);
    mp_qspiflash_release_bus(self);
}

// flush buffer
void mp_qspiflash_flush(mp_qspiflash_t *self) {
#ifdef USE_WR_DELAY
    if (self->flags & 1) {
        self->flags &= ~1;
        // erase sector
        int ret = mp_qspiflash_erase_sector(self, bufsec*SECTOR_SIZE);
        if (ret != 0) {
            mp_qspiflash_release_bus(self);
            printf("<FERR>\n");
            return;
        }
        // write
        for (int i = 0; i < 16; i += 1) {
            int ret = mp_qspiflash_write_page(self, bufsec*SECTOR_SIZE + i*256, buf + i*256);
            if (ret != 0) {
                mp_qspiflash_release_bus(self);
                printf("<PERR>\n");
                return;
            }
        }
        // indicate a clean cache with LED off
        led_state(PYB_LED_RED, 0);
    }
#endif
}

int mp_qspiflash_write_part(mp_qspiflash_t *self, uint32_t addr, size_t len, const uint8_t *src) {
    // TODO optimise so we don't need to erase multiple times for successive writes to a sector
    //
    // if either target area is empty (all 0xff) or identical don't erase...

    // align to 4096 sector
    #define oaddr (addr+offset)
    uint32_t offset = addr & 0xfff;
    uint32_t sec = addr >> 12;
    addr = sec << 12;

    // restriction for now, so we don't need to erase multiple pages
    if (offset + len > sizeof(buf)) {
        printf("%s: len is too large\n", __func__);
        return -MP_EIO;
    }

    //mp_qspiflash_acquire_bus(self);

    // XXX
    // check valid buffer
    //   if buffer target matches --> skip buffer re-read

    if (bufsec == sec) {
    } else {
        // read sector
#ifdef USE_WR_DELAY
        if (bufsec != 0xffffffff) {
            mp_qspiflash_flush(self);
        }
#endif
        mp_qspiflash_read_raw(self, addr, SECTOR_SIZE, buf);
    }

#ifdef USE_WR_DELAY
    bufsec = sec;
    // just copy to buffer
    memcpy(buf + offset, src, len);
    // and mark dirty
    self->flags |= 1;
    // indicate a dirty cache with LED on
    led_state(PYB_LED_RED, 1);
#else
    ...
    // XXX
    // check identity
    //  --> nothing to do

    // XXX
    // check blank
    //  --> skip erase
    uint32_t dirty = 0;
    for (int i = 0; i < len; i++) {
        if (buf[offset+i] != src[i]) {
            if (buf[offset+i] != 0xff) {
                // erase sector
                int ret = mp_qspiflash_erase_sector(self, addr);
                if (ret != 0) {
                    mp_qspiflash_release_bus(self);
                    printf("<EERR>\n");
                    return ret;
                }
                dirty = 0xffff;
                break;
            } else {
                dirty |= (1 << ((offset+i) >> 8));
            }
        }
    }

    bufsec = sec;
    // copy new block into buffer
    memcpy(buf + offset, src, len);

    // write sector in pages of 256 bytes
    for (int i = 0; i < 16; i += 1) {
        if (dirty & (1 << i)) {
            int ret = mp_qspiflash_write_page(self, addr + i*256, buf + i*256);
            if (ret != 0) {
                mp_qspiflash_release_bus(self);
                return ret;
            }
        }
    }

    #if DO_VERIFY
    ...
    if (len <= sizeof(rbuf)) {
        // readback & compare
        //mp_qspiflash_read(self, addr, len, rbuf);
        uint8_t cmd[9] = {CMD_C4READ, addr >> 16, addr >> 8, addr, 0, 0, 0};
        CS_LOW;
        mp_qspiflash_transfer(self, 1, cmd, NULL);
        mp_qspiflash_qwrite(self, dummies, &cmd[1]);
        mp_qspiflash_qread(self, len, rbuf);
        CS_HIGH;
        for (int i = 0; i < len; i++) {
            if (rbuf[i] != src[i]) {
                printf("<MERR %lx:%lx>", oaddr, (uint32_t)i);
                break;
            }
        }
    }
    #endif
#endif

    mp_qspiflash_release_bus(self);
    return 0; // success
}

int mp_qspiflash_write(mp_qspiflash_t *self, uint32_t addr, size_t len, const uint8_t *src) {
    // TODO optimise so we don't need to erase multiple times for successive writes to a sector
    //
    // if either target area is empty (all 0xff) or identical don't erase...
    uint32_t bis = addr / SECTOR_SIZE;
    uint32_t bie = (addr+len-1) / SECTOR_SIZE;

    mp_qspiflash_acquire_bus(self);
    if ((bis <= bufsec) && (bie >= bufsec)) {
        // current buffer affected, handle this part first
        uint32_t taddr = (bufsec+1)*SECTOR_SIZE;
        int32_t offset = addr - bufsec*SECTOR_SIZE;
        int32_t pre = bufsec*SECTOR_SIZE - addr;
        if (offset < 0) {
            offset = 0;
        } else {
            pre = 0;
        }
        int32_t rest = len-pre;
        int32_t trail = 0;
        if (rest > (SECTOR_SIZE-offset)) {
            trail = rest - (SECTOR_SIZE-offset);
            rest = SECTOR_SIZE-offset;
        }
        memcpy(&buf[offset], &src[pre], rest);
        self->flags |= 1;                       // mark dirty
        if ((pre | trail) == 0) {
            mp_qspiflash_release_bus(self);
            return 0;                             // done!
        }
        const uint8_t *p = src;
        while (pre) {
            int rest = pre & (SECTOR_SIZE-1);
            if (rest == 0) {
                rest = SECTOR_SIZE;
            }
            mp_qspiflash_write_part(self, addr, rest, p);
            p += rest;
            addr += rest;
            pre -= rest;
        }
        while (trail) {
            int rest = trail;
            if (rest > SECTOR_SIZE) {
                rest = SECTOR_SIZE;
            }
            mp_qspiflash_write_part(self, taddr, rest, src);
            src += rest;
            taddr += rest;
            trail -= rest;
        }
    } else {
        // current buffer not affected, business as usual
        uint32_t offset = addr & (SECTOR_SIZE-1);
        while (len) {
            int rest = SECTOR_SIZE-offset;
            if (rest > len) {
                rest = len;
            }
            mp_qspiflash_write_part(self, addr, rest, src);
            len -= rest;
            addr += rest;
            src += rest;
            offset = 0;
        }
    }
    mp_qspiflash_release_bus(self);
    return 0;
}

/******************************************************************************/
// Micro Python bindings
//

const mp_obj_base_t machine_qspiflash_obj = {&machine_qspiflash_type};

STATIC mp_obj_t machine_qspi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    mp_qspiflash_init((mp_qspiflash_t *)&qspiflash);

    // return singleton object
    return (mp_obj_t)&machine_qspiflash_obj;
}

STATIC mp_obj_t qspi_info(mp_obj_t self) {
    mp_obj_t tuple[3] = {
        mp_obj_new_int_from_ull(0x200000),
        mp_obj_new_int_from_uint(512),
        mp_obj_new_int(0),
    };
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(qspi_info_obj, qspi_info);

STATIC mp_obj_t machine_qspi_readblocks(mp_obj_t self_in, mp_obj_t block_num, mp_obj_t buf) {
    //printf("%s %p\n", __func__, buf);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);
    //printf("%s %p %d\n", __func__, bufinfo.buf, bufinfo.len);
    //mp_uint_t ret = sdcard_read_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
    mp_uint_t ret = 0;
    mp_qspiflash_read((mp_qspiflash_t*)&qspiflash, mp_obj_get_int(block_num)*512, bufinfo.len, bufinfo.buf);
    return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(machine_qspi_readblocks_obj, machine_qspi_readblocks);

STATIC mp_obj_t machine_qspi_writeblocks(mp_obj_t self, mp_obj_t block_num, mp_obj_t buf) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_READ);
    mp_uint_t ret = 0;  // sdcard_write_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
    mp_qspiflash_write((mp_qspiflash_t*)&qspiflash, mp_obj_get_int(block_num)*512, bufinfo.len, bufinfo.buf);
    return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(machine_qspi_writeblocks_obj, machine_qspi_writeblocks);

STATIC mp_obj_t mp_qspiflash_get_histo(mp_obj_t self_in, mp_obj_t buf) {
    //printf("%s %p\n", __func__, buf);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);
    //printf("%s %p %d\n", __func__, bufinfo.buf, bufinfo.len);
    mp_uint_t ret = 0;
    if (bufinfo.len > 0) {
        if (bufinfo.len > sizeof(erase_histo)) {
            memcpy(bufinfo.buf, (void *)erase_histo, sizeof(erase_histo));
        } else {
            memcpy(bufinfo.buf, (void *)erase_histo, bufinfo.len);
        }
    }
    return mp_obj_new_bool(ret == 0);
}

STATIC mp_obj_t machine_qspi_ioctl(mp_obj_t self, mp_obj_t cmd_in, mp_obj_t arg_in) {
    mp_int_t cmd = mp_obj_get_int(cmd_in);
    switch (cmd) {
        case BP_IOCTL_INIT:
            // evaluate QSPI status TODO
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_DEINIT:
            // nothing to do
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_SYNC:
            mp_qspiflash_flush((mp_qspiflash_t*)&qspiflash);
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_SEC_COUNT:
            return MP_OBJ_NEW_SMALL_INT(2048*2); // TODO

        case BP_IOCTL_SEC_SIZE:
            return MP_OBJ_NEW_SMALL_INT(512);

        case 0x4343:                // 
            return MP_OBJ_NEW_SMALL_INT(mp_qspiflash_set_CR((mp_qspiflash_t*)&qspiflash, mp_obj_get_int(arg_in)));
        case 0x4363:                // 
            return MP_OBJ_NEW_SMALL_INT(mp_qspiflash_get_CR((mp_qspiflash_t*)&qspiflash));
        case 0x4345:                // BP_IOCTL_SEC_ERASE:
            return MP_OBJ_NEW_SMALL_INT(mp_qspiflash_erase_sector((mp_qspiflash_t*)&qspiflash, mp_obj_get_int(arg_in)));
        case 0x4346:                // BP_IOCTL_FULL_ERASE:
            return MP_OBJ_NEW_SMALL_INT(mp_qspiflash_erase_all((mp_qspiflash_t*)&qspiflash));
        case 0x4342:                // semaphore blocks
            return MP_OBJ_NEW_SMALL_INT(qspi_sema_block_cnt);
        case 0x4348:                // erase histogram
            return MP_OBJ_NEW_SMALL_INT(mp_qspiflash_get_histo((mp_qspiflash_t*)&qspiflash, arg_in));
        case 0x4352:                // rd security
            return MP_OBJ_NEW_SMALL_INT(mp_qspiflash_sec_id((mp_qspiflash_t*)&qspiflash));
        case 0x4355:                // rd security
            return MP_OBJ_NEW_SMALL_INT(mp_qspiflash_ulbpr((mp_qspiflash_t*)&qspiflash));
        case 0x4366:                // rd security
            return MP_OBJ_NEW_SMALL_INT(qspiflash.flags);

        default: // unknown command
            return MP_OBJ_NEW_SMALL_INT(-1); // error
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(machine_qspi_ioctl_obj, machine_qspi_ioctl);

STATIC const mp_map_elem_t machine_qspi_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_info), (mp_obj_t)&qspi_info_obj },
    //{ MP_OBJ_NEW_QSTR(MP_QSTR_read), (mp_obj_t)&qspi_read_obj },
    //{ MP_OBJ_NEW_QSTR(MP_QSTR_write), (mp_obj_t)&qspi_write_obj },
    // block device protocol
    { MP_OBJ_NEW_QSTR(MP_QSTR_readblocks), (mp_obj_t)&machine_qspi_readblocks_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_writeblocks), (mp_obj_t)&machine_qspi_writeblocks_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ioctl), (mp_obj_t)&machine_qspi_ioctl_obj },
};

STATIC MP_DEFINE_CONST_DICT(machine_qspi_locals_dict, machine_qspi_locals_dict_table);

const mp_obj_type_t machine_qspiflash_type = {
    { &mp_type_type },
    .name = MP_QSTR_QSPI_Flash,
    .make_new = machine_qspi_make_new,
    .locals_dict = (mp_obj_t)&machine_qspi_locals_dict,
};

void machine_qspi_init_vfs(fs_user_mount_t *vfs, int part) {
    vfs->base.type = &mp_fat_vfs_type;
    vfs->flags |= FSUSER_NATIVE | FSUSER_HAVE_IOCTL;
    vfs->fatfs.drv = vfs;
    vfs->fatfs.part = part;
    vfs->readblocks[0] = (mp_obj_t)&machine_qspi_readblocks_obj;
    vfs->readblocks[1] = (mp_obj_t)&machine_qspiflash_obj;
    vfs->readblocks[2] = (mp_obj_t)mp_qspiflash_read; // native version
    vfs->writeblocks[0] = (mp_obj_t)&machine_qspi_writeblocks_obj;
    vfs->writeblocks[1] = (mp_obj_t)&machine_qspiflash_obj;
    vfs->writeblocks[2] = (mp_obj_t)mp_qspiflash_write; // native version
    vfs->u.ioctl[0] = (mp_obj_t)&machine_qspi_ioctl_obj;
    vfs->u.ioctl[1] = (mp_obj_t)&machine_qspiflash_obj;
}
#endif // MICROPY_HW_QSPIFLASH_SIZE_BITS
