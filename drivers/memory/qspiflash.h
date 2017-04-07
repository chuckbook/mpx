/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Damien P. George
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

#ifdef MICROPY_HW_QSPIFLASH_SIZE_BITS
#ifndef MICROPY_INCLUDED_DRIVERS_MEMORY_QSPIFLASH_H
#define MICROPY_INCLUDED_DRIVERS_MEMORY_QSPIFLASH_H

#include "extmod/machine_qspi.h"

typedef struct _mp_qspiflash_t {
    // TODO replace with generic QSPI object
    volatile uint32_t flags;
    mp_machine_soft_qspi_obj_t qspi;
} mp_qspiflash_t;

extern const struct _mp_obj_type_t machine_qspiflash_type;
extern const struct _mp_obj_base_t machine_qspiflash_obj;

void mp_qspiflash_init(mp_qspiflash_t *self);
void mp_qspiflash_read(mp_qspiflash_t *self, uint32_t addr, size_t len, uint8_t *dest);
int mp_qspiflash_write(mp_qspiflash_t *self, uint32_t addr, size_t len, const uint8_t *src);
void mp_qspiflash_flush(mp_qspiflash_t *self);

struct _fs_user_mount_t;
void machine_qspi_init_vfs(struct _fs_user_mount_t *vfs, int part);

#endif // MICROPY_INCLUDED_DRIVERS_MEMORY_QSPIFLASH_H
#endif // MICROPY_HW_QSPIFLASH_SIZE_BITS
