/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
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

#include "py/runtime.h"
#include "extmod/machine_spi.h"

#if MICROPY_PY_MACHINE_SPI

// if a port didn't define MSB/LSB constants then provide them
#ifndef MICROPY_PY_MACHINE_SPI_MSB
#define MICROPY_PY_MACHINE_SPI_MSB (0)
#define MICROPY_PY_MACHINE_SPI_LSB (1)
#endif

#if USE_DUMMY_PAT
const static uint32_t dummy_write[512] = {
0xefbeadde, 0xf0beadde, 0xf1beadde, 0xf2beadde, 0xf3beadde, 0xf4beadde, 0xf5beadde, 0xf6beadde, 
0xf7beadde, 0xf8beadde, 0xf9beadde, 0xfabeadde, 0xfbbeadde, 0xfcbeadde, 0xfdbeadde, 0xfebeadde, 
0xffbeadde, 0x00bfadde, 0x01bfadde, 0x02bfadde, 0x03bfadde, 0x04bfadde, 0x05bfadde, 0x06bfadde, 
0x07bfadde, 0x08bfadde, 0x09bfadde, 0x0abfadde, 0x0bbfadde, 0x0cbfadde, 0x0dbfadde, 0x0ebfadde, 
0x0fbfadde, 0x10bfadde, 0x11bfadde, 0x12bfadde, 0x13bfadde, 0x14bfadde, 0x15bfadde, 0x16bfadde, 
0x17bfadde, 0x18bfadde, 0x19bfadde, 0x1abfadde, 0x1bbfadde, 0x1cbfadde, 0x1dbfadde, 0x1ebfadde, 
0x1fbfadde, 0x20bfadde, 0x21bfadde, 0x22bfadde, 0x23bfadde, 0x24bfadde, 0x25bfadde, 0x26bfadde, 
0x27bfadde, 0x28bfadde, 0x29bfadde, 0x2abfadde, 0x2bbfadde, 0x2cbfadde, 0x2dbfadde, 0x2ebfadde, 
0x2fbfadde, 0x30bfadde, 0x31bfadde, 0x32bfadde, 0x33bfadde, 0x34bfadde, 0x35bfadde, 0x36bfadde, 
0x37bfadde, 0x38bfadde, 0x39bfadde, 0x3abfadde, 0x3bbfadde, 0x3cbfadde, 0x3dbfadde, 0x3ebfadde, 
0x3fbfadde, 0x40bfadde, 0x41bfadde, 0x42bfadde, 0x43bfadde, 0x44bfadde, 0x45bfadde, 0x46bfadde, 
0x47bfadde, 0x48bfadde, 0x49bfadde, 0x4abfadde, 0x4bbfadde, 0x4cbfadde, 0x4dbfadde, 0x4ebfadde, 
0x4fbfadde, 0x50bfadde, 0x51bfadde, 0x52bfadde, 0x53bfadde, 0x54bfadde, 0x55bfadde, 0x56bfadde, 
0x57bfadde, 0x58bfadde, 0x59bfadde, 0x5abfadde, 0x5bbfadde, 0x5cbfadde, 0x5dbfadde, 0x5ebfadde, 
0x5fbfadde, 0x60bfadde, 0x61bfadde, 0x62bfadde, 0x63bfadde, 0x64bfadde, 0x65bfadde, 0x66bfadde, 
0x67bfadde, 0x68bfadde, 0x69bfadde, 0x6abfadde, 0x6bbfadde, 0x6cbfadde, 0x6dbfadde, 0x6ebfadde, 
0x6fbfadde, 0x70bfadde, 0x71bfadde, 0x72bfadde, 0x73bfadde, 0x74bfadde, 0x75bfadde, 0x76bfadde, 
0x77bfadde, 0x78bfadde, 0x79bfadde, 0x7abfadde, 0x7bbfadde, 0x7cbfadde, 0x7dbfadde, 0x7ebfadde, 
0x7fbfadde, 0x80bfadde, 0x81bfadde, 0x82bfadde, 0x83bfadde, 0x84bfadde, 0x85bfadde, 0x86bfadde, 
0x87bfadde, 0x88bfadde, 0x89bfadde, 0x8abfadde, 0x8bbfadde, 0x8cbfadde, 0x8dbfadde, 0x8ebfadde, 
0x8fbfadde, 0x90bfadde, 0x91bfadde, 0x92bfadde, 0x93bfadde, 0x94bfadde, 0x95bfadde, 0x96bfadde, 
0x97bfadde, 0x98bfadde, 0x99bfadde, 0x9abfadde, 0x9bbfadde, 0x9cbfadde, 0x9dbfadde, 0x9ebfadde, 
0x9fbfadde, 0xa0bfadde, 0xa1bfadde, 0xa2bfadde, 0xa3bfadde, 0xa4bfadde, 0xa5bfadde, 0xa6bfadde, 
0xa7bfadde, 0xa8bfadde, 0xa9bfadde, 0xaabfadde, 0xabbfadde, 0xacbfadde, 0xadbfadde, 0xaebfadde, 
0xafbfadde, 0xb0bfadde, 0xb1bfadde, 0xb2bfadde, 0xb3bfadde, 0xb4bfadde, 0xb5bfadde, 0xb6bfadde, 
0xb7bfadde, 0xb8bfadde, 0xb9bfadde, 0xbabfadde, 0xbbbfadde, 0xbcbfadde, 0xbdbfadde, 0xbebfadde, 
0xbfbfadde, 0xc0bfadde, 0xc1bfadde, 0xc2bfadde, 0xc3bfadde, 0xc4bfadde, 0xc5bfadde, 0xc6bfadde, 
0xc7bfadde, 0xc8bfadde, 0xc9bfadde, 0xcabfadde, 0xcbbfadde, 0xccbfadde, 0xcdbfadde, 0xcebfadde, 
0xcfbfadde, 0xd0bfadde, 0xd1bfadde, 0xd2bfadde, 0xd3bfadde, 0xd4bfadde, 0xd5bfadde, 0xd6bfadde, 
0xd7bfadde, 0xd8bfadde, 0xd9bfadde, 0xdabfadde, 0xdbbfadde, 0xdcbfadde, 0xddbfadde, 0xdebfadde, 
0xdfbfadde, 0xe0bfadde, 0xe1bfadde, 0xe2bfadde, 0xe3bfadde, 0xe4bfadde, 0xe5bfadde, 0xe6bfadde, 
0xe7bfadde, 0xe8bfadde, 0xe9bfadde, 0xeabfadde, 0xebbfadde, 0xecbfadde, 0xedbfadde, 0xeebfadde, 
0xefbfadde, 0xf0bfadde, 0xf1bfadde, 0xf2bfadde, 0xf3bfadde, 0xf4bfadde, 0xf5bfadde, 0xf6bfadde, 
0xf7bfadde, 0xf8bfadde, 0xf9bfadde, 0xfabfadde, 0xfbbfadde, 0xfcbfadde, 0xfdbfadde, 0xfebfadde, 
0xffbfadde, 0x00c0adde, 0x01c0adde, 0x02c0adde, 0x03c0adde, 0x04c0adde, 0x05c0adde, 0x06c0adde, 
0x07c0adde, 0x08c0adde, 0x09c0adde, 0x0ac0adde, 0x0bc0adde, 0x0cc0adde, 0x0dc0adde, 0x0ec0adde, 
0x0fc0adde, 0x10c0adde, 0x11c0adde, 0x12c0adde, 0x13c0adde, 0x14c0adde, 0x15c0adde, 0x16c0adde, 
0x17c0adde, 0x18c0adde, 0x19c0adde, 0x1ac0adde, 0x1bc0adde, 0x1cc0adde, 0x1dc0adde, 0x1ec0adde, 
0x1fc0adde, 0x20c0adde, 0x21c0adde, 0x22c0adde, 0x23c0adde, 0x24c0adde, 0x25c0adde, 0x26c0adde, 
0x27c0adde, 0x28c0adde, 0x29c0adde, 0x2ac0adde, 0x2bc0adde, 0x2cc0adde, 0x2dc0adde, 0x2ec0adde, 
0x2fc0adde, 0x30c0adde, 0x31c0adde, 0x32c0adde, 0x33c0adde, 0x34c0adde, 0x35c0adde, 0x36c0adde, 
0x37c0adde, 0x38c0adde, 0x39c0adde, 0x3ac0adde, 0x3bc0adde, 0x3cc0adde, 0x3dc0adde, 0x3ec0adde, 
0x3fc0adde, 0x40c0adde, 0x41c0adde, 0x42c0adde, 0x43c0adde, 0x44c0adde, 0x45c0adde, 0x46c0adde, 
0x47c0adde, 0x48c0adde, 0x49c0adde, 0x4ac0adde, 0x4bc0adde, 0x4cc0adde, 0x4dc0adde, 0x4ec0adde, 
0x4fc0adde, 0x50c0adde, 0x51c0adde, 0x52c0adde, 0x53c0adde, 0x54c0adde, 0x55c0adde, 0x56c0adde, 
0x57c0adde, 0x58c0adde, 0x59c0adde, 0x5ac0adde, 0x5bc0adde, 0x5cc0adde, 0x5dc0adde, 0x5ec0adde, 
0x5fc0adde, 0x60c0adde, 0x61c0adde, 0x62c0adde, 0x63c0adde, 0x64c0adde, 0x65c0adde, 0x66c0adde, 
0x67c0adde, 0x68c0adde, 0x69c0adde, 0x6ac0adde, 0x6bc0adde, 0x6cc0adde, 0x6dc0adde, 0x6ec0adde, 
0x6fc0adde, 0x70c0adde, 0x71c0adde, 0x72c0adde, 0x73c0adde, 0x74c0adde, 0x75c0adde, 0x76c0adde, 
0x77c0adde, 0x78c0adde, 0x79c0adde, 0x7ac0adde, 0x7bc0adde, 0x7cc0adde, 0x7dc0adde, 0x7ec0adde, 
0x7fc0adde, 0x80c0adde, 0x81c0adde, 0x82c0adde, 0x83c0adde, 0x84c0adde, 0x85c0adde, 0x86c0adde, 
0x87c0adde, 0x88c0adde, 0x89c0adde, 0x8ac0adde, 0x8bc0adde, 0x8cc0adde, 0x8dc0adde, 0x8ec0adde, 
0x8fc0adde, 0x90c0adde, 0x91c0adde, 0x92c0adde, 0x93c0adde, 0x94c0adde, 0x95c0adde, 0x96c0adde, 
0x97c0adde, 0x98c0adde, 0x99c0adde, 0x9ac0adde, 0x9bc0adde, 0x9cc0adde, 0x9dc0adde, 0x9ec0adde, 
0x9fc0adde, 0xa0c0adde, 0xa1c0adde, 0xa2c0adde, 0xa3c0adde, 0xa4c0adde, 0xa5c0adde, 0xa6c0adde, 
0xa7c0adde, 0xa8c0adde, 0xa9c0adde, 0xaac0adde, 0xabc0adde, 0xacc0adde, 0xadc0adde, 0xaec0adde, 
0xafc0adde, 0xb0c0adde, 0xb1c0adde, 0xb2c0adde, 0xb3c0adde, 0xb4c0adde, 0xb5c0adde, 0xb6c0adde, 
0xb7c0adde, 0xb8c0adde, 0xb9c0adde, 0xbac0adde, 0xbbc0adde, 0xbcc0adde, 0xbdc0adde, 0xbec0adde, 
0xbfc0adde, 0xc0c0adde, 0xc1c0adde, 0xc2c0adde, 0xc3c0adde, 0xc4c0adde, 0xc5c0adde, 0xc6c0adde, 
0xc7c0adde, 0xc8c0adde, 0xc9c0adde, 0xcac0adde, 0xcbc0adde, 0xccc0adde, 0xcdc0adde, 0xcec0adde, 
0xcfc0adde, 0xd0c0adde, 0xd1c0adde, 0xd2c0adde, 0xd3c0adde, 0xd4c0adde, 0xd5c0adde, 0xd6c0adde, 
0xd7c0adde, 0xd8c0adde, 0xd9c0adde, 0xdac0adde, 0xdbc0adde, 0xdcc0adde, 0xddc0adde, 0xdec0adde, 
0xdfc0adde, 0xe0c0adde, 0xe1c0adde, 0xe2c0adde, 0xe3c0adde, 0xe4c0adde, 0xe5c0adde, 0xe6c0adde, 
0xe7c0adde, 0xe8c0adde, 0xe9c0adde, 0xeac0adde, 0xebc0adde, 0xecc0adde, 0xedc0adde, 0xeec0adde, 
                           };
#endif

void mp_machine_soft_spi_transfer(mp_obj_base_t *self_in, size_t len, const uint8_t *src, uint8_t *dest) {
    mp_machine_soft_spi_obj_t *self = (mp_machine_soft_spi_obj_t*)self_in;
    uint32_t delay_half = self->delay_half;
#if USE_DUMMY_PAT
    if (src == NULL) {
//printf("%s %d\n", __func__, len);
        src = (uint8_t *)dummy_write;
    }
#endif

    // only MSB transfer is implemented

    // If a port defines MICROPY_PY_MACHINE_SPI_MIN_DELAY, and the configured
    // delay_half is equal to this value, then the software SPI implementation
    // will run as fast as possible, limited only by CPU speed and GPIO time.
    #ifdef MICROPY_PY_MACHINE_SPI_MIN_DELAY
    if (delay_half == MICROPY_PY_MACHINE_SPI_MIN_DELAY) {
        if (0) {
        #if 0
        } else if (self->polarity == 0) {
            if (dest == NULL) {
                for (size_t i = 0; i < len; ++i) {
                    uint8_t data_out = src[i];
                    for (int j = 0; j < 8; ++j, data_out <<= 1) {
                        mp_hal_pin_write(self->mosi, (data_out >> 7) & 1);
                        mp_hal_pin_write(self->sck, 1); //  - self->polarity);
                        mp_hal_pin_write(self->sck, 0); //self->polarity);
                    }
                }
            } else {
                for (size_t i = 0; i < len; ++i) {
                    uint8_t data_out = src[i];
                    uint8_t data_in = 0;
                    for (int j = 0; j < 8; ++j, data_out <<= 1) {
                        mp_hal_pin_write(self->mosi, (data_out >> 7) & 1);
                        mp_hal_pin_write(self->sck, 1); //  - self->polarity);
                        data_in = (data_in << 1) | mp_hal_pin_read(self->miso);
                        mp_hal_pin_write(self->sck, 0); //self->polarity);
                    }
                    if (dest != NULL) {
                        dest[i] = data_in;
                    }
                }
            }
        #endif
        } else {
            for (size_t i = 0; i < len; ++i) {
                uint8_t data_out = src[i];
                uint8_t data_in = 0;
                for (int j = 0; j < 8; ++j, data_out <<= 1) {
                    mp_hal_pin_write(self->mosi, (data_out >> 7) & 1);
                    mp_hal_pin_write(self->sck, 1 - self->polarity);
                    data_in = (data_in << 1) | mp_hal_pin_read(self->miso);
                    mp_hal_pin_write(self->sck, self->polarity);
                }
                if (dest != NULL) {
                    dest[i] = data_in;
                }
            }
        }
        return;
    }
    #endif

    for (size_t i = 0; i < len; ++i) {
        uint8_t data_out = src[i];
        uint8_t data_in = 0;
        for (int j = 0; j < 8; ++j, data_out <<= 1) {
            mp_hal_pin_write(self->mosi, (data_out >> 7) & 1);
            if (self->phase == 0) {
                mp_hal_delay_us_fast(delay_half);
                mp_hal_pin_write(self->sck, 1 - self->polarity);
            } else {
                mp_hal_pin_write(self->sck, 1 - self->polarity);
                mp_hal_delay_us_fast(delay_half);
            }
            data_in = (data_in << 1) | mp_hal_pin_read(self->miso);
            if (self->phase == 0) {
                mp_hal_delay_us_fast(delay_half);
                mp_hal_pin_write(self->sck, self->polarity);
            } else {
                mp_hal_pin_write(self->sck, self->polarity);
                mp_hal_delay_us_fast(delay_half);
            }
        }
        if (dest != NULL) {
            dest[i] = data_in;
        }
    }
}

/******************************************************************************/
// MicroPython bindings for generic machine.SPI

STATIC mp_obj_t mp_machine_soft_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args);

mp_obj_t mp_machine_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check the id argument, if given
    if (n_args > 0) {
        if (args[0] != MP_OBJ_NEW_SMALL_INT(-1)) {
            #if defined(MICROPY_PY_MACHINE_SPI_MAKE_NEW)
            // dispatch to port-specific constructor
            extern mp_obj_t MICROPY_PY_MACHINE_SPI_MAKE_NEW(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args);
            return MICROPY_PY_MACHINE_SPI_MAKE_NEW(type, n_args, n_kw, args);
            #else
            mp_raise_ValueError("invalid SPI peripheral");
            #endif
        }
        --n_args;
        ++args;
    }

    // software SPI
    return mp_machine_soft_spi_make_new(type, n_args, n_kw, args);
}

STATIC mp_obj_t machine_spi_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[0]);
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)s->type->protocol;
    spi_p->init(s, n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_spi_init_obj, 1, machine_spi_init);

STATIC mp_obj_t machine_spi_deinit(mp_obj_t self) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(self);
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)s->type->protocol;
    if (spi_p->deinit != NULL) {
        spi_p->deinit(s);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_spi_deinit_obj, machine_spi_deinit);

STATIC void mp_machine_spi_transfer(mp_obj_t self, size_t len, const void *src, void *dest) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(self);
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)s->type->protocol;
    spi_p->transfer(s, len, src, dest);
}

STATIC mp_obj_t mp_machine_spi_read(size_t n_args, const mp_obj_t *args) {
    vstr_t vstr;
    vstr_init_len(&vstr, mp_obj_get_int(args[1]));
    memset(vstr.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, vstr.len);
    mp_machine_spi_transfer(args[0], vstr.len, vstr.buf, vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_spi_read_obj, 2, 3, mp_machine_spi_read);

STATIC mp_obj_t mp_machine_spi_readinto(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_WRITE);
    memset(bufinfo.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, bufinfo.len);
    mp_machine_spi_transfer(args[0], bufinfo.len, bufinfo.buf, bufinfo.buf);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_spi_readinto_obj, 2, 3, mp_machine_spi_readinto);

STATIC mp_obj_t mp_machine_spi_write(mp_obj_t self, mp_obj_t wr_buf) {
    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);
    mp_machine_spi_transfer(self, src.len, (const uint8_t*)src.buf, NULL);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_spi_write_obj, mp_machine_spi_write);

STATIC mp_obj_t mp_machine_spi_write_readinto(mp_obj_t self, mp_obj_t wr_buf, mp_obj_t rd_buf) {
    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);
    mp_buffer_info_t dest;
    mp_get_buffer_raise(rd_buf, &dest, MP_BUFFER_WRITE);
    if (src.len != dest.len) {
        mp_raise_ValueError("buffers must be the same length");
    }
    mp_machine_spi_transfer(self, src.len, src.buf, dest.buf);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_spi_write_readinto_obj, mp_machine_spi_write_readinto);

STATIC const mp_rom_map_elem_t machine_spi_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_spi_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&machine_spi_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_machine_spi_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_machine_spi_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_machine_spi_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_readinto), MP_ROM_PTR(&mp_machine_spi_write_readinto_obj) },

    { MP_ROM_QSTR(MP_QSTR_MSB), MP_ROM_INT(MICROPY_PY_MACHINE_SPI_MSB) },
    { MP_ROM_QSTR(MP_QSTR_LSB), MP_ROM_INT(MICROPY_PY_MACHINE_SPI_LSB) },
};

MP_DEFINE_CONST_DICT(mp_machine_spi_locals_dict, machine_spi_locals_dict_table);

/******************************************************************************/
// Implementation of soft SPI

STATIC uint32_t baudrate_from_delay_half(uint32_t delay_half) {
    #ifdef MICROPY_PY_MACHINE_SPI_MIN_DELAY
    if (delay_half == MICROPY_PY_MACHINE_SPI_MIN_DELAY) {
        return MICROPY_PY_MACHINE_SPI_MAX_BAUDRATE;
    } else
    #endif
    {
        return 500000 / delay_half;
    }
}

STATIC uint32_t baudrate_to_delay_half(uint32_t baudrate) {
    #ifdef MICROPY_PY_MACHINE_SPI_MIN_DELAY
    if (baudrate >= MICROPY_PY_MACHINE_SPI_MAX_BAUDRATE) {
        return MICROPY_PY_MACHINE_SPI_MIN_DELAY;
    } else
    #endif
    {
        uint32_t delay_half = 500000 / baudrate;
        // round delay_half up so that: actual_baudrate <= requested_baudrate
        if (500000 % baudrate != 0) {
            delay_half += 1;
        }
        return delay_half;
    }
}

STATIC void mp_machine_soft_spi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    mp_machine_soft_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "SoftSPI(baudrate=%u, polarity=%u, phase=%u,"
        " sck=" MP_HAL_PIN_FMT ", mosi=" MP_HAL_PIN_FMT ", miso=" MP_HAL_PIN_FMT ")",
        baudrate_from_delay_half(self->delay_half), self->polarity, self->phase,
        mp_hal_pin_name(self->sck), mp_hal_pin_name(self->mosi), mp_hal_pin_name(self->miso));
}

STATIC mp_obj_t mp_machine_soft_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit, ARG_sck, ARG_mosi, ARG_miso };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 500000} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = MICROPY_PY_MACHINE_SPI_MSB} },
        { MP_QSTR_sck,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mosi,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_miso,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // create new object
    mp_machine_soft_spi_obj_t *self = m_new_obj(mp_machine_soft_spi_obj_t);
    self->base.type = &mp_machine_soft_spi_type;

    // set parameters
    self->delay_half = baudrate_to_delay_half(args[ARG_baudrate].u_int);
    self->polarity = args[ARG_polarity].u_int;
    self->phase = args[ARG_phase].u_int;
    if (args[ARG_bits].u_int != 8) {
        mp_raise_ValueError("bits must be 8");
    }
    if (args[ARG_firstbit].u_int != MICROPY_PY_MACHINE_SPI_MSB) {
        mp_raise_ValueError("firstbit must be MSB");
    }
    if (args[ARG_sck].u_obj == MP_OBJ_NULL
        || args[ARG_mosi].u_obj == MP_OBJ_NULL
        || args[ARG_miso].u_obj == MP_OBJ_NULL) {
        mp_raise_ValueError("must specify all of sck/mosi/miso");
    }
    self->sck = mp_hal_get_pin_obj(args[ARG_sck].u_obj);
    self->mosi = mp_hal_get_pin_obj(args[ARG_mosi].u_obj);
    self->miso = mp_hal_get_pin_obj(args[ARG_miso].u_obj);

    // configure pins
    mp_hal_pin_write(self->sck, self->polarity);
    mp_hal_pin_output(self->sck);
    mp_hal_pin_output(self->mosi);
    mp_hal_pin_input(self->miso);

    return MP_OBJ_FROM_PTR(self);
}

STATIC void mp_machine_soft_spi_init(mp_obj_base_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    mp_machine_soft_spi_obj_t *self = (mp_machine_soft_spi_obj_t*)self_in;

    enum { ARG_baudrate, ARG_polarity, ARG_phase, ARG_sck, ARG_mosi, ARG_miso };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_polarity, MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_phase, MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_sck, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mosi, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_miso, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[ARG_baudrate].u_int != -1) {
        self->delay_half = baudrate_to_delay_half(args[ARG_baudrate].u_int);
    }
    if (args[ARG_polarity].u_int != -1) {
        self->polarity = args[ARG_polarity].u_int;
    }
    if (args[ARG_phase].u_int != -1) {
        self->phase = args[ARG_phase].u_int;
    }
    if (args[ARG_sck].u_obj != MP_OBJ_NULL) {
        self->sck = mp_hal_get_pin_obj(args[ARG_sck].u_obj);
    }
    if (args[ARG_mosi].u_obj != MP_OBJ_NULL) {
        self->mosi = mp_hal_get_pin_obj(args[ARG_mosi].u_obj);
    }
    if (args[ARG_miso].u_obj != MP_OBJ_NULL) {
        self->miso = mp_hal_get_pin_obj(args[ARG_miso].u_obj);
    }

    // configure pins
    mp_hal_pin_write(self->sck, self->polarity);
    mp_hal_pin_output(self->sck);
    mp_hal_pin_output(self->mosi);
    mp_hal_pin_input(self->miso);
}

STATIC const mp_machine_spi_p_t mp_machine_soft_spi_p = {
    .init = mp_machine_soft_spi_init,
    .deinit = NULL,
    .transfer = mp_machine_soft_spi_transfer,
};

const mp_obj_type_t mp_machine_soft_spi_type = {
    { &mp_type_type },
    .name = MP_QSTR_SoftSPI,
    .print = mp_machine_soft_spi_print,
    .make_new = mp_machine_spi_make_new, // delegate to master constructor
    .protocol = &mp_machine_soft_spi_p,
    .locals_dict = (mp_obj_dict_t*)&mp_machine_spi_locals_dict,
};

#endif // MICROPY_PY_MACHINE_SPI
