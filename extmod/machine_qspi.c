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
#include "extmod/machine_qspi.h"

#ifdef MICROPY_HW_QSPIFLASH_SIZE_BITS // MICROPY_PY_MACHINE_QSPI

void mp_machine_soft_qspi_transfer(mp_obj_base_t *self_in, size_t len, const uint8_t *src, uint8_t *dest) {
    mp_machine_soft_qspi_obj_t *self = (mp_machine_soft_qspi_obj_t*)self_in;
#if 0
printf("%s", __func__);
for (int i = 0; i < len; i++) {
    if (src) {
        printf(" %02x", src[i]);
    } else {
        printf(" XX");
    }
}
printf("\n");
#endif

    // only MSB transfer is implemented

    // will run as fast as possible, limited only by CPU speed and GPIO time.
    mp_hal_pin_input(self->sio1);
    mp_hal_pin_output(self->sio0);
    if (self->sio3) {
        mp_hal_pin_write(self->sio2, 1);            // disable HWP
        mp_hal_pin_output(self->sio2);
        mp_hal_pin_write(self->sio3, 1);            // disable HWP
        mp_hal_pin_output(self->sio3);
    }
    if (src) {
        for (size_t i = 0; i < len; ++i) {
            uint8_t data_out = src[i];
            uint8_t data_in = 0;
            for (int j = 0; j < 8; ++j, data_out <<= 1) {
                mp_hal_pin_write(self->sio0, (data_out >> 7) & 1);
                mp_hal_pin_write(self->sck, 1);
#if defined(MICROPY_HW_QSPI_SLOW) && (MICROPY_HW_QSPI_SLOW == 1)
mp_hal_pin_write(self->sck, 1);
#endif
                data_in = (data_in << 1) | mp_hal_pin_read(self->sio1);
                mp_hal_pin_write(self->sck, 0);
            }
            if (dest != NULL) {
                dest[i] = data_in;
            }
        }
    } else {
        for (size_t i = 0; i < len; ++i) {
            uint8_t data_in = 0;
            for (int j = 0; j < 8; ++j) {
                mp_hal_pin_write(self->sck, 1);
#if defined(MICROPY_HW_QSPI_SLOW) && (MICROPY_HW_QSPI_SLOW == 1)
mp_hal_pin_write(self->sck, 1);
#endif
                data_in = (data_in << 1) | mp_hal_pin_read(self->sio1);
                mp_hal_pin_write(self->sck, 0);
            }
            if (dest != NULL) {
                dest[i] = data_in;
            }
        }
    }
}

#if defined(MICROPY_HW_QSPIFLASH_TURBO) && (MICROPY_HW_QSPIFLASH_TURBO == 1)
#if 0   // NITRO
const uint32_t rqqmapd[] = {
                            ((7 << 11) << 16),                  // 0 --> 0
                            ((6 << 11) << 16) | (1 << 11),      // 1 --> 1
                            ((5 << 11) << 16) | (2 << 11),      // 2 --> 2
                            ((4 << 11) << 16) | (3 << 11),      // 3 --> 3

                            ((7 << 11) << 16),                  // 4 --> 0
                            ((6 << 11) << 16) | (1 << 11),      // 5 --> 1
                            ((5 << 11) << 16) | (2 << 11),      // 6 --> 2
                            ((4 << 11) << 16) | (3 << 11),      // 7 --> 3

                            ((3 << 11) << 16) | (4 << 11),      // 8 --> 4
                            ((2 << 11) << 16) | (5 << 11),      // 9 --> 5
                            ((1 << 11) << 16) | (6 << 11),      //10 --> 6
                            ((0 << 11) << 16) | (7 << 11),      //11 --> 7

                            ((3 << 11) << 16) | (4 << 11),      //12 --> 4
                            ((2 << 11) << 16) | (5 << 11),      //13 --> 5
                            ((1 << 11) << 16) | (6 << 11),      //14 --> 6
                            ((0 << 11) << 16) | (7 << 11),      //15 --> 7
                          };
const uint32_t rqqmape[] = {
                            (1 << 2) << 16,                     // 0
                            (1 << 2) << 16,                     // 1
                            (1 << 2) << 16,                     // 2
                            (1 << 2) << 16,                     // 3
                            (1 << 2),                           // 4
                            (1 << 2),                           // 5
                            (1 << 2),                           // 6
                            (1 << 2),                           // 7

                            (1 << 2) << 16,                     // 8
                            (1 << 2) << 16,                     // 9
                            (1 << 2) << 16,                     //10
                            (1 << 2) << 16,                     //11
                            (1 << 2),                           //12
                            (1 << 2),                           //13
                            (1 << 2),                           //14
                            (1 << 2),                           //15
                          };
#define mp_machine_nibble_write(x, v) GPIOD->BSRR = rqqmapd[v]; GPIOE->BSRR = rqqmape[v];
# else
void mp_machine_nibble_write(mp_machine_soft_qspi_obj_t *self, uint8_t v) {
    mp_hal_pin_write(self->sio0, v & 1);
    mp_hal_pin_write(self->sio1, (v >> 1) & 1);
    mp_hal_pin_write(self->sio2, (v >> 2) & 1);
    mp_hal_pin_write(self->sio3, (v >> 3) & 1);
}
# endif

#ifdef MXX_FLASH
/*
 * D11  D12 E2  D13
 */
const uint8_t qqmap[] = { 0, 1, 2, 3, 8, 9, 10, 11};

#define mp_machine_nibble_read(x) (qqmap[((GPIOD->IDR >> 11) & 7)] | (GPIOE->IDR & 4))
#define mp_machine_dbl_read() ((GPIOD->IDR >> 11) & 3)
#else
/*
 * D11  D12 E2  D13
 * E14      E13  
 */
const uint8_t qqmap[] = { 0, 1, 2, 3, 8, 9, 10, 11};

#define mp_machine_nibble_read(x) (qqmap[((GPIOD->IDR >> 11) & 7)] | (GPIOE->IDR & 4))
#define mp_machine_dbl_read() ((GPIOD->IDR >> 11) & 3)
#endif
#define mp_machine_clk_high() (GPIOB->BSRR = (1 << 2))
#define mp_machine_clk_low() (GPIOB->BSRR = (1 << (2+16)))

#define mp_machine_all_in() mp_hal_pin_input(self->sio2); mp_hal_pin_input(self->sio3); mp_hal_pin_input(self->sio0); mp_hal_pin_input(self->sio1);
#define mp_machine_all_out() mp_hal_pin_output(self->sio2); mp_hal_pin_output(self->sio3); mp_hal_pin_output(self->sio0); mp_hal_pin_output(self->sio1);
#define mp_machine_dbl_in() mp_hal_pin_input(self->sio0); mp_hal_pin_input(self->sio1);
#define mp_machine_dbl_out() mp_hal_pin_output(self->sio0); mp_hal_pin_output(self->sio1);

#else   // NON TURBO

void mp_machine_nibble_write(mp_machine_soft_qspi_obj_t *self, uint8_t v) {
    mp_hal_pin_write(self->sio0, v & 1);
    mp_hal_pin_write(self->sio1, (v >> 1) & 1);
    mp_hal_pin_write(self->sio2, (v >> 2) & 1);
    mp_hal_pin_write(self->sio3, (v >> 3) & 1);
}

#define mp_machine_nibble_read(x) ( mp_hal_pin_read(self->sio0) | (mp_hal_pin_read(self->sio1) << 1) | (mp_hal_pin_read(self->sio2) << 2) | (mp_hal_pin_read(self->sio3) << 3) )
#define mp_machine_dbl_read() ( mp_hal_pin_read(self->sio0) | (mp_hal_pin_read(self->sio1) << 1) )
#define mp_machine_clk_high() mp_hal_pin_write(self->sck, 1)
#define mp_machine_clk_low() mp_hal_pin_write(self->sck, 0)
#define mp_machine_all_in() mp_hal_pin_input(self->sio2); mp_hal_pin_input(self->sio3); mp_hal_pin_input(self->sio0); mp_hal_pin_input(self->sio1);
#define mp_machine_all_out() mp_hal_pin_output(self->sio2); mp_hal_pin_output(self->sio3); mp_hal_pin_output(self->sio0); mp_hal_pin_output(self->sio1);
#define mp_machine_dbl_in() mp_hal_pin_input(self->sio0); mp_hal_pin_input(self->sio1);
#define mp_machine_dbl_out() mp_hal_pin_output(self->sio0); mp_hal_pin_output(self->sio1);

#endif

/*
bsize	SPI	    QSPI	QSPI-FAST
  512   0.7	    0.8	    1.4
 4096   0.8	    1.3	    3.5
10240   0.8	    1.4	    4.0
*/

void mp_machine_soft_qspi_qread(mp_obj_base_t *self_in, size_t len, uint8_t *buf) {
    mp_machine_soft_qspi_obj_t *self = (mp_machine_soft_qspi_obj_t*)self_in;

    // TODO check for proper configuration

    // only MSB transfer is implemented

    // make all pins IN
    mp_machine_all_in();

    // will run as fast as possible, limited only by CPU speed and GPIO time.
        #if 0
        uint8_t data_in;
        for (size_t i = 0; i < len; ++i) {
            //mp_hal_pin_write(self->sck, 1);
            mp_machine_clk_high();
            //mp_hal_pin_write(self->sck, 1);
            data_in = mp_machine_nibble_read();
            //mp_hal_pin_write(self->sck, 0);
            mp_machine_clk_low();
            //mp_hal_pin_write(self->sck, 0);
            //mp_hal_pin_write(self->sck, 1);
            mp_machine_clk_high();
            //mp_hal_pin_write(self->sck, 1);
            buf[i] = (data_in << 4) | mp_machine_nibble_read();
            //mp_hal_pin_write(self->sck, 0);
            mp_machine_clk_low();
            //mp_hal_pin_write(self->sck, 0);
        }
        #else
        while (len--) {
            mp_machine_clk_high();
            uint8_t data_in = mp_machine_nibble_read();
            mp_machine_clk_low();
            mp_machine_clk_high();
            *buf++ = (data_in << 4) | mp_machine_nibble_read();
            mp_machine_clk_low();
        }
        #endif
}

void mp_machine_soft_qspi_dread(mp_obj_base_t *self_in, size_t len, uint8_t *buf) {
    mp_machine_soft_qspi_obj_t *self = (mp_machine_soft_qspi_obj_t*)self_in;

    // TODO check for proper configuration

    // only MSB transfer is implemented

    // make all pins IN
    mp_machine_dbl_in();

    // will run as fast as possible, limited only by CPU speed and GPIO time.
        while (len--) {
#if defined(MICROPY_HW_QSPI_SLOW) && (MICROPY_HW_QSPI_SLOW == 1)
            #if 0
            uint8_t data_in = 0;
            for (int j = 6; j >= 0; j -= 2) {
                mp_machine_clk_high();
                mp_machine_clk_high();
                data_in |= mp_machine_dbl_read() << j;
                mp_machine_clk_low();
            }
            *buf++ = data_in;
            #else
            mp_machine_clk_high();
            mp_machine_clk_high();
            uint8_t data_in = mp_machine_dbl_read();
            mp_machine_clk_low();
            mp_machine_clk_high();
            mp_machine_clk_high();
            data_in = (data_in << 2) | mp_machine_dbl_read();
            mp_machine_clk_low();
            mp_machine_clk_high();
            mp_machine_clk_high();
            data_in = (data_in << 2) | mp_machine_dbl_read();
            mp_machine_clk_low();
            mp_machine_clk_high();
            mp_machine_clk_high();
            *buf++ = (data_in << 2) | mp_machine_dbl_read();
            mp_machine_clk_low();
            #endif
#else
            mp_machine_clk_high();
            uint8_t data_in = mp_machine_dbl_read();
            mp_machine_clk_low();
            mp_machine_clk_high();
            data_in = (data_in << 2) | mp_machine_dbl_read();
            mp_machine_clk_low();
            mp_machine_clk_high();
            data_in = (data_in << 2) | mp_machine_dbl_read();
            mp_machine_clk_low();
            mp_machine_clk_high();
            *buf++ = (data_in << 2) | mp_machine_dbl_read();
            mp_machine_clk_low();
#endif
        }
}

void mp_machine_soft_qspi_qwrite(mp_obj_base_t *self_in, size_t len, const uint8_t *buf) {
    mp_machine_soft_qspi_obj_t *self = (mp_machine_soft_qspi_obj_t*)self_in;

    // only MSB transfer is implemented

    mp_machine_all_out();
    // will run as fast as possible, limited only by CPU speed and GPIO time.
        for (size_t i = 0; i < len; ++i) {
            mp_machine_nibble_write(self, buf[i] >> 4);
            mp_machine_clk_high();
            mp_machine_clk_low();

            mp_machine_nibble_write(self, buf[i]);
            mp_machine_clk_high();
            mp_machine_clk_low();
        }
        //mp_hal_pin_input(self->sio1);
}

/******************************************************************************/
// MicroPython bindings for generic machine.SPI

STATIC mp_obj_t mp_machine_soft_qspi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args);

mp_obj_t mp_machine_qspi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check the id argument, if given
    if (n_args > 0) {
        if (args[0] != MP_OBJ_NEW_SMALL_INT(-1)) {
            #if defined(MICROPY_PY_MACHINE_QSPI_MAKE_NEW)
            // dispatch to port-specific constructor
            extern mp_obj_t MICROPY_PY_MACHINE_QSPI_MAKE_NEW(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args);
            return MICROPY_PY_MACHINE_QSPI_MAKE_NEW(type, n_args, n_kw, args);
            #else
            mp_raise_ValueError("invalid SPI peripheral");
            #endif
        }
        --n_args;
        ++args;
    }

    // software QSPI
    return mp_machine_soft_qspi_make_new(type, n_args, n_kw, args);
}

STATIC mp_obj_t machine_qspi_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[0]);
    mp_machine_qspi_p_t *qspi_p = (mp_machine_qspi_p_t*)s->type->protocol;
    qspi_p->init(s, n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_qspi_init_obj, 1, machine_qspi_init);

STATIC mp_obj_t machine_qspi_deinit(mp_obj_t self) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(self);
    mp_machine_qspi_p_t *qspi_p = (mp_machine_qspi_p_t*)s->type->protocol;
    if (qspi_p->deinit != NULL) {
        qspi_p->deinit(s);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_qspi_deinit_obj, machine_qspi_deinit);

STATIC void mp_machine_qspi_transfer(mp_obj_t self, size_t len, const void *src, void *dest) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(self);
    mp_machine_qspi_p_t *qspi_p = (mp_machine_qspi_p_t*)s->type->protocol;
    qspi_p->transfer(s, len, src, dest);
}
#if 1
STATIC void mp_machine_qspi_qread(mp_obj_t self, size_t len, void *buf) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(self);
    mp_machine_qspi_p_t *qspi_p = (mp_machine_qspi_p_t*)s->type->protocol;
    qspi_p->qread(s, len, buf);
}

STATIC void mp_machine_qspi_qwrite(mp_obj_t self, size_t len, const void *buf) {
    mp_obj_base_t *s = (mp_obj_base_t*)MP_OBJ_TO_PTR(self);
    mp_machine_qspi_p_t *qspi_p = (mp_machine_qspi_p_t*)s->type->protocol;
    qspi_p->qwrite(s, len, buf);
}
#endif

#if 1
STATIC mp_obj_t mp_machine_qspi_read(size_t n_args, const mp_obj_t *args) {
    vstr_t vstr;
    vstr_init_len(&vstr, mp_obj_get_int(args[1]));
    memset(vstr.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, vstr.len);
    mp_machine_qspi_qread(args[0], vstr.len, vstr.buf);
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_qspi_read_obj, 2, 3, mp_machine_qspi_read);

STATIC mp_obj_t mp_machine_qspi_readinto(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_WRITE);
    memset(bufinfo.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, bufinfo.len);
    mp_machine_qspi_qread(args[0], bufinfo.len, bufinfo.buf);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_qspi_readinto_obj, 2, 3, mp_machine_qspi_readinto);

STATIC mp_obj_t mp_machine_qspi_write(mp_obj_t self, mp_obj_t wr_buf) {
    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);
    mp_machine_qspi_qwrite(self, src.len, (const uint8_t*)src.buf);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_qspi_write_obj, mp_machine_qspi_write);

STATIC mp_obj_t mp_machine_qspi_write_readinto(mp_obj_t self, mp_obj_t wr_buf, mp_obj_t rd_buf) {
    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);
    if (rd_buf == mp_const_none) {
        mp_machine_qspi_transfer(self, src.len, src.buf, NULL);
    } else {
        mp_buffer_info_t dest;
        mp_get_buffer_raise(rd_buf, &dest, MP_BUFFER_WRITE);
        if (src.len != dest.len) {
            mp_raise_ValueError("buffers must be the same length");
        }
        mp_machine_qspi_transfer(self, src.len, src.buf, dest.buf);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_qspi_write_readinto_obj, mp_machine_qspi_write_readinto);

#endif

STATIC const mp_rom_map_elem_t machine_qspi_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_qspi_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&machine_qspi_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_machine_qspi_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_machine_qspi_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_machine_qspi_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_readinto), MP_ROM_PTR(&mp_machine_qspi_write_readinto_obj) },
};

MP_DEFINE_CONST_DICT(mp_machine_qspi_locals_dict, machine_qspi_locals_dict_table);

/******************************************************************************/
// Implementation of soft QSPI
#if 0
STATIC uint32_t baudrate_from_delay_half(uint32_t delay_half) {
    return 10000000;
}

STATIC uint32_t baudrate_to_delay_half(uint32_t baudrate) {
    return 0;
}
#endif

STATIC void mp_machine_soft_qspi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    mp_machine_soft_qspi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "SoftQSPI( sck=" MP_HAL_PIN_FMT ", sio0=" MP_HAL_PIN_FMT ", sio1=" MP_HAL_PIN_FMT 
            ", sio2=" MP_HAL_PIN_FMT ", sio3=" MP_HAL_PIN_FMT ", cs=" MP_HAL_PIN_FMT ")",
            mp_hal_pin_name(self->sck), mp_hal_pin_name(self->sio0), mp_hal_pin_name(self->sio1),
            mp_hal_pin_name(self->sio2), mp_hal_pin_name(self->sio3), mp_hal_pin_name(self->cs));
}

STATIC mp_obj_t mp_machine_soft_qspi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    //enum { ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit, ARG_sck, ARG_sio0, ARG_sio1, ARG_sio2, ARG_sio3 };
    enum { ARG_sck, ARG_sio0, ARG_sio1, ARG_sio2, ARG_sio3, ARG_cs };
    static const mp_arg_t allowed_args[] = {
        //{ MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 500000} },
        //{ MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        //{ MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        //{ MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8} },
        //{ MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = MICROPY_PY_MACHINE_SPI_MSB} },
        { MP_QSTR_sck,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio0,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio1,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio2,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio3,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_cs,       MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // create new object
    mp_machine_soft_qspi_obj_t *self = m_new_obj(mp_machine_soft_qspi_obj_t);
    self->base.type = &mp_machine_soft_qspi_type;

    // set parameters
        if (args[ARG_sck].u_obj == MP_OBJ_NULL
            || args[ARG_sio0].u_obj == MP_OBJ_NULL
            || args[ARG_sio1].u_obj == MP_OBJ_NULL
            || args[ARG_sio3].u_obj == MP_OBJ_NULL
            || args[ARG_sio2].u_obj == MP_OBJ_NULL
            || args[ARG_cs].u_obj == MP_OBJ_NULL) {
            mp_raise_ValueError("must specify all of sck/sio0/sio1/sio2/sio3/cs");
        }
        self->sck = mp_hal_get_pin_obj(args[ARG_sck].u_obj);
        self->sio0 = mp_hal_get_pin_obj(args[ARG_sio0].u_obj);
        self->sio1 = mp_hal_get_pin_obj(args[ARG_sio1].u_obj);
        self->sio2 = mp_hal_get_pin_obj(args[ARG_sio2].u_obj);
        self->sio3 = mp_hal_get_pin_obj(args[ARG_sio3].u_obj);
        self->cs = mp_hal_get_pin_obj(args[ARG_cs].u_obj);

        // configure pins
        mp_hal_pin_output(self->cs);
        mp_hal_pin_output(self->sck);
        mp_hal_pin_output(self->sio0);
        mp_hal_pin_write(self->sio2, 1);
        mp_hal_pin_output(self->sio2);
        mp_hal_pin_write(self->sio3, 1);
        mp_hal_pin_output(self->sio3);
        mp_hal_pin_input(self->sio1);

    return MP_OBJ_FROM_PTR(self);
}

STATIC void mp_machine_soft_qspi_init(mp_obj_base_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    mp_machine_soft_qspi_obj_t *self = (mp_machine_soft_qspi_obj_t*)self_in;

    //enum { ARG_baudrate, ARG_polarity, ARG_phase, ARG_sck, ARG_sio0, ARG_sio1, ARG_sio2, ARG_sio3 };
    enum { ARG_sck, ARG_sio0, ARG_sio1, ARG_sio2, ARG_sio3, ARG_cs };
    static const mp_arg_t allowed_args[] = {
        //{ MP_QSTR_baudrate, MP_ARG_INT, {.u_int = -1} },
        //{ MP_QSTR_polarity, MP_ARG_INT, {.u_int = -1} },
        //{ MP_QSTR_phase, MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_sck,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio0, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio1, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio2, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sio3, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_cs,   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    if (args[ARG_sck].u_obj != MP_OBJ_NULL) {
        self->sck = mp_hal_get_pin_obj(args[ARG_sck].u_obj);
    }
    if (args[ARG_sio0].u_obj != MP_OBJ_NULL) {
        self->sio0 = mp_hal_get_pin_obj(args[ARG_sio0].u_obj);
    }
    if (args[ARG_sio1].u_obj != MP_OBJ_NULL) {
        self->sio1 = mp_hal_get_pin_obj(args[ARG_sio1].u_obj);
    }
    if (args[ARG_sio2].u_obj != MP_OBJ_NULL) {
        self->sio2 = mp_hal_get_pin_obj(args[ARG_sio2].u_obj);
    }
    if (args[ARG_sio3].u_obj != MP_OBJ_NULL) {
        self->sio3 = mp_hal_get_pin_obj(args[ARG_sio3].u_obj);
    }
    if (args[ARG_cs].u_obj != MP_OBJ_NULL) {
        self->cs = mp_hal_get_pin_obj(args[ARG_cs].u_obj);
    }

    // configure pins
    mp_hal_pin_write(self->sck, 0);
    mp_hal_pin_output(self->sck);
    mp_hal_pin_write(self->sck, 1);
    mp_hal_pin_output(self->cs);
    mp_hal_pin_output(self->sio0);
    mp_hal_pin_input(self->sio1);
    mp_hal_pin_write(self->sio2, 1);
    mp_hal_pin_output(self->sio2);
    mp_hal_pin_write(self->sio3, 1);
    mp_hal_pin_output(self->sio3);
}

STATIC const mp_machine_qspi_p_t mp_machine_soft_qspi_p = {
    .init = mp_machine_soft_qspi_init,
    .deinit = NULL,
    .transfer = mp_machine_soft_qspi_transfer,
    .qread = mp_machine_soft_qspi_qread,
    .qwrite = mp_machine_soft_qspi_qwrite,
    .dread = mp_machine_soft_qspi_dread,
};

const mp_obj_type_t mp_machine_soft_qspi_type = {
    { &mp_type_type },
    .name = MP_QSTR_SoftQSPI,
    .print = mp_machine_soft_qspi_print,
    .make_new = mp_machine_qspi_make_new, // delegate to master constructor
    .protocol = &mp_machine_soft_qspi_p,
    .locals_dict = (mp_obj_dict_t*)&mp_machine_qspi_locals_dict,
};

#endif // MICROPY_HW_QSPIFLASH_SIZE_BITS
