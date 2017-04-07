/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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
#include STM32_HAL_H

#include "py/nlr.h"
#include "py/runtime.h"

#if MICROPY_HW_HAS_USB_FIFO

#include "pin.h"
#include "genhdr/pins.h"
#include "bufhelper.h"
#include "usb_fifo.h"

/// \moduleref pyb
/// \class USB_FIFO - USB_FIFO control
///
/// The USB_FIFO class is used to control the USB_FIFO
///
///     usb_fifo = pyb.USB_FIFO(buffer, itme_len)
///
/// Then you can use:
///

#if 0
typedef struct _pyb_usb_fifo_obj_t {
    mp_obj_base_t base;

    // hardware control for the USB_FIFO
    uint8_t *buffer;
    volatile uint32_t fifo_cnt;
    volatile uint32_t out_ix;
    uint32_t item_len;
    uint32_t fifo_size;
} pyb_usb_fifo_obj_t;
#endif

volatile pyb_usb_fifo_obj_t *me = NULL;

/// \classmethod \constructor(buffer, item_size)
///
/// Construct an USB_FIFO object
STATIC mp_obj_t pyb_usb_fifo_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    int fifo_len = 0;
    int len = 0;
    int item_len = 0;
    uint8_t *p = NULL;
    mp_arg_check_num(n_args, n_kw, 2, 2, false);

    if (MP_OBJ_IS_TYPE(args[1], &mp_type_int)) {
        item_len = mp_obj_get_int(args[1]);
    } else {
        item_len = mp_obj_get_int(args[1]);
    }
    if (item_len < 1) {
printf("usb_fifo_new %d\n", item_len);
        return MP_OBJ_NULL;
    }

    if (MP_OBJ_IS_TYPE(args[0], &mp_type_bytearray)) {
        mp_buffer_info_t bufinfo_fifo;
        mp_get_buffer(args[0], &bufinfo_fifo, MP_BUFFER_RW);
        if ( bufinfo_fifo.typecode == 0) {
            len = bufinfo_fifo.len;
            p = bufinfo_fifo.buf;
            fifo_len = len/item_len;
            if (fifo_len < 2) {
printf("usb_fifo_new fifo_len %d\n", fifo_len);
                return MP_OBJ_NULL;
            }
        } else {
printf("usb_fifo_new typecode %d\n", bufinfo_fifo.typecode);
            return MP_OBJ_NULL;
        }
    } else {
printf("usb_fifo_new typecode %p\n", args[0]);
        return MP_OBJ_NULL;
    }

    printf("usb_fifo: %d %d %d %p\n", len, item_len, fifo_len, p);

    // create usb_fifo object
    //pyb_usb_fifo_obj_t *usb_fifo = m_new_obj(pyb_usb_fifo_obj_t);
    me = m_new_obj(pyb_usb_fifo_obj_t);
    me->base.type = &pyb_usb_fifo_type;
    me->item_len = item_len;
    me->fifo_size = fifo_len;
    me->fifo_cnt = 0;
    me->out_ix = 0;
    me->buffer = p;
    return (pyb_usb_fifo_obj_t *)me;
}

void fifo_put(pyb_usb_fifo_obj_t *fifo, uint8_t *p, int len) {
    // fill fifo...
    //printf("fifo write %d %d\n", len, cnt);
#if 0
    int ix = fifo->item_len*(fifo->fifo_cnt % fifo->fifo_size);
    int blen = fifo->item_len*fifo->fifo_size;
    for (int i = 0; i < len; i++) {
        fifo->buffer[(ix+i)%blen] = p[i];
    }
    fifo->fifo_cnt += len/fifo->item_len;
#else
    uint32_t fc = fifo->fifo_cnt;
    int ix = fifo->item_len*(fc % fifo->fifo_size);
    int blen = fifo->item_len*fifo->fifo_size;
    for (int i = 0; i < len; i++) {
        fifo->buffer[(ix+i)%blen] = p[i];
    }
    fifo->fifo_cnt = fc+len/fifo->item_len;
#endif
}

void fifo_putw(pyb_usb_fifo_obj_t *fifo, uint8_t *p, int len) {
    // fill fifo...
    //printf("fifo write %d %d\n", len, cnt);
#if 1
    int ix = fifo->item_len*(fifo->fifo_cnt % fifo->fifo_size);
    int blen = fifo->item_len*fifo->fifo_size;
    for (int i = 0; i < len; i++) {
        while ((fifo->fifo_cnt-fifo->out_ix) >= fifo->fifo_size) {
//printf("w: %u %u\n", (unsigned int)fifo->fifo_cnt, (unsigned int)fifo->out_ix);
        }
        fifo->buffer[(ix+i)%blen] = p[i];
        if ((i % fifo->item_len) == 0) {
            fifo->fifo_cnt += 1;
        }
    }
#else
    for (int i = 0; i < cnt; i++) {
        int ix = fifo->fifo_cnt;
        int ox = fifo->out_ix;
        while ((ix-ox) >= fifo->fifo_size) {
        }
        ix %= fifo->fifo_size;
        ox %= fifo->fifo_size;
        for (int j = 0; j < fifo->item_len; j++) {
            fifo->buffer[ix*fifo->item_len+j] = p[i*fifo->item_len+j];
        }
        fifo->fifo_cnt++;
    }
#endif
}

/// \method write(data)
///
/// Write the bytearray data to the fifo.
/// len(data) is multiple of item_len
STATIC mp_obj_t pyb_usb_fifo_write(mp_obj_t self_in, mp_obj_t data) {
    pyb_usb_fifo_obj_t *self = self_in;
#if 0
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(str, &len);
    usb_fifo_write_strn(self, data, len);
#endif
    if (MP_OBJ_IS_TYPE(data, &mp_type_bytearray)) {
        mp_buffer_info_t bufinfo_data;
        mp_get_buffer(data, &bufinfo_data, MP_BUFFER_RW);
        if ( bufinfo_data.typecode == 0) {
            int len = bufinfo_data.len;
            int cnt = len/self->item_len;
            uint8_t *p = bufinfo_data.buf;
            if ( cnt*self->item_len == len ) {
                fifo_put(self, p, len);
            } else {
                printf("ERR item len missmatch\n");
                //return MP_OBJ_NULL;
            }
        } else {
            printf("ERR typecode missmatch %d\n", bufinfo_data.typecode);
            //return MP_OBJ_NULL;
        }
    } else if MP_OBJ_IS_STR(data) {
        mp_uint_t len;
        const char *p = mp_obj_str_get_data(data, &len);
        int cnt = len/self->item_len;
        if ( cnt*self->item_len == len ) {
            fifo_put(self, (uint8_t *)p, len);
        } else {
            printf("ERR item len missmatch\n");
            //return MP_OBJ_NULL;
        }
    } else {
        printf("ERR type missmatch\n");
        //return MP_OBJ_NULL;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_usb_fifo_write_obj, pyb_usb_fifo_write);

/// \method wait(data)
///
/// Write the bytearray data to the fifo.
/// len(data) is multiple of item_len
STATIC mp_obj_t pyb_usb_fifo_wait(mp_obj_t self_in, mp_obj_t data) {
    pyb_usb_fifo_obj_t *self = self_in;
#if 0
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(str, &len);
    usb_fifo_write_strn(self, data, len);
#endif
    if (MP_OBJ_IS_TYPE(data, &mp_type_bytearray)) {
        mp_buffer_info_t bufinfo_data;
        mp_get_buffer(data, &bufinfo_data, MP_BUFFER_RW);
        if ( bufinfo_data.typecode == 0) {
            int len = bufinfo_data.len;
            int cnt = len/self->item_len;
            uint8_t *p = bufinfo_data.buf;
            if ( cnt*self->item_len == len ) {
                fifo_putw(self, p, len);
            } else {
                printf("ERR item len missmatch\n");
                //return MP_OBJ_NULL;
            }
        } else {
            printf("ERR typecode missmatch %d\n", bufinfo_data.typecode);
            //return MP_OBJ_NULL;
        }
    } else if MP_OBJ_IS_STR(data) {
        mp_uint_t len;
        const char *p = mp_obj_str_get_data(data, &len);
        int cnt = len/self->item_len;
        if ( cnt*self->item_len == len ) {
            fifo_putw(self, (uint8_t *)p, len);
        } else {
            printf("ERR item len missmatch\n");
            //return MP_OBJ_NULL;
        }
    } else {
        printf("ERR type missmatch\n");
        //return MP_OBJ_NULL;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_usb_fifo_wait_obj, pyb_usb_fifo_wait);

/// \method get()
///
/// Get the (size, item_len, cnt, out_ix)
///
STATIC mp_obj_t pyb_usb_fifo_get(mp_obj_t self_in) {
    pyb_usb_fifo_obj_t *self = self_in;
    //printf("usb_fifo_get %p,%p\n", x_in, y_in);
    mp_obj_t tuple[] = {
                             mp_obj_new_int((uint32_t)self->fifo_size),
                             mp_obj_new_int((uint32_t)self->item_len),
                             mp_obj_new_int((uint32_t)self->fifo_cnt),
                             mp_obj_new_int((uint32_t)self->out_ix),
                        };
    return mp_obj_new_tuple(MP_ARRAY_SIZE(tuple), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_fifo_get_obj, pyb_usb_fifo_get);

/*
 */
STATIC mp_obj_t pyb_usb_fifo_reset(mp_obj_t self_in) {
    pyb_usb_fifo_obj_t *self = self_in;
    self->out_ix = self->fifo_cnt;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_fifo_reset_obj, pyb_usb_fifo_reset);

STATIC const mp_map_elem_t pyb_usb_fifo_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_write), (mp_obj_t)&pyb_usb_fifo_write_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_wait), (mp_obj_t)&pyb_usb_fifo_wait_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get), (mp_obj_t)&pyb_usb_fifo_get_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_reset), (mp_obj_t)&pyb_usb_fifo_reset_obj },
};

STATIC MP_DEFINE_CONST_DICT(pyb_usb_fifo_locals_dict, pyb_usb_fifo_locals_dict_table);

const mp_obj_type_t pyb_usb_fifo_type = {
    { &mp_type_type },
    .name = MP_QSTR_USB_FIFO,
    .make_new = pyb_usb_fifo_make_new,
    .locals_dict = (mp_obj_t)&pyb_usb_fifo_locals_dict,
};

void usb_fifo(uint8_t *buf, uint16_t len) {
    if (me != NULL) {
        uint16_t *wp = (uint16_t *)buf;
        uint32_t *qp = (uint32_t *)buf;
        uint32_t avail = (me->fifo_cnt - me->out_ix)/me->item_len;
//printf("wmf %u %u", (unsigned int)len, (unsigned int)avail);
        if (avail > me->fifo_size) {
            avail = 0;
        }
        uint32_t nrec = (len-16)/(me->item_len);
//printf(" %u", (unsigned int)nrec);
        if (nrec > avail) {
            nrec = avail;
        }
        qp[0] = me->out_ix;
        qp[1] = me->fifo_cnt;
        wp[4] = avail;
        wp[5] = me->item_len;
        wp[6] = nrec;
        wp[7] = 0;
//printf(" %u %u %u %u\n", (unsigned int)len, (unsigned int)qp[0], (unsigned int)qp[1], (unsigned int)avail);
        if (nrec) {
            uint8_t *dp = (uint8_t *)&buf[16];
            for (int i = 0; i < nrec; i++) {
                uint32_t ix = me->item_len*(me->out_ix % me->fifo_size);
                uint8_t *xp = &me->buffer[ix];
                //int ix = nrec;
                for (int j = 0; j < me->item_len; j++) {
                    //buf[16+i*me->item_len+j] = xp[j];
                    *dp = xp[j];
                    dp++;
                }
                me->out_ix++;
            }
        } else {
            me->out_ix = me->fifo_cnt;
        }
#if 0
#endif
    }
}

#endif // MICROPY_HW_HAS_USB_FIFO
