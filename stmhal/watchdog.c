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

#if MICROPY_HW_HAS_WDG

#include "pin.h"
#include "genhdr/pins.h"
#include "bufhelper.h"
#include "watchdog.h"

/// \moduleref pyb
/// \class WATCHDOG - WATCHDOG control
///
/// The WATCHDOG class is used to control the WATCHDOG
///
///     watchdog = pyb.watchdog(timeout)
///
/// Then you can use:
///

typedef struct _pyb_watchdog_obj_t {
    mp_obj_base_t base;

    // hardware control for the watchdog
    uint32_t timeout;
} pyb_watchdog_obj_t;

volatile pyb_watchdog_obj_t *wdg_me = NULL;

/// \classmethod \constructor(buffer, item_size)
///
/// Construct an watchdog object
STATIC mp_obj_t pyb_watchdog_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    int timeout = 0;
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    if (MP_OBJ_IS_TYPE(args[0], &mp_type_int)) {
        timeout = mp_obj_get_int(args[0]);
    } else {
        timeout = mp_obj_get_int(args[0]);
    }
    if (timeout < 1000) {
printf("watchdog too short %d us\n", timeout);
        return MP_OBJ_NULL;
    }

    int i;
    for (i = 0; ; ) {
        if (timeout < 512000) {
            break;
        } else {
            if (i >= 6) {
        // timeout too long...
printf("watchdog too long %d us\n", (1 << i)*timeout);
                return MP_OBJ_NULL;
            }
            i++;
            timeout /= 2;
        }
    }
    timeout /= 125;
    if ( timeout > 0xfff) {
    }
    printf("watchdog: %d us\n", (1 << i)*timeout*125);

    while (IWDG->SR & 2);
    IWDG->KR = 0x5555;
    IWDG->RLR = timeout;

    while (IWDG->SR & 1);
    IWDG->KR = 0x5555;
    IWDG->PR = i;

    IWDG->KR = 0xcccc;

    // create watchdog object
    //pyb_watchdog_obj_t *watchdog = m_new_obj(pyb_watchdog_obj_t);
    wdg_me = m_new_obj(pyb_watchdog_obj_t);
    wdg_me->base.type = &pyb_watchdog_type;
    wdg_me->timeout = timeout;
    return (pyb_watchdog_obj_t *)wdg_me;
}

/// \method trig()
///
/// Trigger watchdog
//STATIC mp_obj_t pyb_watchdog_trig(mp_obj_t self_in, mp_obj_t data) 
STATIC mp_obj_t pyb_watchdog_trig(mp_obj_t self_in)
{
    //pyb_watchdog_obj_t *self = self_in;
    IWDG->KR = 0xaaaa;
    return mp_obj_new_int( (((IWDG->PR & 7) == 7)?256:1 << ((IWDG->PR & 7))) * ((IWDG->RLR & 0xfff)+1)*125);
    //return mp_const_none;
}
//STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_watchdog_trig_obj, pyb_watchdog_trig);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_watchdog_trig_obj, pyb_watchdog_trig);

STATIC const mp_map_elem_t pyb_watchdog_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_trig), (mp_obj_t)&pyb_watchdog_trig_obj },
};

STATIC MP_DEFINE_CONST_DICT(pyb_watchdog_locals_dict, pyb_watchdog_locals_dict_table);

const mp_obj_type_t pyb_watchdog_type = {
    { &mp_type_type },
    .name = MP_QSTR_WATCHDOG,
    .make_new = pyb_watchdog_make_new,
    .locals_dict = (mp_obj_t)&pyb_watchdog_locals_dict,
};

#endif // MICROPY_HW_HAS_WATCHDOG
