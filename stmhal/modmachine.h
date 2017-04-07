/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2015 Damien P. George
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

#ifndef __MICROPY_INCLUDED_STMHAL_MODMACHINE_H__
#define __MICROPY_INCLUDED_STMHAL_MODMACHINE_H__

#include "py/mpstate.h"
#include "py/nlr.h"
#include "py/obj.h"

#if MICROPY_HW_HAS_WIM
#include "wim.h"
#endif
#if MICROPY_HW_HAS_OLED
#include "oled.h"
#endif
//#include "watchdog.h"
#if MICROPY_HW_HAS_QSPI
#include "qspi.h"
#endif
#if MICROPY_HW_HAS_USB_FIFO
#include "usb_fifo.h"
#endif

#if MICROPY_HW_HAS_MRT
#include "mrt.h"
#endif

void machine_init(void);

MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_info_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_unique_id_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_reset_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_bootloader_obj);
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_freq_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_sleep_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_deepsleep_obj);

#endif // __MICROPY_INCLUDED_STMHAL_MODMACHINE_H__
