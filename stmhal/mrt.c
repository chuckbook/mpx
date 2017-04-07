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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "py/runtime.h"
#include "py/binary.h"
#include "mrt.h"
#include "rtc.h"

#include "py/objstr.h"
#include "timer.h"
#include "systick.h"
#if MICROPY_HW_HAS_USB_FIFO
#include "usb_fifo.h"
#endif
#include "py/mphal.h"


/// \moduleref pyb
/// \class MRT - accelerometer control
///
/// Accel is an object that controls the accelerometer.  Example usage:
///
///     accel = pyb.Accel()
///     for i in range(10):
///         print(accel.x(), accel.y(), accel.z())
///
/// Raw values are between -32 and 31.


/******************************************************************************/
/* Micro Python bindings                                                      */

#if MICROPY_HW_HAS_MRT


typedef struct _pyb_mrt_obj_t {
    mp_obj_base_t base;
} pyb_mrt_obj_t;



/// \classmethod \constructor()
/// Create and return an accelerometer object.
///
/// Note: if you read accelerometer values immediately after creating this object
/// you will get 0.  It takes around 20ms for the first sample to be ready, so,
/// unless you have some other code between creating this object and reading its
/// values, you should put a `pyb.delay(20)` after creating it.  For example:
///
///     accel = pyb.Accel()
///     pyb.delay(20)
///     print(accel.x())



/// \method x()
/// Get the x-axis value.
STATIC mp_obj_t mrt_x(mp_obj_t self_in) {
    return mp_obj_new_int(666);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mrt_x_obj, mrt_x);



void mrt_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_mrt_obj_t *self = self_in;
    mp_printf(print, "LED(%lu)", self);
}



///

uint32_t cMAXX(uint32_t a) {
 return a*a;
}

///


/// \method MaXX()
/// test
/*
uint32_t mrt_MaXX(uint32_t SA) { //mp_obj_t
   //return mp_obj_new_int(77);
  //uint32_t i = 0;
  //uint32_t size = 8000;


  SA = 1233;

  return (uint32_t)(SA);
*/
STATIC mp_obj_t mrt_MaXX(mp_obj_t *list, mp_obj_t *newlist) {
    //48 bis 122
    static mp_int_t w = 74;
    static mp_int_t c1 = 0;
    static mp_int_t c2 = 0;
    mp_int_t t1,t2;

    t1 = (c2+48) * 256 + (c1+48);
    t2 = (122 - c2) * 256 + (122 - c1);

    c1 = (c1 + 1);

    if(c1 > w) {
     c1 = 0;
     c2 = c2 + 1;
     if(c2 > w) {
      c2 = 0;
     }
    }


    static mp_int_t i = 2;

    mp_obj_list_t *o = MP_OBJ_TO_PTR(newlist);
    t1 = mp_obj_get_int(o->items[0]);
    t2 = mp_obj_get_int(o->items[1]);

    if (MP_OBJ_IS_TYPE(list, &mp_type_bytearray)) {

      mp_buffer_info_t list_info;
      mp_get_buffer_raise(list, &list_info, MP_BUFFER_RW);
      mp_int_t len = list_info.len;
      byte *p = list_info.buf;
      uint16_t t = mp_obj_get_int(MP_OBJ_NEW_SMALL_INT(sys_tick_get_microseconds()));
      //t = 22874;

      p[i + 1] = (t & 0xFF);	//Zeit
      p[i] = ((t >> 8) & 0xFF);	//Zeit

      p[i + 3] = t1 & 0xFF;
      p[i + 2] = (t1 >> 8) & 0xFF;

      p[i + 5] = t2 & 0xFF;
      p[i + 4] = (t2 >> 8) & 0xFF;

      i = i + 8;
      i = (i)%len;
      if(i <= 2) {
	i = 2;
      }
    }
  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(mrt_MaXX_obj, mrt_MaXX);

STATIC mp_obj_t mrt_Mess(mp_obj_t list, mp_obj_t *self, mp_obj_t *self2) {
  static mp_int_t i = 2;	//pos des SPI bytearray
    TIM_HandleTypeDef *tim_a = pyb_timer_get_handle(self);
    TIM_HandleTypeDef *tim_b = pyb_timer_get_handle(self2);
    mp_uint_t t1 = tim_a->Instance->CNT;
    mp_uint_t t2 = tim_b->Instance->CNT;
  if (MP_OBJ_IS_TYPE(list, &mp_type_bytearray)) { //liste ist ein bytearray
      mp_buffer_info_t list_info;
      mp_get_buffer_raise(list, &list_info, MP_BUFFER_RW);
      byte *p = list_info.buf;
      mp_int_t len = list_info.len - 8;
      uint32_t t = mp_obj_get_int(MP_OBJ_NEW_SMALL_INT(sys_tick_get_microseconds()));

      p[i + 1] = (t & 0xFF);	//Zeit
      p[i] = ((t >> 8) & 0xFF);	//Zeit

      p[i + 3] = t1 & 0xFF;
      p[i + 2] = (t1 >> 8) & 0xFF;

      p[i + 5] = t2 & 0xFF;
      p[i + 4] = (t2 >> 8) & 0xFF;

      //p[i + 7] = 10;
      i = i + 8;
      i = (i)%len;
      if(i <= 2) {
	i = 2;
      }

  return mp_const_none;
  }
  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(mrt_Mess_obj, mrt_Mess);

STATIC mp_obj_t mrt_ZmaxZ(mp_obj_t list, mp_obj_t self,mp_obj_t self2) {
    mp_obj_list_t *o = MP_OBJ_TO_PTR(list); //geht
    static mp_int_t count = 0;
    mp_int_t len = o->len;

    mp_int_t i = 0;

    for (i = 0; i < len; i = i + 1) {
      o->items[(i+count)%(o->len)] = mp_obj_new_int(count);
    }

        RTC_DateTypeDef date;
        RTC_TimeTypeDef time;
        HAL_RTC_GetTime(&RTCHandle, &time, FORMAT_BIN);
        HAL_RTC_GetDate(&RTCHandle, &date, FORMAT_BIN);


        TIM_HandleTypeDef *tim_a = pyb_timer_get_handle(self);
        TIM_HandleTypeDef *tim_b = pyb_timer_get_handle(self2);
        mp_obj_t tuple[7] = {
            mp_obj_new_int(time.Seconds),
            mp_obj_new_int(time.SubSeconds),
            mp_obj_new_int(4350),
            mp_obj_new_int(count+i),
            mp_obj_new_int(mp_obj_get_int(o->items[0])),
            mp_obj_new_int(tim_a->Instance->CNT),
            mp_obj_new_int(tim_b->Instance->CNT),
        };
          tim_a->Instance->CNT = 0;
          tim_b->Instance->CNT = 0;
	count = count + 2;
        return mp_obj_new_tuple(7, tuple);
}

MP_DEFINE_CONST_FUN_OBJ_3(mrt_ZmaxZ_obj, mrt_ZmaxZ);

static uint32_t mrt_ts[2];
static uint8_t *mrt_ts8 = (uint8_t *)mrt_ts;
static uint16_t *mrt_ts16 = (uint16_t *)mrt_ts;
static uint32_t mrt_irqen = 0;

#if MICROPY_HW_HAS_USB_FIFO
volatile static pyb_usb_fifo_obj_t *tim1_fifo = NULL;
void fifo_put(pyb_usb_fifo_obj_t *fifo, uint8_t *p, int len);
#endif

#if 1
void OTG_FS_IRQHandler(void)
{
    extern PCD_HandleTypeDef pcd_fs_handle;
    if (mrt_irqen) enable_irq(0);
    HAL_PCD_IRQHandler(&pcd_fs_handle);
}
#endif

#define ALL_FIFO

void TIM1_UP_TIM10_IRQHandler(void) {
    GPIO_clear_pin(GPIOB, (1 << 7));
#ifdef USE_SYSTICK
    mrt_ts[0] = SysTick->VAL;
#else
    mrt_ts[0] = sys_tick_get_microseconds();
#endif
    mrt_ts16[2] = (uint16_t)(TIM1->CNT);
    //mrt_ts16[2] = (uint16_t)(TIM2->CNT);
    TIM2->CNT = 0;
    mrt_ts16[3] = (uint16_t)(TIM5->CNT);
    TIM5->CNT = 0;
#if MICROPY_HW_HAS_USB_FIFO
    if (tim1_fifo) {
        //__HAL_TIM_CLEAR_IT(&tim->tim, irq_mask);
        TIM1->SR = 0;
        uint32_t fc = tim1_fifo->fifo_cnt;
        #ifdef ALL_FIFO
        int rix = fc % tim1_fifo->fifo_size;
        int ix = tim1_fifo->item_len*rix;
        for (int i = 0; i < tim1_fifo->item_len; i++) {
            tim1_fifo->buffer[ix] = mrt_ts8[i];
            ix++;
        }
        #else
        int ix = 8*(fc & (1024-1));
        uint32_t *p32 = (uint32_t *)tim1_fifo->buffer;
        p32[((ix+0)&0x1fff)>>2] = mrt_ts[0];
        p32[((ix+4)&0x1fff)>>2] = mrt_ts[1];
        #endif
        tim1_fifo->fifo_cnt = fc+1;
    } else {
        timer_irq_handler(1);
    }
#else
    timer_irq_handler(1);
#endif
    timer_irq_handler(10);
    GPIO_clear_pin(GPIOB, (1 << 7));
}

STATIC mp_obj_t mrt_resetmax(mp_obj_t self_in, mp_obj_t data) {
    mp_obj_t res_obj = MP_OBJ_NULL;
    GPIO_set_pin(GPIOA, (1 << 13)); // red
    if (MP_OBJ_IS_TYPE(data, &mp_type_bytearray)) {
        mp_buffer_info_t bufinfo_data;
        mp_get_buffer(data, &bufinfo_data, MP_BUFFER_RW);
        if ( bufinfo_data.typecode == 0) {
            int len = bufinfo_data.len;
            if ( len >= 8 ) {
                volatile uint16_t *p16 = bufinfo_data.buf;
                volatile uint32_t *p32 = bufinfo_data.buf;
                p32[0] = mrt_ts[0];
                p16[2] = (uint16_t)mrt_ts16[2];
                p16[3] = (uint16_t)mrt_ts16[3];
                res_obj = mp_const_none;
            }
        }
    }

    GPIO_clear_pin(GPIOB, (1 << 7)); // red
    return res_obj;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mrt_resetmax_obj, mrt_resetmax);

STATIC mp_obj_t mrt_attach(mp_obj_t self_in, mp_obj_t data) {
    mp_obj_t res_obj = MP_OBJ_NULL;
#if MICROPY_HW_HAS_USB_FIFO
    if (MP_OBJ_IS_TYPE(data, &pyb_usb_fifo_type)) {
        tim1_fifo = (pyb_usb_fifo_obj_t *)data;
        //printf("attach --> fifo %p %u\n", data, (unsigned int)(fifo->item_len));

        // immediately push to usb_fifo....
        if (tim1_fifo->item_len != 8) {
            tim1_fifo = NULL;
        } else {
            res_obj = data;
        }
    } else {
        tim1_fifo = NULL;
    }
#endif
    return res_obj;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mrt_attach_obj, mrt_attach);

typedef struct _pyb_timer_channel_obj_t {
    mp_obj_base_t base;
    struct _pyb_timer_obj_t *timer;
    uint8_t channel;
    uint8_t mode;
    mp_obj_t callback;
    struct _pyb_timer_channel_obj_t *next;
} pyb_timer_channel_obj_t;

typedef struct _pyb_timer_obj_t {
    mp_obj_base_t base;
    uint8_t tim_id;
    uint8_t is_32bit;
    mp_obj_t callback;
    TIM_HandleTypeDef tim;
    IRQn_Type irqn;
    pyb_timer_channel_obj_t *channel;
} pyb_timer_obj_t;

STATIC mp_obj_t mrt_priomax(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_Timer, /* MP_ARG_REQUIRED | MP_ARG_KW_ONLY |*/ MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_priority, /*MP_ARG_KW_ONLY |*/ MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_sub, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_int, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[0].u_obj != mp_const_none) {
        if (MP_OBJ_IS_TYPE(args[0].u_obj, &pyb_timer_type)) {
        uint32_t irqn = ((pyb_timer_obj_t *)args[0].u_obj)->irqn;
        uint32_t prio;
        uint32_t sub;
        HAL_NVIC_GetPriority(irqn, HAL_NVIC_GetPriorityGrouping(), &prio, &sub);
        if (args[1].u_int >= 0) prio = args[1].u_int;
        if (args[2].u_int >= 0) sub = args[2].u_int;
        if (args[3].u_int >= 0) mrt_irqen = args[3].u_int;
        HAL_NVIC_SetPriority(irqn, prio, sub);
        mp_obj_t tuple[4] = {
          (MP_OBJ_NEW_SMALL_INT(irqn)),
          (MP_OBJ_NEW_SMALL_INT(prio)),
          (MP_OBJ_NEW_SMALL_INT(sub)),
          (MP_OBJ_NEW_SMALL_INT(mrt_irqen)),
        };
        return mp_obj_new_tuple(4, tuple);
        } else {
            return MP_OBJ_NULL;
        }
    } else {
        if (args[3].u_int >= 0) mrt_irqen = args[3].u_int;
        mp_obj_t tuple[300];
        tuple[0] = MP_OBJ_NEW_SMALL_INT(mrt_irqen);
        int i;
        int ix = 1;
        for (i = 4; i < 15; i++) {
            uint8_t h;
#if defined(MCU_SERIES_F7)
            if ((h = SCB->SHPR[(i-4)]))
#else
            if ((h = SCB->SHP[(i-4)]))
#endif
            {
                mp_obj_t dbl[2];
                dbl[0] = mp_obj_new_int(i-16);
                dbl[1] = mp_obj_new_int(h);
                tuple[ix++] = mp_obj_new_tuple(2, dbl);
            }
        }
        for (i = 0; i < 8; i++) {
            int j;
            for (j = 0; j < 32; j++) {
                if (NVIC->ISER[i] & (1 << j)) {
                    mp_obj_t dbl[2];
                    dbl[0] = mp_obj_new_int(i*32+j);
                    dbl[1] = mp_obj_new_int(NVIC->IP[i*32+j]);
                    tuple[ix++] = mp_obj_new_tuple(2, dbl);
                }
            }
        }
        return mp_obj_new_tuple(ix, tuple);
    }
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mrt_priomax_obj, 0, mrt_priomax);



STATIC mp_obj_t mrt_obj_make_new(mp_obj_t type_in, uint n_args, uint n_kw, const mp_obj_t *args) {
pyb_mrt_obj_t *self = m_new_obj(pyb_mrt_obj_t);
self->base.type = type_in;
return self;
}


STATIC const mp_map_elem_t mrt_locals_dict_table[] = {
    // TODO add init, deinit, and perhaps reset methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_x), (mp_obj_t)&mrt_x_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MaXX), (mp_obj_t)&mrt_MaXX_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ZmaxZ), (mp_obj_t)&mrt_ZmaxZ_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_Mess), (mp_obj_t)&mrt_Mess_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_resetmax), (mp_obj_t)&mrt_resetmax_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_attach), (mp_obj_t)&mrt_attach_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_priomax), (mp_obj_t)&mrt_priomax_obj },
};

STATIC MP_DEFINE_CONST_DICT(mrt_locals_dict, mrt_locals_dict_table);

const mp_obj_type_t pyb_mrt_type = {
    { &mp_type_type },
    .name = MP_QSTR_MRT,
    .make_new = mrt_obj_make_new,
    .print = mrt_obj_print,
    .locals_dict = (mp_obj_t)&mrt_locals_dict,
};

#endif // MICROPY_HW_HAS_MRT
