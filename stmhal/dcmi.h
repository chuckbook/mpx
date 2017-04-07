/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 206 chuckbook
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
#if defined(MICROPY_HW_ENABLE_DCMI) && MICROPY_HW_ENABLE_DCMI
#include "dma.h"

typedef struct _pyb_dcmi_obj_t {
    mp_obj_base_t base;
    //uint32_t DCMI_channel; // DCMI_CHANNEL_1 or DCMI_CHANNEL_2
    DCMI_HandleTypeDef *dcmi;
    const dma_descr_t *rx_dma_descr;
    //uint16_t pin; // GPIO_PIN_4 or GPIO_PIN_5
    //uint8_t bits; // 8 or 12
    //uint8_t state;
    uint32_t w;
    uint32_t h;
    uint32_t xoff;
    uint32_t yoff;
    uint32_t needconf;
} pyb_dcmi_obj_t;


void dcmi_init0(void);
//void dcmi_init(DCMI_HandleTypeDef *dcmi);
void dcmi_init(const pyb_dcmi_obj_t *self);

extern const mp_obj_type_t pyb_dcmi_type;
#endif
