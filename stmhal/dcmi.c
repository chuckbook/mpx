/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 chuckbook
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
#include <string.h>

#include STM32_HAL_H

#include "py/nlr.h"
#include "py/runtime.h"
#include "timer.h"
#include "dcmi.h"
//#include "dma.h"
#include "pin.h"
#include "genhdr/pins.h"

/// \moduleref pyb
/// \class DCMI- digital to analog conversion
///
/// The DCMIis used to output analog values (a specific voltage) on pin X5 or pin X6.
/// The voltage will be between 0 and 3.3V.
///
/// *This module will undergo changes to the API.*
///
/// Example usage:
///
///     from pyb import DCMI
///
///     DCMI= DCMI1)            # create DCMI1 on pin X5
///     DCMIwrite(128)          # write a value to the DCMI(makes X5 1.65V)
///
/// To output a continuous sine-wave:
///
///     import math
///     from pyb import DCMI
///
///     # create a buffer containing a sine-wave
///     buf = bytearray(100)
///     for i in range(len(buf)):
///         buf[i] = 128 + int(127 * math.sin(2 * math.pi * i / len(buf)))
///
///     # output the sine-wave at 400Hz
///     DCMI= DCMI1)
///     DCMIwrite_timed(buf, 400 * len(buf), mode=DCMICIRCULAR)

#if defined(MICROPY_HW_ENABLE_DCMI) && MICROPY_HW_ENABLE_DCMI

STATIC DCMI_HandleTypeDef DCMIHandle = {.Instance = NULL};

#define DCMI_RX_DMA_STREAM (DMA2_Stream1)

/* DCMI GPIOs */
static const pin_obj_t *dcmi_pins[] = {
    &pyb_pin_DCMI_D0,
    &pyb_pin_DCMI_D1,
    &pyb_pin_DCMI_D2,
    &pyb_pin_DCMI_D3,
    &pyb_pin_DCMI_D4,
    &pyb_pin_DCMI_D5,
    &pyb_pin_DCMI_D6,
    &pyb_pin_DCMI_D7,
    &pyb_pin_DCMI_HS,
    &pyb_pin_DCMI_VS,
    &pyb_pin_DCMI_PIXCLK,
};

#define NUM_DCMI_PINS   (sizeof(dcmi_pins)/sizeof(dcmi_pins[0]))

DMA_HandleTypeDef  DMAHandle;

void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t newState) {
    if (newState != DISABLE) {
        DMAy_Streamx->CR |= (uint32_t)DMA_SxCR_EN;
    } else {
        DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
    }
}

uint32_t dcmi_ints[10];

static void DCMI_DMA_ErrorCallback(DMA_HandleTypeDef *hdma) {
    dcmi_ints[6] += 1;
}

static void DCMI_DMA_HalfCpltCallback(DMA_HandleTypeDef *hdma) {
    dcmi_ints[7] += 1;
}

static void DCMI_DMA_M1CpltCallback(DMA_HandleTypeDef *hdma) {
    dcmi_ints[8] += 1;
}

static void DCMI_DMA_CpltCallback(DMA_HandleTypeDef *hdma) {
    //SCB_InvalidateDCache();
    dcmi_ints[9] += 1;
}

#if 0
static int dma_config() {
#if 0
    __DMA2_CLK_ENABLE();

    // DMA Stream configuration
    DMAHandle.Instance              = DMA2_Stream1;             /* Select the DMA instance          */
    DMAHandle.Init.Channel          = DMA_CHANNEL_1;            /* DMA Channel                      */
    DMAHandle.Init.Direction        = DMA_PERIPH_TO_MEMORY;     /* Peripheral to memory transfer    */
    DMAHandle.Init.MemInc           = DMA_MINC_ENABLE;          /* Memory increment mode Enable     */
    DMAHandle.Init.PeriphInc        = DMA_PINC_DISABLE;         /* Peripheral increment mode Enable */
    DMAHandle.Init.PeriphDataAlignment  = DMA_PDATAALIGN_WORD;  /* Peripheral data alignment : Word */
    DMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;      /* Memory data alignment : Word     */
#if 0
    DMAHandle.Init.Mode             = DMA_NORMAL;               /* Normal DMA mode                  */
#else
    DMAHandle.Init.Mode             = 0x20;                     /* peri. DMA mode                  */
#endif
    DMAHandle.Init.Priority         = DMA_PRIORITY_HIGH;        /* Priority level : high            */
    DMAHandle.Init.FIFOMode         = DMA_FIFOMODE_ENABLE;      /* FIFO mode enabled                */
    DMAHandle.Init.FIFOThreshold    = DMA_FIFO_THRESHOLD_FULL;  /* FIFO threshold full              */
    DMAHandle.Init.MemBurst         = DMA_MBURST_INC4;          /* Memory burst                     */
    DMAHandle.Init.PeriphBurst      = DMA_PBURST_SINGLE;        /* Peripheral burst                 */

    // Configure and disable DMA IRQ Channel
    //HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, IRQ_PRI_DMA21, IRQ_SUBPRI_DMA21);
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);

    // Initialize the DMA stream
    HAL_DMA_DeInit(&DMAHandle);
    if (HAL_DMA_Init(&DMAHandle) != HAL_OK) {
        // Initialization Error
        return -1;
    }

    return 0;
#else
    dma_init(&DMAHandle, &dma_DCMI_RX, (void*)NULL);
    //DCMI_RX_DMA_STREAM->CR |= 0x120;
    DMAHandle.XferErrorCallback = DCMI_DMA_ErrorCallback;
    DMAHandle.XferHalfCpltCallback = DCMI_DMA_HalfCpltCallback;
    DMAHandle.XferM1CpltCallback = DCMI_DMA_M1CpltCallback;
    DMAHandle.XferCpltCallback = DCMI_DMA_CpltCallback;
    printf("%s %p\n", __func__, DMAHandle.XferErrorCallback);
    printf("%s %p\n", __func__, DMAHandle.XferHalfCpltCallback);
    printf("%s %p\n", __func__, DMAHandle.XferM1CpltCallback);
    printf("%s %p\n", __func__, DMAHandle.XferCpltCallback);
    return 0;
#endif
}
#endif

void dcmi_init0(void) {
    memset(&DCMIHandle, 0, sizeof(DCMI_HandleTypeDef));
    DCMIHandle.Instance = DCMI;
}

/******************************************************************************/
// Micro Python bindings

typedef enum {
    DCMI_STATE_RESET,
    DCMI_STATE_WRITE_SINGLE,
    DCMI_STATE_BUILTIN_WAVEFORM,
    DCMI_STATE_DMA_WAVEFORM, // should be last enum since we use space beyond it
} pyb_dcmi_state_t;
#if 0
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
} pyb_dcmi_obj_t;
#endif
//void dcmi_init(DCMI_HandleTypeDef *dcmi)
void dcmi_init(const pyb_dcmi_obj_t *self) {
    DCMI_HandleTypeDef *dcmi = (DCMI_HandleTypeDef *)self->dcmi;
    printf("%s %p\n", __func__, dcmi);

    /* DCMI clock enable */
    __DCMI_CLK_ENABLE();

    /* DCMI GPIOs configuration */
    GPIO_InitTypeDef  GPIO_InitStructure;
    //GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
    GPIO_InitStructure.Pull      = GPIO_NOPULL;
    //GPIO_InitStructure.Speed     = GPIO_SPEED_LOW;
    GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Mode      = GPIO_MODE_INPUT;

    GPIO_InitStructure.Pin = pyb_pin_DCMI_EXPO.pin_mask;
    HAL_GPIO_Init(pyb_pin_DCMI_EXPO.gpio, &GPIO_InitStructure);

    GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStructure.Pin = pyb_pin_DCMI_OE.pin_mask;
    HAL_GPIO_Init(pyb_pin_DCMI_OE.gpio, &GPIO_InitStructure);
#if 0
    GPIO_InitStructure.Pin = pyb_pin_DCMI_RESET.pin_mask;
    HAL_GPIO_Init(pyb_pin_DCMI_RESET.gpio, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = pyb_pin_EN_VCAM.pin_mask;
    HAL_GPIO_Init(pyb_pin_EN_VCAM.gpio, &GPIO_InitStructure);
#endif
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Alternate = GPIO_AF13_DCMI;

    for (int i = 0; i < NUM_DCMI_PINS; i++) {
        printf("%s %p %x\n", __func__, dcmi_pins[i]->gpio, dcmi_pins[i]->pin);
        GPIO_InitStructure.Pin = dcmi_pins[i]->pin_mask;
        HAL_GPIO_Init(dcmi_pins[i]->gpio, &GPIO_InitStructure);
    }

    GPIO_InitStructure.Alternate = GPIO_AF0_MCO;
    GPIO_InitStructure.Pin = pyb_pin_DCMI_SYSCLK.pin_mask;
    HAL_GPIO_Init(pyb_pin_DCMI_SYSCLK.gpio, &GPIO_InitStructure);

    DCMI->CR = 0x0004;
#if 0
    DCMI->CWSTRTR = 0x410051;
    DCMI->CWSIZER = 0x10001;
#else
    DCMI->CWSTRTR = 0x000000;
    //DCMI->CWSIZER = ((96-1) << 16) | ((128)-1);
    DCMI->CWSIZER = ((self->h-1) << 16) | ((self->w)-1);
#endif
    //DCMI->CR = 0x4006;
    //DCMI->CR = 0x4006;

    // RESET = 1
    // OE = 1
#if 0
    HAL_GPIO_WritePin(pyb_pin_DCMI_RESET.gpio, pyb_pin_DCMI_RESET.pin_mask, 1);
    HAL_GPIO_WritePin(pyb_pin_EN_VCAM.gpio, pyb_pin_EN_VCAM.pin_mask, 0);
#endif
    HAL_GPIO_WritePin(pyb_pin_DCMI_OE.gpio, pyb_pin_DCMI_OE.pin_mask, 1);

    DCMI->IER = 0x1f;   // 1d;

    uint16_t *p = (uint16_t *)0x20070000;
    for (int i = 0; i < 0x7800; i++) {
        p[i] = (uint16_t)0xe5e5;        //i;
    }

    // configur DMA
    //dma_config();
    //dma_init(&DMAHandle, &dma_DCMI_RX, dcmi);
    dma_init(&DMAHandle, &dma_DCMI_RX, dcmi);
    //DCMI_RX_DMA_STREAM->CR |= 0x120;
    dcmi->DMA_Handle = &DMAHandle;
    dcmi->DMA_Handle->XferErrorCallback = DCMI_DMA_ErrorCallback;
    dcmi->DMA_Handle->XferHalfCpltCallback = DCMI_DMA_HalfCpltCallback;
    dcmi->DMA_Handle->XferM1CpltCallback = DCMI_DMA_M1CpltCallback;
    dcmi->DMA_Handle->XferCpltCallback = DCMI_DMA_CpltCallback;
    printf("%s %p\n", __func__, DMAHandle.XferErrorCallback);
    printf("%s %p\n", __func__, DMAHandle.XferHalfCpltCallback);
    printf("%s %p\n", __func__, DMAHandle.XferM1CpltCallback);
    printf("%s %p\n", __func__, DMAHandle.XferCpltCallback);

    // Configure and enable DCMI IRQ Channel
    //HAL_NVIC_SetPriority(DCMI_IRQn, IRQ_PRI_DCMI, IRQ_SUBPRI_DCMI);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);

    {
        //DCMI->CR |= (1 << 1);                      // start new capture
        DCMI->CR |= (1 << 14);                      // start new capture
        //HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
        //HAL_DMA_Start_IT(&DMAHandle, 0x50050028, 0x20070000, 0x8000);
        //HAL_DMA_Start(&DMAHandle, 0x50050028, 0x20070000, 128*96/1);
        HAL_DMA_Start_IT(&DMAHandle, 0x50050028, 0x20070000, (self->w * self->h)/4);
        DCMI->CR |= 1;
    } 
#if 0
    memset(&DCMI_Handle, 0, sizeof DCMI_Handle);
    DCMI_Handle.Instance = DCMI;
    DCMI_Handle.State = HAL_DCMI_STATE_RESET;
    HAL_DCMI_Init(&DCMI_Handle);
#endif
}

STATIC pyb_dcmi_obj_t pyb_dcmi_obj[] = {
    {{&pyb_dcmi_type}, &DCMIHandle, &dma_DCMI_RX, 128, 96, 0, 0, 0},
    };

STATIC mp_obj_t pyb_dcmi_init_helper(const pyb_dcmi_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_w, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_h, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_xoff, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_yoff, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
    };
    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t w = (args[0].u_int)?args[0].u_int:self->w;
    uint32_t h = (args[1].u_int)?args[1].u_int:self->h;
    uint32_t xoff = (args[1].u_int)?args[2].u_int:self->xoff;
    uint32_t yoff = (args[1].u_int)?args[3].u_int:self->yoff;

    if (((w+xoff) <= 320) && ((h+yoff) <= 240) && (w*h <= 0x10000)) {
        pyb_dcmi_obj_t *self = &pyb_dcmi_obj[0];
        self->w = w;
        self->h = h;
        self->xoff = xoff;
        self->yoff = yoff;
        self->needconf |= 1;
    }
    printf("%s %p %d %p %p %lu,%lu %lu,%lu 0x%lx\n", __func__, self, n_args, pos_args, kw_args, w, h, xoff, yoff, w*h);

#if 0
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bits, MP_ARG_INT, {.u_int = 8} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // GPIO configuration
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = self->pin;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // DCMI peripheral clock
    #if defined(MCU_SERIES_F4) || defined(MCU_SERIES_F7)
    __DCMI_CLK_ENABLE();
    #elif defined(MCU_SERIES_L4)
    __HAL_RCC_DCMI1_CLK_ENABLE();
    #else
    #error Unsupported Processor
    #endif

    // stop anything already going on
    __DMA2_CLK_ENABLE();
    DMA_HandleTypeDef DMA_Handle;
    /* Get currently configured dma */
    dma_init_handle(&DMA_Handle, self->rx_dma_descr, (void*)NULL);
    // Need to deinit DMA first
    DMA_Handle.State = HAL_DMA_STATE_READY;
    HAL_DMA_DeInit(&DMA_Handle);

    HAL_DCMI_Stop(&DCMI_Handle);
    if (DCMI_Handle.DMA_Handle != NULL) {
       // HAL_DCMI_Stop_DMA(&DCMI_Handle, self->DCMI_channel);
    }

    // reset state of DCMI
    self->state = DCMI_STATE_RESET;
#else
#endif
    //dcmi_init(self->dcmi);
    dcmi_init(self);
    return mp_const_none;
}

// create the DCMI object
// currently support either DCMI1 on X5 (id = 1) or DCMI2 on X6 (id = 2)

/// \classmethod \constructor(port)
/// Construct a new DCMI object.
///
/// `port` can be a pin object, or an integer (1 or 2).
/// DCMI(1) is on pin X5 and DCMI(2) is on pin X6.
STATIC mp_obj_t pyb_dcmi_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    printf("%s %d %d %p\n", __func__, n_args, n_kw, args);

#if 0
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    // get pin/channel to output on
    mp_int_t DCMI_id;
    if (MP_OBJ_IS_INT(args[0])) {
        DCMI_id = mp_obj_get_int(args[0]);
    } else {
        const pin_obj_t *pin = pin_find(args[0]);
        if (pin == &pin_A4) {
            DCMI_id = 1;
        } else if (pin == &pin_A5) {
            DCMI_id = 2;
        } else {
            nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "pin %q does not have DCMI capabilities", pin->name));
        }
    }

    pyb_dcmi_obj_t *dcmi = m_new_obj(pyb_dcmi_obj_t);
    DCMI->base.type = &pyb_dcmi_type;

    if (DCMI_id == 1) {
        DCMI->pin = GPIO_PIN_4;
        DCMI->DCMI_channel = DCMI_CHANNEL_1;
        DCMI->rx_dma_descr = &dma_DCMI_1_TX;
    } else if (DCMI_id == 2) {
        DCMI->pin = GPIO_PIN_5;
        DCMI->DCMI_channel = DCMI_CHANNEL_2;
        DCMI->rx_dma_descr = &dma_DCMI_2_TX;
    } else {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "DCMI %d does not exist", DCMI_id));
    }

    // configure the peripheral
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    pyb_dcmi_init_helper(DCMI, n_args - 1, args + 1, &kw_args);
#endif
//dcmi_init();
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    int dcmi_id = 0;
    // get DCMI object
    const pyb_dcmi_obj_t *dcmi_obj = &pyb_dcmi_obj[dcmi_id];
    if (n_args > 1 || n_kw > 0) {
        // configure the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        pyb_dcmi_init_helper(dcmi_obj, n_args - 1, args + 1, &kw_args);
    }
    
    // return object
    return (mp_obj_t)dcmi_obj;
}

STATIC mp_obj_t pyb_dcmi_init(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    printf("%s %d %p %p\n", __func__, n_args, args, kw_args);
    return pyb_dcmi_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_dcmi_init_obj, 1, pyb_dcmi_init);

/// \method deinit()
/// Turn off the DCMI, enable other use of pin.
STATIC mp_obj_t pyb_dcmi_deinit(mp_obj_t self_in) {
    printf("%s %p\n", __func__, self_in);
#if 0
    pyb_dcmi_obj_t *self = self_in;
    if (self->DCMI_channel == DCMI_CHANNEL_1) {
        DCMI_Handle.Instance->CR &= ~DCMI_CR_EN1;
        DCMI_Handle.Instance->CR |= DCMI_CR_BOFF1;
    } else {
        DCMI_Handle.Instance->CR &= ~DCMI_CR_EN2;
        DCMI_Handle.Instance->CR |= DCMI_CR_BOFF2;
    }
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_dcmi_deinit_obj, pyb_dcmi_deinit);

/// \method write(value)
/// Direct access to the DCMI output (8 bit only at the moment).
STATIC mp_obj_t pyb_dcmi_write(mp_obj_t self_in, mp_obj_t val) {
    printf("%s %p %p\n", __func__, self_in, val);
#if 0
    pyb_dcmi_obj_t *self = self_in;

    if (self->state != DCMI_STATE_WRITE_SINGLE) {
        DCMI_ChannelConfTypeDef config;
        config.DCMI_Trigger = DCMI_TRIGGER_NONE;
        config.DCMI_OutputBuffer = DCMI_OUTPUTBUFFER_DISABLE;
        HAL_DCMI_ConfigChannel(&DCMI_Handle, &config, self->DCMI_channel);
        self->state = DCMI_STATE_WRITE_SINGLE;
    }

    // DCMI output is always 12-bit at the hardware level, and we provide support
    // for multiple bit "resolutions" simply by shifting the input value.
    HAL_DCMI_SetValue(&DCMI_Handle, self->DCMI_channel, DCMI_ALIGN_12B_R,
        mp_obj_get_int(val) << (12 - self->bits));

    HAL_DCMI_Start(&DCMI_Handle, self->DCMI_channel);
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_dcmi_write_obj, pyb_dcmi_write);

/// \method read(ba, addr)
/// Direct access to frame buffer
STATIC mp_obj_t pyb_dcmi_read(mp_obj_t self_in, mp_obj_t ba, mp_obj_t addr) {
    uint32_t lf = dcmi_ints[9];
    pyb_dcmi_obj_t *self = self_in;
    volatile uint8_t *fb = (volatile uint8_t *)(0x20070000 + (mp_obj_get_int(addr) >> 16));
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(ba, &bufinfo, MP_BUFFER_WRITE);
    //SCB_InvalidateDCache();
#if 0
    printf("%s %p %p 0x%x %02x\n", __func__, self_in, ba, mp_obj_get_int(addr), *fb);
        self->w = w;
        self->h = h;
        self->xoff = xoff;
        self->yoff = yoff;
        self->needconf |= 1;
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);
    mp_uint_t ret = storage_read_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / FLASH_BLOCK_SIZE);
    return MP_OBJ_NEW_SMALL_INT(ret);
#endif
    // subsample & convert to c565
    int n = bufinfo.len;
    int ad = mp_obj_get_int(addr);
    int yinc = ((ad >> 4) & 0xf)+1;
    int xinc = (ad & 0xf)+1;
    if (ad & 0x2000) {
        uint32_t start = HAL_GetTick();

        // wait for frame...
        while (lf == dcmi_ints[9]) {
            if (HAL_GetTick() - start >= 40) {
                break;
            }
        }
    }
    if (ad & 0x8000) {
        uint8_t *p8 = (uint8_t *)bufinfo.buf;
        
        for (int y = 0; y < self->h; y += yinc) {
            for (int x = 0; x < self->w; x += xinc) {
                *p8++ = fb[y*self->w + x];
                n -= 1;
                if (n <= 0) {
                    break;
                }
            }
        }
    } else {
        uint16_t *p16 = (uint16_t *)bufinfo.buf;
        //printf("%s %p %d %d %d\n", __func__, p16, n, xinc, yinc);
        for (int y = 0; y < self->h; y += yinc) {
            for (int x = 0; x < self->w; x += xinc) {
                uint8_t pix = fb[y*self->w + x];
                *p16++ = ((pix >> 3) << 11) | ((pix >> 2) << 5) | (pix >> 3);
                n -= 2;
                if (n <= 0) {
                    break;
                }
            }
        }
    }
    if (ad & 0x4000) {
        uint32_t *p32 = (uint32_t *)bufinfo.buf;
        p32[0] = dcmi_ints[(ad >> 8) & 0xf];
    }
#if 0
    pyb_dcmi_obj_t *self = self_in;

    if (self->state != DCMI_STATE_WRITE_SINGLE) {
        DCMI_ChannelConfTypeDef config;
        config.DCMI_Trigger = DCMI_TRIGGER_NONE;
        config.DCMI_OutputBuffer = DCMI_OUTPUTBUFFER_DISABLE;
        HAL_DCMI_ConfigChannel(&DCMI_Handle, &config, self->DCMI_channel);
        self->state = DCMI_STATE_WRITE_SINGLE;
    }

    // DCMI output is always 12-bit at the hardware level, and we provide support
    // for multiple bit "resolutions" simply by shifting the input value.
    HAL_DCMI_SetValue(&DCMI_Handle, self->DCMI_channel, DCMI_ALIGN_12B_R,
        mp_obj_get_int(val) << (12 - self->bits));

    HAL_DCMI_Start(&DCMI_Handle, self->DCMI_channel);
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_dcmi_read_obj, pyb_dcmi_read);

/// \method window(ba, w, h, xoff, yoff)
/// define DMA window
STATIC mp_obj_t pyb_dcmi_window(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode, MP_ARG_INT, {.u_int = 0 } },
    };
    // parse args
    //pyb_dcmi_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t mode = args[0].u_int;

    switch (mode) {
    case 0:
    default:
            {
            mp_obj_t tuple[] = {
                                 mp_obj_new_int(dcmi_ints[0]),
                                 mp_obj_new_int(dcmi_ints[1]),
                                 mp_obj_new_int(dcmi_ints[2]),
                                 mp_obj_new_int(dcmi_ints[3]),
                                 mp_obj_new_int(dcmi_ints[4]),
                                 mp_obj_new_int(dcmi_ints[5]),
                                 mp_obj_new_int(dcmi_ints[6]),
                                 mp_obj_new_int(dcmi_ints[7]),
                                 mp_obj_new_int(dcmi_ints[8]),
                                 mp_obj_new_int(dcmi_ints[8])
                                };
            return mp_obj_new_tuple(MP_ARRAY_SIZE(tuple), tuple);
            }
            break;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_dcmi_window_obj, 0, pyb_dcmi_window);

/// \method info()
/// get status info
STATIC mp_obj_t pyb_dcmi_info(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_w, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_h, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_xoff, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_yoff, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0 } },
    };
    // parse args
    pyb_dcmi_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t w = (args[0].u_int)?args[0].u_int:self->w;
    uint32_t h = (args[1].u_int)?args[1].u_int:self->h;
    uint32_t xoff = (args[1].u_int)?args[2].u_int:self->xoff;
    uint32_t yoff = (args[1].u_int)?args[3].u_int:self->yoff;

    if (((w+xoff) <= 320) && ((h+yoff) <= 240) && (w*h <= 0x10000)) {
        self->w = w;
        self->h = h;
        self->xoff = xoff;
        self->yoff = yoff;
        self->needconf |= 1;
    }

    printf("%s %lu,%lu %lu,%lu 0x%lx\n", __func__, w, h, xoff, yoff, w*h);
#if 0
    pyb_dcmi_obj_t *self = self_in;

    if (self->state != DCMI_STATE_WRITE_SINGLE) {
        DCMI_ChannelConfTypeDef config;
        config.DCMI_Trigger = DCMI_TRIGGER_NONE;
        config.DCMI_OutputBuffer = DCMI_OUTPUTBUFFER_DISABLE;
        HAL_DCMI_ConfigChannel(&DCMI_Handle, &config, self->DCMI_channel);
        self->state = DCMI_STATE_WRITE_SINGLE;
    }

    // DCMI output is always 12-bit at the hardware level, and we provide support
    // for multiple bit "resolutions" simply by shifting the input value.
    HAL_DCMI_SetValue(&DCMI_Handle, self->DCMI_channel, DCMI_ALIGN_12B_R,
        mp_obj_get_int(val) << (12 - self->bits));

    HAL_DCMI_Start(&DCMI_Handle, self->DCMI_channel);
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_dcmi_info_obj, 0, pyb_dcmi_info);

STATIC const mp_map_elem_t pyb_dcmi_locals_dict_table[] = {
    // instance methods
#if 0
    { MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&pyb_dcmi_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_deinit), (mp_obj_t)&pyb_dcmi_deinit_obj },
#endif
#if 1
    { MP_OBJ_NEW_QSTR(MP_QSTR_write), (mp_obj_t)&pyb_dcmi_write_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read), (mp_obj_t)&pyb_dcmi_read_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_window), (mp_obj_t)&pyb_dcmi_window_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_info), (mp_obj_t)&pyb_dcmi_info_obj },
#endif
#if 0
    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_NORMAL),      MP_OBJ_NEW_SMALL_INT(DMA_NORMAL) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_CIRCULAR),    MP_OBJ_NEW_SMALL_INT(DMA_CIRCULAR) },
#endif
};

STATIC MP_DEFINE_CONST_DICT(pyb_dcmi_locals_dict, pyb_dcmi_locals_dict_table);

const mp_obj_type_t pyb_dcmi_type = {
    { &mp_type_type },
    .name = MP_QSTR_DCMI,
    .make_new = pyb_dcmi_make_new,
    .locals_dict = (mp_obj_t)&pyb_dcmi_locals_dict,
};

//uint32_t start_capture = 1;
void restart_dma(void) {
    pyb_dcmi_obj_t *self = &pyb_dcmi_obj[0];
    if (self->needconf & 1) {
        DMA_Cmd( DCMI_RX_DMA_STREAM, DISABLE);
        self->needconf &= ~1;
        DCMI->CR = 0x0004;
        DCMI->CWSTRTR = (self->yoff << 16) | self->xoff;
        DCMI->CWSIZER = ((self->h-1) << 16) | ((self->w)-1);
        DCMI->CR |= (1 << 14);
        HAL_DMA_Start_IT(&DMAHandle, 0x50050028, 0x20070000, (self->w * self->h)/4);
        DCMI->CR |= 1;
    } else {
        HAL_DMA_Start_IT(&DMAHandle, 0x50050028, 0x20070000, (self->w * self->h)/4);
        DCMI->CR |= 1;
    }
}

void DCMI_IRQHandler(void) {
    uint32_t isr_value = DCMI->MISR;        // READ_REG(hdcmi->Instance->MISR);
    DCMI->ICR = isr_value;
    if (isr_value & 1) {
      //DMA_Cmd( DCMI_RX_DMA_STREAM, DISABLE);

        dcmi_ints[0]++;
    #if 0
        HAL_DMA_Start_IT(&DMAHandle, 0x50050028, 0x20070000, 128*96/4);
        DCMI->CR |= 1;
    #endif
        restart_dma();
    }
    if (isr_value & 2) {
        // overflow...
        dcmi_ints[1]++;
        //start_capture = 1;
    #if 0
        HAL_DMA_Start_IT(&DMAHandle, 0x50050028, 0x20070000, 128*96/4);
        DCMI->CR |= 1;
    #endif
        restart_dma();
    }
    if (isr_value & 4) {
        dcmi_ints[2]++;
    }
    if (isr_value & 8) {
        dcmi_ints[3]++;
#if 0
        if (start_capture) {
            start_capture = 0;
        }
#endif
    }
    if (isr_value & 0x10) {
        dcmi_ints[4]++;
    }
    //DCMI->ICR = isr_value;
}

#endif // MICROPY_HW_ENABLE_DCMI
