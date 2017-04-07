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

#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/machine_spi.h"
#include "irq.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "bufhelper.h"
#include "dma.h"
#include "spi.h"

/// \moduleref pyb
/// \class SPI - a master-driven serial protocol
///
/// SPI is a serial protocol that is driven by a master.  At the physical level
/// there are 3 lines: SCK, MOSI, MISO.
///
/// See usage model of I2C; SPI is very similar.  Main difference is
/// parameters to init the SPI bus:
///
///     from pyb import SPI
///     spi = SPI(1, SPI.MASTER, baudrate=600000, polarity=1, phase=0, crc=0x7)
///
/// Only required parameter is mode, SPI.MASTER or SPI.SLAVE.  Polarity can be
/// 0 or 1, and is the level the idle clock line sits at.  Phase can be 0 or 1
/// to sample data on the first or second clock edge respectively.  Crc can be
/// None for no CRC, or a polynomial specifier.
///
/// Additional method for SPI:
///
///     data = spi.send_recv(b'1234')        # send 4 bytes and receive 4 bytes
///     buf = bytearray(4)
///     spi.send_recv(b'1234', buf)          # send 4 bytes and receive 4 into buf
///     spi.send_recv(buf, buf)              # send/recv 4 bytes from/to buf

// Possible DMA configurations for SPI busses:
// SPI1_TX: DMA2_Stream3.CHANNEL_3 or DMA2_Stream5.CHANNEL_3
// SPI1_RX: DMA2_Stream0.CHANNEL_3 or DMA2_Stream2.CHANNEL_3
// SPI2_TX: DMA1_Stream4.CHANNEL_0
// SPI2_RX: DMA1_Stream3.CHANNEL_0
// SPI3_TX: DMA1_Stream5.CHANNEL_0 or DMA1_Stream7.CHANNEL_0
// SPI3_RX: DMA1_Stream0.CHANNEL_0 or DMA1_Stream2.CHANNEL_0
// SPI4_TX: DMA2_Stream4.CHANNEL_5 or DMA2_Stream1.CHANNEL_4
// SPI4_RX: DMA2_Stream3.CHANNEL_5 or DMA2_Stream0.CHANNEL_4
// SPI5_TX: DMA2_Stream4.CHANNEL_2 or DMA2_Stream6.CHANNEL_7
// SPI5_RX: DMA2_Stream3.CHANNEL_2 or DMA2_Stream5.CHANNEL_7
// SPI6_TX: DMA2_Stream5.CHANNEL_1
// SPI6_RX: DMA2_Stream6.CHANNEL_1

typedef struct _pyb_spi_obj_t {
    mp_obj_base_t base;
    SPI_HandleTypeDef *spi;
    const dma_descr_t *tx_dma_descr;
    const dma_descr_t *rx_dma_descr;
} pyb_spi_obj_t;

#if defined(MICROPY_HW_SPI1_SCK)
SPI_HandleTypeDef SPIHandle1 = {.Instance = NULL};
#endif
#if defined(MICROPY_HW_SPI2_SCK)
SPI_HandleTypeDef SPIHandle2 = {.Instance = NULL};
#endif
#if defined(MICROPY_HW_SPI3_SCK)
SPI_HandleTypeDef SPIHandle3 = {.Instance = NULL};
#endif
#if defined(MICROPY_HW_SPI4_SCK)
SPI_HandleTypeDef SPIHandle4 = {.Instance = NULL};
#endif
#if defined(MICROPY_HW_SPI5_SCK)
SPI_HandleTypeDef SPIHandle5 = {.Instance = NULL};
#endif
#if defined(MICROPY_HW_SPI6_SCK)
SPI_HandleTypeDef SPIHandle6 = {.Instance = NULL};
#endif

STATIC const pyb_spi_obj_t pyb_spi_obj[] = {
    #if defined(MICROPY_HW_SPI1_SCK)
    {{&pyb_spi_type}, &SPIHandle1, &dma_SPI_1_TX, &dma_SPI_1_RX},
    #else
    {{&pyb_spi_type}, NULL, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_SPI2_SCK)
    {{&pyb_spi_type}, &SPIHandle2, &dma_SPI_2_TX, &dma_SPI_2_RX},
    #else
    {{&pyb_spi_type}, NULL, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_SPI3_SCK)
    {{&pyb_spi_type}, &SPIHandle3, &dma_SPI_3_TX, &dma_SPI_3_RX},
    #else
    {{&pyb_spi_type}, NULL, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_SPI4_SCK)
    {{&pyb_spi_type}, &SPIHandle4, &dma_SPI_4_TX, &dma_SPI_4_RX},
    #else
    {{&pyb_spi_type}, NULL, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_SPI5_SCK)
    {{&pyb_spi_type}, &SPIHandle5, &dma_SPI_5_TX, &dma_SPI_5_RX},
    #else
    {{&pyb_spi_type}, NULL, NULL, NULL},
    #endif
    #if defined(MICROPY_HW_SPI6_SCK)
    {{&pyb_spi_type}, &SPIHandle6, &dma_SPI_6_TX, &dma_SPI_6_RX},
    #else
    {{&pyb_spi_type}, NULL, NULL, NULL},
    #endif
};

void spi_init0(void) {
    // reset the SPI handles
    #if defined(MICROPY_HW_SPI1_SCK)
    memset(&SPIHandle1, 0, sizeof(SPI_HandleTypeDef));
    SPIHandle1.Instance = SPI1;
    #endif
    #if defined(MICROPY_HW_SPI2_SCK)
    memset(&SPIHandle2, 0, sizeof(SPI_HandleTypeDef));
    SPIHandle2.Instance = SPI2;
    #endif
    #if defined(MICROPY_HW_SPI3_SCK)
    memset(&SPIHandle3, 0, sizeof(SPI_HandleTypeDef));
    SPIHandle3.Instance = SPI3;
    #endif
    #if defined(MICROPY_HW_SPI4_SCK)
    memset(&SPIHandle4, 0, sizeof(SPI_HandleTypeDef));
    SPIHandle4.Instance = SPI4;
    #endif
    #if defined(MICROPY_HW_SPI5_SCK)
    memset(&SPIHandle5, 0, sizeof(SPI_HandleTypeDef));
    SPIHandle5.Instance = SPI5;
    #endif
    #if defined(MICROPY_HW_SPI6_SCK)
    memset(&SPIHandle6, 0, sizeof(SPI_HandleTypeDef));
    SPIHandle6.Instance = SPI6;
    #endif
}

STATIC int spi_find(mp_obj_t id) {
    if (MP_OBJ_IS_STR(id)) {
        // given a string id
        const char *port = mp_obj_str_get_str(id);
        if (0) {
        #ifdef MICROPY_HW_SPI1_NAME
        } else if (strcmp(port, MICROPY_HW_SPI1_NAME) == 0) {
            return 1;
        #endif
        #ifdef MICROPY_HW_SPI2_NAME
        } else if (strcmp(port, MICROPY_HW_SPI2_NAME) == 0) {
            return 2;
        #endif
        #ifdef MICROPY_HW_SPI3_NAME
        } else if (strcmp(port, MICROPY_HW_SPI3_NAME) == 0) {
            return 3;
        #endif
        #ifdef MICROPY_HW_SPI4_NAME
        } else if (strcmp(port, MICROPY_HW_SPI4_NAME) == 0) {
            return 4;
        #endif
        #ifdef MICROPY_HW_SPI5_NAME
        } else if (strcmp(port, MICROPY_HW_SPI5_NAME) == 0) {
            return 5;
        #endif
        #ifdef MICROPY_HW_SPI6_NAME
        } else if (strcmp(port, MICROPY_HW_SPI6_NAME) == 0) {
            return 6;
        #endif
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            "SPI(%s) does not exist", port));
    } else {
        // given an integer id
        int spi_id = mp_obj_get_int(id);
        if (spi_id >= 1 && spi_id <= MP_ARRAY_SIZE(pyb_spi_obj)
            && pyb_spi_obj[spi_id - 1].spi != NULL) {
            return spi_id;
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            "SPI(%d) does not exist", spi_id));
    }
}

// sets the parameters in the SPI_InitTypeDef struct
// if an argument is -1 then the corresponding parameter is not changed
#if 0       // DPG_20170305
STATIC
#endif
void spi_set_params(SPI_HandleTypeDef *spi, uint32_t prescale, int32_t baudrate,
    int32_t polarity, int32_t phase, int32_t bits, int32_t firstbit) {
    SPI_InitTypeDef *init = &spi->Init;

    if (prescale != 0xffffffff || baudrate != -1) {
        if (prescale == 0xffffffff) {
            // prescaler not given, so select one that yields at most the requested baudrate
            mp_uint_t spi_clock;
            if (spi->Instance == SPI2 || spi->Instance == SPI3) {
                // SPI2 and SPI3 are on APB1
                spi_clock = HAL_RCC_GetPCLK1Freq();
            } else {
                // SPI1, SPI4, SPI5 and SPI6 are on APB2
                spi_clock = HAL_RCC_GetPCLK2Freq();
            }
            prescale = spi_clock / baudrate;
        }
        if (prescale <= 2) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; }
        else if (prescale <= 4) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; }
        else if (prescale <= 8) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; }
        else if (prescale <= 16) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; }
        else if (prescale <= 32) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; }
        else if (prescale <= 64) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; }
        else if (prescale <= 128) { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; }
        else { init->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; }
    }

    if (polarity != -1) {
        init->CLKPolarity = polarity == 0 ? SPI_POLARITY_LOW : SPI_POLARITY_HIGH;
    }

    if (phase != -1) {
        init->CLKPhase = phase == 0 ? SPI_PHASE_1EDGE : SPI_PHASE_2EDGE;
    }

    if (bits != -1) {
        init->DataSize = (bits == 16) ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
    }

    if (firstbit != -1) {
        init->FirstBit = firstbit;
    }
}

// TODO allow to take a list of pins to use
void spi_init(SPI_HandleTypeDef *spi, bool enable_nss_pin) {
    const pyb_spi_obj_t *self;
    const pin_obj_t *pins[4];
    pins[0] = NULL;

    if (0) {
    #if defined(MICROPY_HW_SPI1_SCK)
    } else if (spi->Instance == SPI1) {
        self = &pyb_spi_obj[0];
        #if defined(MICROPY_HW_SPI1_NSS)
        pins[0] = &MICROPY_HW_SPI1_NSS;
        #endif
        pins[1] = &MICROPY_HW_SPI1_SCK;
        pins[2] = &MICROPY_HW_SPI1_MISO;
        pins[3] = &MICROPY_HW_SPI1_MOSI;
        // enable the SPI clock
        __SPI1_CLK_ENABLE();
    #endif
    #if defined(MICROPY_HW_SPI2_SCK)
    } else if (spi->Instance == SPI2) {
        self = &pyb_spi_obj[1];
        #if defined(MICROPY_HW_SPI2_NSS)
        pins[0] = &MICROPY_HW_SPI2_NSS;
        #endif
        pins[1] = &MICROPY_HW_SPI2_SCK;
        pins[2] = &MICROPY_HW_SPI2_MISO;
        pins[3] = &MICROPY_HW_SPI2_MOSI;
        // enable the SPI clock
        __SPI2_CLK_ENABLE();
    #endif
    #if defined(MICROPY_HW_SPI3_SCK)
    } else if (spi->Instance == SPI3) {
        self = &pyb_spi_obj[2];
        #if defined(MICROPY_HW_SPI3_NSS)
        pins[0] = &MICROPY_HW_SPI3_NSS;
        #endif
        pins[1] = &MICROPY_HW_SPI3_SCK;
        pins[2] = &MICROPY_HW_SPI3_MISO;
        pins[3] = &MICROPY_HW_SPI3_MOSI;
        // enable the SPI clock
        __SPI3_CLK_ENABLE();
    #endif
    #if defined(MICROPY_HW_SPI4_SCK)
    } else if (spi->Instance == SPI4) {
        self = &pyb_spi_obj[3];
        #if defined(MICROPY_HW_SPI4_NSS)
        pins[0] = &MICROPY_HW_SPI4_NSS;
        #endif
        pins[1] = &MICROPY_HW_SPI4_SCK;
        pins[2] = &MICROPY_HW_SPI4_MISO;
        pins[3] = &MICROPY_HW_SPI4_MOSI;
        // enable the SPI clock
        __SPI4_CLK_ENABLE();
    #endif
    #if defined(MICROPY_HW_SPI5_SCK)
    } else if (spi->Instance == SPI5) {
        self = &pyb_spi_obj[4];
        #if defined(MICROPY_HW_SPI5_NSS)
        pins[0] = &MICROPY_HW_SPI5_NSS;
        #endif
        pins[1] = &MICROPY_HW_SPI5_SCK;
        pins[2] = &MICROPY_HW_SPI5_MISO;
        pins[3] = &MICROPY_HW_SPI5_MOSI;
        // enable the SPI clock
        __SPI5_CLK_ENABLE();
    #endif
    #if defined(MICROPY_HW_SPI6_SCK)
    } else if (spi->Instance == SPI6) {
        self = &pyb_spi_obj[5];
        #if defined(MICROPY_HW_SPI6_NSS)
        pins[0] = &MICROPY_HW_SPI6_NSS;
        #endif
        pins[1] = &MICROPY_HW_SPI6_SCK;
        pins[2] = &MICROPY_HW_SPI6_MISO;
        pins[3] = &MICROPY_HW_SPI6_MOSI;
        // enable the SPI clock
        __SPI6_CLK_ENABLE();
    #endif
    } else {
        // SPI does not exist for this board (shouldn't get here, should be checked by caller)
        return;
    }

    // init the GPIO lines
    uint32_t mode = MP_HAL_PIN_MODE_ALT;
    uint32_t pull = spi->Init.CLKPolarity == SPI_POLARITY_LOW ? MP_HAL_PIN_PULL_DOWN : MP_HAL_PIN_PULL_UP;
    for (uint i = (enable_nss_pin && pins[0] ? 0 : 1); i < 4; i++) {
        mp_hal_pin_config_alt(pins[i], mode, pull, AF_FN_SPI, (self - &pyb_spi_obj[0]) + 1);
    }

    // init the SPI device
    if (HAL_SPI_Init(spi) != HAL_OK) {
        // init error
        // TODO should raise an exception, but this function is not necessarily going to be
        // called via Python, so may not be properly wrapped in an NLR handler
        printf("OSError: HAL_SPI_Init failed\n");
        return;
    }

    // After calling HAL_SPI_Init() it seems that the DMA gets disconnected if
    // it was previously configured.  So we invalidate the DMA channel to force
    // an initialisation the next time we use it.
    dma_invalidate_channel(self->tx_dma_descr);
    dma_invalidate_channel(self->rx_dma_descr);
}

void spi_deinit(SPI_HandleTypeDef *spi) {
    HAL_SPI_DeInit(spi);
    if (0) {
    #if defined(MICROPY_HW_SPI1_SCK)
    } else if (spi->Instance == SPI1) {
        __SPI1_FORCE_RESET();
        __SPI1_RELEASE_RESET();
        __SPI1_CLK_DISABLE();
    #endif
    #if defined(MICROPY_HW_SPI2_SCK)
    } else if (spi->Instance == SPI2) {
        __SPI2_FORCE_RESET();
        __SPI2_RELEASE_RESET();
        __SPI2_CLK_DISABLE();
    #endif
    #if defined(MICROPY_HW_SPI3_SCK)
    } else if (spi->Instance == SPI3) {
        __SPI3_FORCE_RESET();
        __SPI3_RELEASE_RESET();
        __SPI3_CLK_DISABLE();
    #endif
    #if defined(MICROPY_HW_SPI4_SCK)
    } else if (spi->Instance == SPI4) {
        __SPI4_FORCE_RESET();
        __SPI4_RELEASE_RESET();
        __SPI4_CLK_DISABLE();
    #endif
    #if defined(MICROPY_HW_SPI5_SCK)
    } else if (spi->Instance == SPI5) {
        __SPI5_FORCE_RESET();
        __SPI5_RELEASE_RESET();
        __SPI5_CLK_DISABLE();
    #endif
    #if defined(MICROPY_HW_SPI6_SCK)
    } else if (spi->Instance == SPI6) {
        __SPI6_FORCE_RESET();
        __SPI6_RELEASE_RESET();
        __SPI6_CLK_DISABLE();
    #endif
    }
}

STATIC HAL_StatusTypeDef spi_wait_dma_finished(SPI_HandleTypeDef *spi, uint32_t timeout) {
    // Note: we can't use WFI to idle in this loop because the DMA completion
    // interrupt may occur before the WFI.  Hence we miss it and have to wait
    // until the next sys-tick (up to 1ms).
    uint32_t start = HAL_GetTick();
    while (HAL_SPI_GetState(spi) != HAL_SPI_STATE_READY) {
        if (HAL_GetTick() - start >= timeout) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

// A transfer of "len" bytes should take len*8*1000/baudrate milliseconds.
// To simplify the calculation we assume the baudrate is never less than 8kHz
// and use that value for the baudrate in the formula, plus a small constant.
#define SPI_TRANSFER_TIMEOUT(len) ((len) + 100)

#if 1       // DPG_20170305
const pyb_spi_obj_t *spi_get_obj_from_handle(SPI_HandleTypeDef *spi) {
    for (size_t i = 0; i < MP_ARRAY_SIZE(pyb_spi_obj); ++i) {
        if (pyb_spi_obj[i].spi == spi) {
            return &pyb_spi_obj[i];
        }
    }
    return NULL;
}

#if USE_DUMMY_PAT
static uint32_t dummy_read[512];
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
#else
#if USE_DUMMY_PAT
uint32_t dummy_read[2048];
#endif

STATIC
#endif
void spi_transfer(const pyb_spi_obj_t *self, size_t len, const uint8_t *src, uint8_t *dest, uint32_t timeout) {
    // Note: there seems to be a problem sending 1 byte using DMA the first
    // time directly after the SPI/DMA is initialised.  The cause of this is
    // unknown but we sidestep the issue by using polling for 1 byte transfer.
#if USE_DUMMY_PAT
if (dest == NULL) {
    if (len <= sizeof(dummy_read)) {
        dest = (uint8_t *)dummy_read;
    }
}
if (src == NULL) {
    if (len <= sizeof(dummy_write)) {
        src = (uint8_t *)dummy_write;
    }
}
#endif

    HAL_StatusTypeDef status;

    if (dest == NULL) {
        // send only
        if (len == 1 || query_irq() == IRQ_STATE_DISABLED) {
            status = HAL_SPI_Transmit(self->spi, (uint8_t*)src, len, timeout);
        } else {
            DMA_HandleTypeDef tx_dma;
            dma_init(&tx_dma, self->tx_dma_descr, self->spi);
            self->spi->hdmatx = &tx_dma;
            self->spi->hdmarx = NULL;
            MP_HAL_CLEAN_DCACHE(src, len);
            status = HAL_SPI_Transmit_DMA(self->spi, (uint8_t*)src, len);
            if (status == HAL_OK) {
                status = spi_wait_dma_finished(self->spi, timeout);
            }
            dma_deinit(self->tx_dma_descr);
        }
    } else if (src == NULL) {
        // receive only
        if (len == 1 || query_irq() == IRQ_STATE_DISABLED) {
            status = HAL_SPI_Receive(self->spi, dest, len, timeout);
        } else {
            DMA_HandleTypeDef tx_dma, rx_dma;
            if (self->spi->Init.Mode == SPI_MODE_MASTER) {
                // in master mode the HAL actually does a TransmitReceive call
                dma_init(&tx_dma, self->tx_dma_descr, self->spi);
                self->spi->hdmatx = &tx_dma;
            } else {
                self->spi->hdmatx = NULL;
            }
            dma_init(&rx_dma, self->rx_dma_descr, self->spi);
            self->spi->hdmarx = &rx_dma;
            MP_HAL_CLEANINVALIDATE_DCACHE(dest, len);
            status = HAL_SPI_Receive_DMA(self->spi, dest, len);
            if (status == HAL_OK) {
                status = spi_wait_dma_finished(self->spi, timeout);
            }
            if (self->spi->hdmatx != NULL) {
                dma_deinit(self->tx_dma_descr);
            }
            dma_deinit(self->rx_dma_descr);
        }
    } else {
        // send and receive
        if (len == 1 || query_irq() == IRQ_STATE_DISABLED) {
            status = HAL_SPI_TransmitReceive(self->spi, (uint8_t*)src, dest, len, timeout);
        } else {
            DMA_HandleTypeDef tx_dma, rx_dma;
            dma_init(&tx_dma, self->tx_dma_descr, self->spi);
            self->spi->hdmatx = &tx_dma;
            dma_init(&rx_dma, self->rx_dma_descr, self->spi);
            self->spi->hdmarx = &rx_dma;
            MP_HAL_CLEAN_DCACHE(src, len);
            MP_HAL_CLEANINVALIDATE_DCACHE(dest, len);
            status = HAL_SPI_TransmitReceive_DMA(self->spi, (uint8_t*)src, dest, len);
            if (status == HAL_OK) {
                status = spi_wait_dma_finished(self->spi, timeout);
            }
            dma_deinit(self->tx_dma_descr);
            dma_deinit(self->rx_dma_descr);
        }
    }

    if (status != HAL_OK) {
        mp_hal_raise(status);
    }
}

//STATIC
void spi_print(const mp_print_t *print, SPI_HandleTypeDef *spi, bool legacy) {
    uint spi_num = 1; // default to SPI1
    if (spi->Instance == SPI2) { spi_num = 2; }
    else if (spi->Instance == SPI3) { spi_num = 3; }
    #if defined(SPI4)
    else if (spi->Instance == SPI4) { spi_num = 4; }
    #endif
    #if defined(SPI5)
    else if (spi->Instance == SPI5) { spi_num = 5; }
    #endif
    #if defined(SPI6)
    else if (spi->Instance == SPI6) { spi_num = 6; }
    #endif

    mp_printf(print, "SPI(%u", spi_num);
    if (spi->State != HAL_SPI_STATE_RESET) {
        if (spi->Init.Mode == SPI_MODE_MASTER) {
            // compute baudrate
            uint spi_clock;
            if (spi->Instance == SPI2 || spi->Instance == SPI3) {
                // SPI2 and SPI3 are on APB1
                spi_clock = HAL_RCC_GetPCLK1Freq();
            } else {
                // SPI1, SPI4, SPI5 and SPI6 are on APB2
                spi_clock = HAL_RCC_GetPCLK2Freq();
            }
            uint log_prescaler = (spi->Init.BaudRatePrescaler >> 3) + 1;
            uint baudrate = spi_clock >> log_prescaler;
            if (legacy) {
                mp_printf(print, ", SPI.MASTER");
            }
            mp_printf(print, ", baudrate=%u", baudrate);
            if (legacy) {
                mp_printf(print, ", prescaler=%u", 1 << log_prescaler);
            }
        } else {
            mp_printf(print, ", SPI.SLAVE");
        }
        mp_printf(print, ", polarity=%u, phase=%u, bits=%u", spi->Init.CLKPolarity == SPI_POLARITY_LOW ? 0 : 1, spi->Init.CLKPhase == SPI_PHASE_1EDGE ? 0 : 1, spi->Init.DataSize == SPI_DATASIZE_8BIT ? 8 : 16);
        if (spi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLED) {
            mp_printf(print, ", crc=0x%x", spi->Init.CRCPolynomial);
        }
    }
    mp_print_str(print, ")");
}

/******************************************************************************/
/* MicroPython bindings for legacy pyb API                                    */

SPI_HandleTypeDef *spi_get_handle(mp_obj_t o) {
    if (!MP_OBJ_IS_TYPE(o, &pyb_spi_type)) {
        mp_raise_ValueError("expecting an SPI object");
    }
    pyb_spi_obj_t *self = o;
    return self->spi;
}

STATIC void pyb_spi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_spi_obj_t *self = self_in;
    spi_print(print, self->spi, true);
}

/// \method init(mode, baudrate=328125, *, polarity=1, phase=0, bits=8, firstbit=SPI.MSB, ti=False, crc=None)
///
/// Initialise the SPI bus with the given parameters:
///
///   - `mode` must be either `SPI.MASTER` or `SPI.SLAVE`.
///   - `baudrate` is the SCK clock rate (only sensible for a master).
STATIC mp_obj_t pyb_spi_init_helper(const pyb_spi_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 328125} },
        { MP_QSTR_prescaler, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0xffffffff} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 1} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 0} },
        { MP_QSTR_dir,      MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SPI_DIRECTION_2LINES} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 8} },
        { MP_QSTR_nss,      MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SPI_NSS_SOFT} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = SPI_FIRSTBIT_MSB} },
        { MP_QSTR_ti,       MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_crc,      MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // set the SPI configuration values
    SPI_InitTypeDef *init = &self->spi->Init;
    init->Mode = args[0].u_int;

    spi_set_params(self->spi, args[2].u_int, args[1].u_int, args[3].u_int, args[4].u_int,
        args[6].u_int, args[8].u_int);

    init->Direction = args[5].u_int;
    init->NSS = args[7].u_int;
    init->TIMode = args[9].u_bool ? SPI_TIMODE_ENABLED : SPI_TIMODE_DISABLED;
    if (args[10].u_obj == mp_const_none) {
        init->CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        init->CRCPolynomial = 0;
    } else {
        init->CRCCalculation = SPI_CRCCALCULATION_ENABLED;
        init->CRCPolynomial = mp_obj_get_int(args[10].u_obj);
    }

    // init the SPI bus
    spi_init(self->spi, init->NSS != SPI_NSS_SOFT);

    return mp_const_none;
}

/// \classmethod \constructor(bus, ...)
///
/// Construct an SPI object on the given bus.  `bus` can be 1 or 2.
/// With no additional parameters, the SPI object is created but not
/// initialised (it has the settings from the last initialisation of
/// the bus, if any).  If extra arguments are given, the bus is initialised.
/// See `init` for parameters of initialisation.
///
/// The physical pins of the SPI busses are:
///
///   - `SPI(1)` is on the X position: `(NSS, SCK, MISO, MOSI) = (X5, X6, X7, X8) = (PA4, PA5, PA6, PA7)`
///   - `SPI(2)` is on the Y position: `(NSS, SCK, MISO, MOSI) = (Y5, Y6, Y7, Y8) = (PB12, PB13, PB14, PB15)`
///
/// At the moment, the NSS pin is not used by the SPI driver and is free
/// for other use.
STATIC mp_obj_t pyb_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // work out SPI bus
    int spi_id = spi_find(args[0]);

    // get SPI object
    const pyb_spi_obj_t *spi_obj = &pyb_spi_obj[spi_id - 1];

    if (n_args > 1 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        pyb_spi_init_helper(spi_obj, n_args - 1, args + 1, &kw_args);
    }

    return (mp_obj_t)spi_obj;
}

STATIC mp_obj_t pyb_spi_init(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return pyb_spi_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_spi_init_obj, 1, pyb_spi_init);

/// \method deinit()
/// Turn off the SPI bus.
STATIC mp_obj_t pyb_spi_deinit(mp_obj_t self_in) {
    pyb_spi_obj_t *self = self_in;
    spi_deinit(self->spi);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_spi_deinit_obj, pyb_spi_deinit);

/// \method send(send, *, timeout=5000)
/// Send data on the bus:
///
///   - `send` is the data to send (an integer to send, or a buffer object).
///   - `timeout` is the timeout in milliseconds to wait for the send.
///
/// Return value: `None`.
STATIC mp_obj_t pyb_spi_send(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // TODO assumes transmission size is 8-bits wide

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_send,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    pyb_spi_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get the buffer to send from
    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    pyb_buf_get_for_send(args[0].u_obj, &bufinfo, data);

    // send the data
    spi_transfer(self, bufinfo.len, bufinfo.buf, NULL, args[1].u_int);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_spi_send_obj, 1, pyb_spi_send);

/// \method recv(recv, *, timeout=5000)
///
/// Receive data on the bus:
///
///   - `recv` can be an integer, which is the number of bytes to receive,
///     or a mutable buffer, which will be filled with received bytes.
///   - `timeout` is the timeout in milliseconds to wait for the receive.
///
/// Return value: if `recv` is an integer then a new buffer of the bytes received,
/// otherwise the same buffer that was passed in to `recv`.
STATIC mp_obj_t pyb_spi_recv(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // TODO assumes transmission size is 8-bits wide

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_recv,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    pyb_spi_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get the buffer to receive into
    vstr_t vstr;
    mp_obj_t o_ret = pyb_buf_get_for_recv(args[0].u_obj, &vstr);

    // receive the data
    spi_transfer(self, vstr.len, NULL, (uint8_t*)vstr.buf, args[1].u_int);

    // return the received data
    if (o_ret != MP_OBJ_NULL) {
        return o_ret;
    } else {
        return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_spi_recv_obj, 1, pyb_spi_recv);

/// \method send_recv(send, recv=None, *, timeout=5000)
///
/// Send and receive data on the bus at the same time:
///
///   - `send` is the data to send (an integer to send, or a buffer object).
///   - `recv` is a mutable buffer which will be filled with received bytes.
///   It can be the same as `send`, or omitted.  If omitted, a new buffer will
///   be created.
///   - `timeout` is the timeout in milliseconds to wait for the receive.
///
/// Return value: the buffer with the received bytes.
STATIC mp_obj_t pyb_spi_send_recv(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // TODO assumes transmission size is 8-bits wide

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_send,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_recv,    MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    pyb_spi_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get buffers to send from/receive to
    mp_buffer_info_t bufinfo_send;
    uint8_t data_send[1];
    mp_buffer_info_t bufinfo_recv;
    vstr_t vstr_recv;
    mp_obj_t o_ret;

    if (args[0].u_obj == args[1].u_obj) {
        // same object for send and receive, it must be a r/w buffer
        mp_get_buffer_raise(args[0].u_obj, &bufinfo_send, MP_BUFFER_RW);
        bufinfo_recv = bufinfo_send;
        o_ret = args[0].u_obj;
    } else {
        // get the buffer to send from
        pyb_buf_get_for_send(args[0].u_obj, &bufinfo_send, data_send);

        // get the buffer to receive into
        if (args[1].u_obj == MP_OBJ_NULL) {
            // only send argument given, so create a fresh buffer of the send length
            vstr_init_len(&vstr_recv, bufinfo_send.len);
            bufinfo_recv.len = vstr_recv.len;
            bufinfo_recv.buf = vstr_recv.buf;
            o_ret = MP_OBJ_NULL;
        } else {
            // recv argument given
            mp_get_buffer_raise(args[1].u_obj, &bufinfo_recv, MP_BUFFER_WRITE);
            if (bufinfo_recv.len != bufinfo_send.len) {
                nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "recv must be same length as send"));
            }
            o_ret = args[1].u_obj;
        }
    }

    // do the transfer
    spi_transfer(self, bufinfo_send.len, bufinfo_send.buf, bufinfo_recv.buf, args[2].u_int);

    // return the received data
    if (o_ret != MP_OBJ_NULL) {
        return o_ret;
    } else {
        return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr_recv);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_spi_send_recv_obj, 1, pyb_spi_send_recv);

STATIC const mp_map_elem_t pyb_spi_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&pyb_spi_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_deinit), (mp_obj_t)&pyb_spi_deinit_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_read), (mp_obj_t)&mp_machine_spi_read_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_readinto), (mp_obj_t)&mp_machine_spi_readinto_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_write), (mp_obj_t)&mp_machine_spi_write_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_write_readinto), (mp_obj_t)&mp_machine_spi_write_readinto_obj },

    // legacy methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_send), (mp_obj_t)&pyb_spi_send_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv), (mp_obj_t)&pyb_spi_recv_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_send_recv), (mp_obj_t)&pyb_spi_send_recv_obj },

    // class constants
    /// \constant MASTER - for initialising the bus to master mode
    /// \constant SLAVE - for initialising the bus to slave mode
    /// \constant MSB - set the first bit to MSB
    /// \constant LSB - set the first bit to LSB
    { MP_OBJ_NEW_QSTR(MP_QSTR_MASTER), MP_OBJ_NEW_SMALL_INT(SPI_MODE_MASTER) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SLAVE),  MP_OBJ_NEW_SMALL_INT(SPI_MODE_SLAVE) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_MSB),    MP_OBJ_NEW_SMALL_INT(SPI_FIRSTBIT_MSB) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_LSB),    MP_OBJ_NEW_SMALL_INT(SPI_FIRSTBIT_LSB) },
    /* TODO
    { MP_OBJ_NEW_QSTR(MP_QSTR_DIRECTION_2LINES             ((uint32_t)0x00000000)
    { MP_OBJ_NEW_QSTR(MP_QSTR_DIRECTION_2LINES_RXONLY      SPI_CR1_RXONLY
    { MP_OBJ_NEW_QSTR(MP_QSTR_DIRECTION_1LINE              SPI_CR1_BIDIMODE
    { MP_OBJ_NEW_QSTR(MP_QSTR_NSS_SOFT                    SPI_CR1_SSM
    { MP_OBJ_NEW_QSTR(MP_QSTR_NSS_HARD_INPUT              ((uint32_t)0x00000000)
    { MP_OBJ_NEW_QSTR(MP_QSTR_NSS_HARD_OUTPUT             ((uint32_t)0x00040000)
    */
};

STATIC MP_DEFINE_CONST_DICT(pyb_spi_locals_dict, pyb_spi_locals_dict_table);

STATIC void spi_transfer_machine(mp_obj_base_t *self_in, size_t len, const uint8_t *src, uint8_t *dest) {
    spi_transfer((pyb_spi_obj_t*)self_in, len, src, dest, SPI_TRANSFER_TIMEOUT(len));
}

STATIC const mp_machine_spi_p_t pyb_spi_p = {
    .transfer = spi_transfer_machine,
};

const mp_obj_type_t pyb_spi_type = {
    { &mp_type_type },
    .name = MP_QSTR_SPI,
    .print = pyb_spi_print,
    .make_new = pyb_spi_make_new,
    .protocol = &pyb_spi_p,
    .locals_dict = (mp_obj_t)&pyb_spi_locals_dict,
};

/******************************************************************************/
// Implementation of hard SPI for machine module

typedef struct _machine_hard_spi_obj_t {
    mp_obj_base_t base;
    const pyb_spi_obj_t *pyb;
} machine_hard_spi_obj_t;

STATIC const machine_hard_spi_obj_t machine_hard_spi_obj[] = {
    {{&machine_hard_spi_type}, &pyb_spi_obj[0]},
    {{&machine_hard_spi_type}, &pyb_spi_obj[1]},
    {{&machine_hard_spi_type}, &pyb_spi_obj[2]},
    {{&machine_hard_spi_type}, &pyb_spi_obj[3]},
    {{&machine_hard_spi_type}, &pyb_spi_obj[4]},
    {{&machine_hard_spi_type}, &pyb_spi_obj[5]},
};

STATIC void machine_hard_spi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_hard_spi_obj_t *self = (machine_hard_spi_obj_t*)self_in;
    spi_print(print, self->pyb->spi, false);
}

mp_obj_t machine_hard_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_id, ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit, ARG_sck, ARG_mosi, ARG_miso };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id,       MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_SMALL_INT(-1)} },
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 500000} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = SPI_FIRSTBIT_MSB} },
        { MP_QSTR_sck,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_mosi,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_miso,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get static peripheral object
    int spi_id = spi_find(args[ARG_id].u_obj);
    const machine_hard_spi_obj_t *self = &machine_hard_spi_obj[spi_id - 1];

    // here we would check the sck/mosi/miso pins and configure them, but it's not implemented
    if (args[ARG_sck].u_obj != MP_OBJ_NULL
        || args[ARG_mosi].u_obj != MP_OBJ_NULL
        || args[ARG_miso].u_obj != MP_OBJ_NULL) {
        mp_raise_ValueError("explicit choice of sck/mosi/miso is not implemented");
    }

    // set the SPI configuration values
    SPI_InitTypeDef *init = &self->pyb->spi->Init;
    init->Mode = SPI_MODE_MASTER;

    // these parameters are not currently configurable
    init->Direction = SPI_DIRECTION_2LINES;
    init->NSS = SPI_NSS_SOFT;
    init->TIMode = SPI_TIMODE_DISABLED;
    init->CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    init->CRCPolynomial = 0;

    // set configurable paramaters
    spi_set_params(self->pyb->spi, 0xffffffff, args[ARG_baudrate].u_int,
        args[ARG_polarity].u_int, args[ARG_phase].u_int, args[ARG_bits].u_int,
        args[ARG_firstbit].u_int);

    // init the SPI bus
    spi_init(self->pyb->spi, false);

    return MP_OBJ_FROM_PTR(self);
}

STATIC void machine_hard_spi_init(mp_obj_base_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    machine_hard_spi_obj_t *self = (machine_hard_spi_obj_t*)self_in;

    enum { ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // set the SPI configuration values
    spi_set_params(self->pyb->spi, 0xffffffff, args[ARG_baudrate].u_int,
        args[ARG_polarity].u_int, args[ARG_phase].u_int, args[ARG_bits].u_int,
        args[ARG_firstbit].u_int);

    // re-init the SPI bus
    spi_init(self->pyb->spi, false);
}

STATIC void machine_hard_spi_deinit(mp_obj_base_t *self_in) {
    machine_hard_spi_obj_t *self = (machine_hard_spi_obj_t*)self_in;
    spi_deinit(self->pyb->spi);
}

STATIC void machine_hard_spi_transfer(mp_obj_base_t *self_in, size_t len, const uint8_t *src, uint8_t *dest) {
    machine_hard_spi_obj_t *self = (machine_hard_spi_obj_t*)self_in;
    spi_transfer(self->pyb, len, src, dest, SPI_TRANSFER_TIMEOUT(len));
}

STATIC const mp_machine_spi_p_t machine_hard_spi_p = {
    .init = machine_hard_spi_init,
    .deinit = machine_hard_spi_deinit,
    .transfer = machine_hard_spi_transfer,
};

const mp_obj_type_t machine_hard_spi_type = {
    { &mp_type_type },
    .name = MP_QSTR_SPI,
    .print = machine_hard_spi_print,
    .make_new = mp_machine_spi_make_new, // delegate to master constructor
    .protocol = &machine_hard_spi_p,
    .locals_dict = (mp_obj_t)&mp_machine_spi_locals_dict,
};
