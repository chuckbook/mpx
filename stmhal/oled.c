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
//#include STM32_HAL_H

#include "py/nlr.h"
#include "py/runtime.h"

#if MICROPY_HW_HAS_OLED

#include "pin.h"
#include "genhdr/pins.h"
#include "bufhelper.h"
#include "i2c.h"
#include "py/mphal.h"
#include "oled.h"

#if MICROPY_HW_HAS_MMI
#define MMI_I2C_ADDR 0x5a
#endif // MICROPY_HW_HAS_MMI
#define OLED_I2C_ADDR 0x62
#define STAT_I2C_ADDR 0x63

#define GPIO_set_pin(gpio, pin_mask)    (((gpio)->BSRR) = (pin_mask))
#define GPIO_clear_pin(gpio, pin_mask)  (((gpio)->BSRR) = ((pin_mask) << 16))

/// \moduleref pyb
/// \class OLED - OLED control for the OLED touch-sensor pyskin
///
/// The OLED class is used to control the OLED on the OLED touch-sensor pyskin,
/// OLED  The OLED is a 128x96 pixel color screen, part OLED_ESG.
///
/// The pyskin must be connected in either the X or Y positions, and then
/// an OLED object is made using:
///
///     oled = pyb.OLED()
///
/// Then you can use:
///
///     oled.light(True)                 # turn the backlight on
///     oled.write('Hello world!\n')     # print text to the screen
///
/// This driver implements a double buffer for setting/getting pixels.
/// For example, to make a bouncing dot, try:
///
///     x = y = 0
///     dx = dy = 1
///     while True:
///         # update the dot's position
///         x += dx
///         y += dy
///
///         # make the dot bounce of the edges of the screen
///         if x <= 0 or x >= 127: dx = -dx
///         if y <= 0 or y >= 31: dy = -dy
///
///         oled.fill(0)                 # clear the buffer
///         oled.pixel(x, y, 1)          # draw the dot
///         oled.show()                  # show the buffer
///         pyb.delay(50)               # pause for 50ms

#define OLED_INSTR (0)
#define OLED_DATA (1)

#define OLED_CHAR_BUF_W (16)
#define OLED_CHAR_BUF_H (4)

#define OLED_PIX_BUF_W (128)
#define OLED_PIX_BUF_H (32)
#define OLED_PIX_BUF_BYTE_SIZE (OLED_PIX_BUF_W * OLED_PIX_BUF_H / 8)

#if 0
typedef struct _pyb_i2c_obj_t {
    mp_obj_base_t base;
    I2C_HandleTypeDef *i2c;
    DMA_Stream_TypeDef *tx_dma_stream;
    uint32_t tx_dma_channel;
    DMA_Stream_TypeDef *rx_dma_stream;
    uint32_t rx_dma_channel;
} pyb_i2c_obj_t;
#endif
typedef struct _pyb_oled_obj_t {
    mp_obj_base_t base;

    // hardware control for the OLED
    I2C_HandleTypeDef *i2ch;
    const pin_obj_t *pin_mmi_int;
    const pin_obj_t *pin_olon;

    // character buffer for stdout-like output
    char char_buffer[OLED_CHAR_BUF_W * OLED_CHAR_BUF_H];
    int line;
    int column;
    int next_line;
    mp_obj_t *i2c;

} pyb_oled_obj_t;

volatile uint32_t led_buf;
volatile uint32_t mmi_cnt;

STATIC void oledw(I2C_HandleTypeDef *i2c, uint8_t *v, int n) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c, OLED_I2C_ADDR << 1, (void *)v, n, 1000);
    if (status);
}

STATIC void mmiw(I2C_HandleTypeDef *i2c, uint8_t v, uint8_t a) {
#if MICROPY_HW_HAS_MMI
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(i2c, MMI_I2C_ADDR << 1, a, I2C_MEMADD_SIZE_8BIT, (void *)&v, 1, 200);
    if (status);
#endif // MICROPY_HW_HAS_MMI
}

STATIC int mmir(I2C_HandleTypeDef *i2c, uint8_t a, uint8_t *dest, int n) {
#if MICROPY_HW_HAS_MMI
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(i2c, MMI_I2C_ADDR << 1, a, I2C_MEMADD_SIZE_8BIT, dest, n, 200);
    if (status) {
        return -1;
    } else {
        return n;
    }
#else
    return 0;
#endif // MICROPY_HW_HAS_MMI
}

STATIC int statr(I2C_HandleTypeDef *i2c, uint8_t *dest, int n) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(i2c, STAT_I2C_ADDR << 1, (uint8_t*)dest, n, 200);
    if (status) {
        return -1;
    } else {
        return n;
    }
}

#if defined(MCU_SERIES_F7)
#define I2C_TIMING_100      0x40912732      // 54 MHz --> 8.76us    --> 100kHz
#define I2C_TIMING_400      0x10911823      // 54 MHz --> 8.76us    --> 400kHz
#define I2C_TIMING_1000      0x00611116      // 54 MHz --> 8.76us    --> 1000kHz

//#define I2C_TIMING      0x00911315          // 15 MHz --> 2.5us (400kHz) -> 3.120us (320kHz)
//#define I2C_TIMING      0x0091100E          // 16 MHz --> 1.9us (400kHz)    --> 2.5us (400kHz)
//#define I2C_TIMING      0x00410807          // 16 MHz --> 0.9us (800kHz)    --> 1.58us (633kHz)
//#define I2C_TIMING      0x00310504          // 16 MHz --> 0.9us (800kHz)    --> 1.28us (781kHz)
//#define I2C_TIMING      0x00110302          // 16 MHz --> 0.9us (800kHz)    --> 1.05us (952kHz)
#endif

#ifdef TSP_20150802
STATIC void oled_start(int baud) {

#if defined(MICROPY_HW_I2C2_SCL)
    HAL_I2C_DeInit(&I2CHandle2);
#elif defined(MICROPY_HW_I2C1_SCL)
    HAL_I2C_DeInit(&I2CHandle1);
#endif

    // start the I2C bus in master mode
#if defined(MICROPY_HW_I2C2_SCL)
    I2CHandle2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
#if defined(MCU_SERIES_F7)
    if (baud >= 1000000) {
        I2CHandle2.Init.Timing      = I2C_TIMING_1000;
    } else if (baud >= 400000) {
        I2CHandle2.Init.Timing      = I2C_TIMING_400;
    } else {
        I2CHandle2.Init.Timing      = I2C_TIMING_100;
    }
#else
    I2CHandle2.Init.ClockSpeed      = baud;             // TSP_20150708
    I2CHandle2.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;             // TSP_20150708
#endif

    I2CHandle2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    I2CHandle2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    I2CHandle2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
    I2CHandle2.Init.OwnAddress1     = PYB_I2C_MASTER_ADDRESS;
    I2CHandle2.Init.OwnAddress2     = 0xfe; // unused
    i2c_init(&I2CHandle2);
#elif defined(MICROPY_HW_I2C1_SCL)
    I2CHandle1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
#if defined(MCU_SERIES_F7)
    if (baud >= 1000000) {
        I2CHandle1.Init.Timing      = I2C_TIMING_1000;
    } else if (baud >= 400000) {
        I2CHandle1.Init.Timing      = I2C_TIMING_400;
    } else {
        I2CHandle1.Init.Timing      = I2C_TIMING_100;
    }
#else
    I2CHandle1.Init.ClockSpeed      = baud;             // TSP_20150708
    I2CHandle1.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;             // TSP_20150708
#endif
    I2CHandle1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    I2CHandle1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    I2CHandle1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLED;
    I2CHandle1.Init.OwnAddress1     = PYB_I2C_MASTER_ADDRESS;
    I2CHandle1.Init.OwnAddress2     = 0xfe; // unused
    i2c_init(&I2CHandle1);
#endif
#if 0
    // turn off AVDD, wait 30ms, turn on AVDD, wait 30ms again
    GPIO_clear_pin(oled->pin_olon->gpio, GPIO_PIN_5);
    HAL_Delay(30);
    GPIO_set_pin(oled->pin_olon->gpio, GPIO_PIN_5);
    HAL_Delay(30);
#endif
    HAL_StatusTypeDef status;

    //printf("IsDeviceReady\n");
    for (int i = 0; i < 10; i++) {
#if defined(MICROPY_HW_I2C1_SCL)
        status = HAL_I2C_IsDeviceReady(&I2CHandle1, STAT_I2C_ADDR << 1, 10, 200);
#elif defined(MICROPY_HW_I2C2_SCL)
        status = HAL_I2C_IsDeviceReady(&I2CHandle2, STAT_I2C_ADDR << 1, 10, 200);
#endif
        //printf("  got %d\n", status);
        if (status == HAL_OK) {
            break;
        }
    }
#if 0
    // set MMA to active mode
    uint8_t data[1] = {1}; // active mode
    status = HAL_I2C_Mem_Write(&I2CHandle2, MMA_ADDR, MMA_REG_MODE, I2C_MEMADD_SIZE_8BIT, data, 1, 200);

    // wait for MMA to become active
    HAL_Delay(30);
#endif
}
#endif

//STATIC uint8_t hs[16] = "0123456789abcdef";

STATIC void oled_out(pyb_oled_obj_t *oled, int instr_data, uint8_t i) {
#if 0
    GPIO_clear_pin(oled->pin_cs1->gpio, oled->pin_cs1->pin_mask);
    if (instr_data == OLED_INSTR) {
        GPIO_clear_pin(oled->pin_a0->gpio, oled->pin_a0->pin_mask);
    } else {
        GPIO_set_pin(oled->pin_a0->gpio, oled->pin_a0->pin_mask);
    }
    HAL_I2C_Transmit(oled->i2ch, &i, 1, 1000);
#else
#if 0
    uint8_t sbuf[128];
    sbuf[0] = hs[i >> 4];
    sbuf[1] = hs[i & 0xf];
    sbuf[2] = 0xd;
    sbuf[3] = 0xa;
#endif
    //sprintf(sbuf, "oled_out %p %d %d\r\n", oled, instr_data, i);
    //HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(oled->i2ch, OLED_I2C_ADDR, &i, 1, 1000);
    //HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(oled->i2ch, OLED_I2C_ADDR, (void *)sbuf, 4, 1000);
    //if (status);

#endif
}

// write a string to the OLED at the current cursor location
// output it straight away (doesn't use the pixel buffer)
STATIC void oled_write_strn(pyb_oled_obj_t *oled, const char *str, unsigned int len) {
#if 1
    HAL_StatusTypeDef status = 0;
    //printf("oled_write_strn %d", len);
    for (int i = 0; i < len; i++ ) {
        status = HAL_I2C_Master_Transmit(oled->i2ch, OLED_I2C_ADDR << 1, (void *)&str[i], 1, 1000);
        if (status);
        //printf("(%d)%c", status, str[i]);
    }
    //printf("\n");
#else
    int redraw_min = oled->line * OLED_CHAR_BUF_W + oled->column;
    int redraw_max = redraw_min;
    for (; len > 0; len--, str++) {
        // move to next line if needed
        if (oled->next_line) {
            if (oled->line + 1 < OLED_CHAR_BUF_H) {
                oled->line += 1;
            } else {
                oled->line = OLED_CHAR_BUF_H - 1;
                for (int i = 0; i < OLED_CHAR_BUF_W * (OLED_CHAR_BUF_H - 1); i++) {
                    oled->char_buffer[i] = oled->char_buffer[i + OLED_CHAR_BUF_W];
                }
                for (int i = 0; i < OLED_CHAR_BUF_W; i++) {
                    oled->char_buffer[OLED_CHAR_BUF_W * (OLED_CHAR_BUF_H - 1) + i] = ' ';
                }
                redraw_min = 0;
                redraw_max = OLED_CHAR_BUF_W * OLED_CHAR_BUF_H;
            }
            oled->next_line = 0;
            oled->column = 0;
        }
        if (*str == '\n') {
            oled->next_line = 1;
        } else if (*str == '\r') {
            oled->column = 0;
        } else if (*str == '\b') {
            if (oled->column > 0) {
                oled->column--;
                redraw_min = 0; // could optimise this to not redraw everything
            }
        } else if (oled->column >= OLED_CHAR_BUF_W) {
            oled->next_line = 1;
            str -= 1;
            len += 1;
        } else {
            oled->char_buffer[oled->line * OLED_CHAR_BUF_W + oled->column] = *str;
            oled->column += 1;
            int max = oled->line * OLED_CHAR_BUF_W + oled->column;
            if (max > redraw_max) {
                redraw_max = max;
            }
        }
    }

    // we must draw upside down, because the OLED is upside down
    for (int i = redraw_min; i < redraw_max; i++) {
        uint page = i / OLED_CHAR_BUF_W;
        uint offset = 8 * (OLED_CHAR_BUF_W - 1 - (i - (page * OLED_CHAR_BUF_W)));
        oled_out(oled, OLED_INSTR, 0xb0 | page); // page address set
        oled_out(oled, OLED_INSTR, 0x10 | ((offset >> 4) & 0x0f)); // column address set upper
        oled_out(oled, OLED_INSTR, 0x00 | (offset & 0x0f)); // column address set lower
        int chr = oled->char_buffer[i];
        if (chr < 32 || chr > 126) {
            chr = 127;
        }
        const uint8_t *chr_data = &font_petme128_8x8[(chr - 32) * 8];
        for (int j = 7; j >= 0; j--) {
            oled_out(oled, OLED_DATA, chr_data[j]);
        }
    }
#endif
}

/// \classmethod \constructor(skin_position)
///
/// Construct an OLED object in the given skin position.  `skin_position` can be 'X' or 'Y', and
/// should match the position where the OLED pyskin is plugged in.
STATIC mp_obj_t pyb_oled_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
#if 0
    static const mp_arg_t default_args[] = {
        { MP_QSTR_addr,     MP_ARG_INT, {.u_int = 2 } },
        { MP_QSTR_mode,     MP_ARG_INT, {.u_int = PYB_I2C_MASTER_ADDRESS} },
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 400000} },
    };
#endif
#if 0
    static uint32_t default_args[] = {
        5,
        (uint32_t)(MP_QSTR_mode)*4+2,
        1,
        (uint32_t)(MP_QSTR_baudrate)*4+2,
        400000*2+1,
    };
#else
    static mp_obj_t default_args[] = {
                MP_OBJ_NEW_SMALL_INT(2),
                MP_OBJ_NEW_QSTR(MP_QSTR_mode),
                MP_OBJ_NEW_SMALL_INT(0),
                MP_OBJ_NEW_QSTR(MP_QSTR_baudrate),
                MP_OBJ_NEW_SMALL_INT(400000),
    };
#endif
//printf("pyb_oled_make_new: %p %d %d %p\n", type, n_args, n_kw, args);
    // check arguments
    //mp_arg_check_num(n_args, n_kw, 0, 0, false);
    if (n_args) {
        mp_arg_check_num(n_args, n_kw, 0, 3, true);
#if 0
        int i;
        uint32_t *p = (uint32_t *)args;
        for (i = 0; i < 9; i++) {
            printf(" %08x", (unsigned int)p[i]);
        }
        printf("\n");
#endif
    } else {
        n_args = 1;
        n_kw = 2;
        args = (const mp_obj_t *)default_args;
//printf("pyb_oled_make_new(def): %d\n", mp_obj_get_int(args[0]));
    }

    // get OLED position
    //const char *oled_id = mp_obj_str_get_str(args[0]);
    // create oled object
    pyb_oled_obj_t *oled = m_new_obj(pyb_oled_obj_t);
    oled->base.type = &pyb_oled_type;

    // configure pins
    // TODO accept an I2C object and pin objects for full customisation
    //printf("pyb_oled_make_new %s\n", oled_id);
    //printf("pyb_oled_make_new\n");
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx)
    //oled->pin_mmi_int = &pin_G5; // MMI_INT
#if MICROPY_HW_HAS_MMI
    oled->pin_mmi_int = &MICROPY_HW_MMI_INT;
#endif
    //oled->pin_olon = &pin_A15; // OLED_RES
    oled->pin_olon = &MICROPY_HW_OLED_RES;
#else
#if MICROPY_HW_HAS_MMI
    oled->pin_mmi_int = &MICROPY_HW_MMI_INT;
#endif
    oled->pin_olon = &MICROPY_HW_OLED_RES;
    //oled->pin_mmi_int = &pin_B1; // MMI_INT
    //oled->pin_olon = &pin_B0; // OLED_RES
#endif
    // set the pins to default values
    GPIO_set_pin(oled->pin_olon->gpio, oled->pin_olon->pin_mask);

    // init the pins to be push/pull outputs
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = oled->pin_olon->pin_mask;
    HAL_GPIO_Init(oled->pin_olon->gpio, &GPIO_InitStructure);

    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Pin = oled->pin_mmi_int->pin_mask;
    HAL_GPIO_Init(oled->pin_mmi_int->gpio, &GPIO_InitStructure);

    // init the OLED
    if (oled->pin_olon->gpio->IDR & oled->pin_olon->pin_mask) {
        // power up OLED
        GPIO_clear_pin(oled->pin_olon->gpio, oled->pin_olon->pin_mask);
        HAL_Delay(1000); // wait for reset; 2us min
    }
#if 0
    #if defined(MICROPY_HW_I2C2_SCL)
    oled->i2ch = &I2CHandle2;
    #elif defined(MICROPY_HW_I2C1_SCL)
    oled->i2ch = &I2CHandle1;
    #else
    #warning NO I2C
    #endif
#endif
#if 0
    int i = 2;
    if (n_args > 1) {
        i = mp_obj_get_int(args[1])-1;
    }
#endif
//STATIC mp_obj_t pyb_oled_make_new(mp_obj_t type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args)
    oled->i2c = pyb_i2c_type.make_new(type, n_args, n_kw, args);
//printf("pyb_oled_make_new: %p %d %d %p %p\n", type, n_args, n_kw, args, oled->i2c);
    if (oled->i2c == NULL) {
        return mp_const_none;
    }
#if 0
    oled->i2ch = ((pyb_i2c_obj_t *)oled->i2c)->i2c;
#else
    I2C_HandleTypeDef *get_i2c_handle(void *i2c);
    
    oled->i2ch = get_i2c_handle(oled->i2c);
#endif
//printf("pyb_oled_make_new: %p\n", oled->i2ch);
#if TSP_20150802
    //oled->i2c = // pyb_i2c_obf[i].i2c;
    if (n_args > 0) {
        oled_start(mp_obj_get_int(args[0]));
    } else {
        oled_start(400000);
    }
#endif
#if 0
    printf("scan:");
    for (uint addr = 1; addr <= 127; addr++) {
        for (int i = 0; i < 10; i++) {
            HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(oled->i2ch, addr << 1, 10, 200);
            if (status == HAL_OK) {
                printf(" 0x%02x", addr);
                break;
            }
        }
    }
    printf("\n");
#endif
    mmiw(oled->i2ch, 0x63, 0x80);
    mmiw(oled->i2ch, 0x00, 0x5e);

  //o.mmi_send(oled->i2ch, 0x88, 0x5b)

  /* set baseline to 0x46...

    mmiw(oled->i2ch, 0x46, 0x1e);
    mmiw(oled->i2ch, 0x46, 0x1f);
    mmiw(oled->i2ch, 0x46, 0x20);
    mmiw(oled->i2ch, 0x46, 0x21);
    mmiw(oled->i2ch, 0x46, 0x22);
    mmiw(oled->i2ch, 0x46, 0x23);
  */
    mmiw(oled->i2ch, 0x00, 0x5e);

    mmiw(oled->i2ch, 0x01, 0x2b);
    mmiw(oled->i2ch, 0x01, 0x2c);
    mmiw(oled->i2ch, 0x00, 0x2d);
    mmiw(oled->i2ch, 0x00, 0x2e);
    mmiw(oled->i2ch, 0x01, 0x2f);
    mmiw(oled->i2ch, 0x01, 0x30);
    mmiw(oled->i2ch, 0xff, 0x31);
    mmiw(oled->i2ch, 0x02, 0x32);

    for (int i = 0; i < 6; i++) {
        mmiw(oled->i2ch, 0x02, 0x41+i*2);
        mmiw(oled->i2ch, 0x01, 0x42+i*2);
    }

    mmiw(oled->i2ch, 0x0b, 0x7b);
    mmiw(oled->i2ch, 0x9c, 0x7d);
    mmiw(oled->i2ch, 0x65, 0x7e);
    mmiw(oled->i2ch, 0x8c, 0x7f);

    mmiw(oled->i2ch, 0xe0, 0x73);
    mmiw(oled->i2ch, 0x00, 0x74);
    mmiw(oled->i2ch, 0xe0, 0x76);
    mmiw(oled->i2ch, 0xe0, 0x77);

    mmiw(oled->i2ch, 0x86, 0x5e);
#if 0
    int vline;
    vline = oled->pin_mmi_int->pin;
    printf("vline: %d\n", vline);
#endif
#if MICROPY_HW_HAS_MMI
#define MY_PRIO 0x0f

    GPIO_InitTypeDef exti;
    exti.Mode = GPIO_MODE_IT_FALLING;
    exti.Pull = GPIO_PULLUP;
    exti.Speed = GPIO_SPEED_FAST;
    exti.Pin = oled->pin_mmi_int->pin_mask;
    HAL_GPIO_Init(oled->pin_mmi_int->gpio, &exti);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, MY_PRIO, MY_PRIO);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif

    return oled;
}

/// \method command(instr_data, buf)
///
/// Send an arbitrary command to the OLED.  Pass 0 for `instr_data` to send an
/// instruction, otherwise pass 1 to send data.  `buf` is a buffer with the
/// instructions/data to send.
STATIC mp_obj_t pyb_oled_command(mp_obj_t self_in, mp_obj_t instr_data_in, mp_obj_t val) {
    pyb_oled_obj_t *self = self_in;

    // get whether instr or data
    int instr_data = mp_obj_get_int(instr_data_in);

    // get the buffer to send from
    mp_buffer_info_t bufinfo;
    uint8_t data[1];
    pyb_buf_get_for_send(val, &bufinfo, data);

    // send the data
    for (uint i = 0; i < bufinfo.len; i++) {
        oled_out(self, instr_data, ((byte*)bufinfo.buf)[i]);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_oled_command_obj, pyb_oled_command);

/// \method contrast(value)
///
/// Set the contrast of the OLED.  Valid values are between 0 and 47.
STATIC mp_obj_t pyb_oled_contrast(mp_obj_t self_in, mp_obj_t contrast_in) {
    pyb_oled_obj_t *self = self_in;
    int contrast = mp_obj_get_int(contrast_in);
    if (contrast < 0) {
        contrast = 0;
    } else if (contrast > 0x2f) {
        contrast = 0x2f;
    }
    oled_out(self, OLED_INSTR, 0x81); // electronic volume mode set
    oled_out(self, OLED_INSTR, contrast); // electronic volume register set
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_contrast_obj, pyb_oled_contrast);

/// \method light(value)
///
/// Turn the backlight on/off.  True or 1 turns it on, False or 0 turns it off.
STATIC mp_obj_t pyb_oled_light(mp_obj_t self_in, mp_obj_t value) {
    //pyb_oled_obj_t *self = self_in;
#if 1
    //printf("oled_light %p\n", self);
#else
    if (mp_obj_is_true(value)) {
        GPIO_set_pin(oled->pin_bl->gpio, oled->pin_bl->pin_mask);
    } else {
        GPIO_clear_pin(oled->pin_bl->gpio, oled->pin_bl->pin_mask);
    }
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_light_obj, pyb_oled_light);

/// \method write(str)
///
/// Write the string `str` to the screen.  It will appear immediately.
STATIC mp_obj_t pyb_oled_write(mp_obj_t self_in, mp_obj_t str) {
    pyb_oled_obj_t *self = self_in;
    mp_uint_t len;
    const char *data = mp_obj_str_get_data(str, &len);
    oled_write_strn(self, data, len);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_write_obj, pyb_oled_write);

/// \method fill(colour)
///
/// Fill the screen with the given colour (0 or 1 for white or black).
///
/// This method writes to the hidden buffer.  Use `show()` to show the buffer.
STATIC mp_obj_t pyb_oled_fill(mp_obj_t self_in, mp_obj_t col_in) {
#if 1
    //printf("oled_fill %p\n", col_in);
#else
    pyb_oled_obj_t *self = self_in;
    int col = mp_obj_get_int(col_in);
    if (col) {
        col = 0xff;
    }
    memset(self->pix_buf, col, OLED_PIX_BUF_BYTE_SIZE);
    memset(self->pix_buf2, col, OLED_PIX_BUF_BYTE_SIZE);
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_fill_obj, pyb_oled_fill);

/// \method get(x, y)
///
/// Get the pixel at the position `(x, y)`.  Returns 0 or 1.
///
/// This method reads from the visible buffer.
STATIC mp_obj_t pyb_oled_get(mp_obj_t self_in, mp_obj_t x_in, mp_obj_t y_in) {
#if 1
    //printf("oled_get %p,%p\n", x_in, y_in);
#else
    pyb_oled_obj_t *self = self_in;
    int x = mp_obj_get_int(x_in);
    int y = mp_obj_get_int(y_in);
    if (0 <= x && x <= 127 && 0 <= y && y <= 31) {
        uint byte_pos = x + 128 * ((uint)y >> 3);
        if (self->pix_buf[byte_pos] & (1 << (y & 7))) {
            return mp_obj_new_int(1);
        }
    }
#endif
    return mp_obj_new_int(0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_oled_get_obj, pyb_oled_get);

/// \method pixel(x, y, colour)
///
/// Set the pixel at `(x, y)` to the given colour (0 or 1).
///
/// This method writes to the hidden buffer.  Use `show()` to show the buffer.
STATIC mp_obj_t pyb_oled_pixel(mp_uint_t n_args, const mp_obj_t *args) {
    //pyb_oled_obj_t *self = args[0];
    //int x = mp_obj_get_int(args[1]);
    //int y = mp_obj_get_int(args[2]);
#if 1
    //printf("oled_pixel %p %d,%d\n", self, x, y);
#else
    if (0 <= x && x <= 127 && 0 <= y && y <= 31) {
        uint byte_pos = x + 128 * ((uint)y >> 3);
        if (mp_obj_get_int(args[3]) == 0) {
            self->pix_buf2[byte_pos] &= ~(1 << (y & 7));
        } else {
            self->pix_buf2[byte_pos] |= 1 << (y & 7);
        }
    }
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_oled_pixel_obj, 4, 4, pyb_oled_pixel);

/// \method mmi_read(n, a)
STATIC mp_obj_t pyb_mmi_read(mp_obj_t self_in, mp_obj_t p1, mp_obj_t p2) {
    // extract arguments
    pyb_oled_obj_t *self = self_in;
    int n = mp_obj_get_int(p1);
    int a = mp_obj_get_int(p2);
    uint8_t *dest = m_new(uint8_t, n);

    //HAL_StatusTypeDef status = 0;
    if (mmir(self->i2ch, a, dest, n) < 0) {
        return mp_const_none;
    } else {
    //status = HAL_I2C_Mem_Read(self->i2ch, MMI_I2C_ADDR << 1, a, I2C_MEMADD_SIZE_8BIT, dest, n, 200);
    //if (status);
        return mp_obj_new_bytearray_by_ref(n, dest);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_mmi_read_obj, pyb_mmi_read);

/// \method mmi_send(n, a)
STATIC mp_obj_t pyb_mmi_send(mp_obj_t self_in, mp_obj_t p1, mp_obj_t p2) {
    // extract arguments
    pyb_oled_obj_t *self = self_in;
    //uint len;
    //const char *data = mp_obj_str_get_data(p1, &len);
    int n = mp_obj_get_int(p1);
    int a = mp_obj_get_int(p2);

    //HAL_StatusTypeDef status = 0;
    mmiw(self->i2ch, n, a);
    //status = HAL_I2C_Mem_Write(self->i2ch, MMI_I2C_ADDR << 1, a, I2C_MEMADD_SIZE_8BIT, (void *)&n, 1, 200);
    //if (status);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_mmi_send_obj, pyb_mmi_send);

/*
 * baseline
 */
STATIC mp_obj_t pyb_mmi_baseline(mp_obj_t self_in, mp_obj_t p1) {
    pyb_oled_obj_t *self = self_in;

    if (MP_OBJ_IS_TYPE(p1, &mp_type_bytearray)) {
        mp_buffer_info_t bufinfo_trace;
        mp_get_buffer(p1, &bufinfo_trace, MP_BUFFER_RW);
        if ( bufinfo_trace.typecode == 0) {
            int len = bufinfo_trace.len;
            uint8_t *p = bufinfo_trace.buf;
            if (len > 6) {
                len = 6;
            }
            for (int i = 0; i < len; i++) {
                mmiw(self->i2ch, p[i], 0x1e + i);
            }
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_mmi_baseline_obj, pyb_mmi_baseline);

/*
 * keypressed
 */
volatile uint16_t kbd_buf[2];
volatile uint32_t lmmi_cnt;
//STATIC mp_obj_t pyb_keypressed(mp_obj_t self_in)
STATIC mp_obj_t pyb_keypressed(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_stat, MP_ARG_OBJ, {.u_obj = mp_const_none } },
    };
    pyb_oled_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int k = 0;
    int blen = 0;
    uint8_t *buf = NULL;
    if (args[0].u_obj != mp_const_none) {
        //if (verbose) printf( "stat: %p\n", args[0].u_obj );
        if (MP_OBJ_IS_TYPE(args[0].u_obj, &mp_type_bytearray)) {
            mp_buffer_info_t bufinfo_stat;
            mp_get_buffer(args[0].u_obj, &bufinfo_stat, MP_BUFFER_RW);
            if ( bufinfo_stat.typecode == 0) {
                blen = bufinfo_stat.len;
                buf = bufinfo_stat.buf;
            }
        }
    }

    if (lmmi_cnt != mmi_cnt) {
        lmmi_cnt = mmi_cnt;
        if (blen > 0) {
            mmir(self->i2ch, 0, (uint8_t *)buf, blen);
            kbd_buf[0] = buf[0];
        } else {
            //mmir(&I2CHandle2, 0, (uint8_t *)kbd_buf, 1);
            mmir(self->i2ch, 0, (uint8_t *)kbd_buf, 1);
            kbd_buf[0] &= 0xff;
        }
        k=kbd_buf[0] | kbd_buf[1];
        k^=kbd_buf[1];
        kbd_buf[1] = kbd_buf[0];
    } else if (blen > 0) {
        mmir(self->i2ch, 0, (uint8_t *)buf, blen);
    }
    return mp_obj_new_int(k);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_keypressed_obj, 0, pyb_keypressed);

/*
 */
STATIC mp_obj_t pyb_oled_reset(mp_obj_t self_in) {
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_oled_reset_obj, pyb_oled_reset);

/*
 */

#ifdef TSP_20150802
STATIC const pyb_i2c_obj_t pyb_i2c_obj[] = {
#if defined(MICROPY_HW_I2C1_SCL)
    {{&pyb_i2c_type}, &I2CHandle1},
#else
    {{&pyb_i2c_type}, NULL},
#endif
#if defined(MICROPY_HW_I2C2_SCL)
    {{&pyb_i2c_type}, &I2CHandle2}
#else
    {{&pyb_i2c_type}, NULL},
#endif
};
#endif

STATIC mp_obj_t pyb_oled_i2c(mp_obj_t self_in) {
#ifdef TSP_20150802
    const pyb_i2c_obj_t *i2c_obj = &pyb_i2c_obj[1];
    return (mp_obj_t)i2c_obj;
#else
    pyb_oled_obj_t *self = self_in;
    return self->i2c;
#endif
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_oled_i2c_obj, pyb_oled_i2c);

/*
 * power
 */
STATIC mp_obj_t pyb_oled_power(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_on, MP_ARG_INT, {.u_int = -1} },
    };
    pyb_oled_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int lstate = (self->pin_olon->gpio->IDR & self->pin_olon->pin_mask)?0:1;
    if (args[0].u_int == 0) {
        GPIO_set_pin(self->pin_olon->gpio, self->pin_olon->pin_mask);
    } else if (args[0].u_int > 0) {
        GPIO_clear_pin(self->pin_olon->gpio, self->pin_olon->pin_mask);
    }
    return mp_obj_new_int(lstate);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_oled_power_obj, 0, pyb_oled_power);

/*
 * rgbcol
 */
STATIC mp_obj_t pyb_oled_rgbcol(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_red, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_green, MP_ARG_INT, {.u_int = 255 } },
        { MP_QSTR_blue, MP_ARG_INT, {.u_int = 0 } },
    };
    //pyb_oled_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    return mp_obj_new_int(((args[2].u_int & 0xf8) << 8) | ((args[1].u_int & 0xfc) << 3) | (args[0].u_int >> 3));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_oled_rgbcol_obj, 0, pyb_oled_rgbcol);

/*
 * dat
 */
STATIC mp_obj_t pyb_oled_dat(mp_obj_t self_in, mp_obj_t p1) {
    pyb_oled_obj_t *self = self_in;

    if (MP_OBJ_IS_TYPE(p1, &mp_type_bytearray)) {
        mp_buffer_info_t bufinfo_trace;
        mp_get_buffer(p1, &bufinfo_trace, MP_BUFFER_RW);
        if ( bufinfo_trace.typecode == 0) {
            oledw(self->i2ch, bufinfo_trace.buf, bufinfo_trace.len);
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_dat_obj, pyb_oled_dat);

/*
 * stat
 */
//STATIC mp_obj_t pyb_oled_stat(mp_obj_t self_in, mp_obj_t p1)
STATIC mp_obj_t pyb_oled_stat(mp_obj_t self_in)
{
    pyb_oled_obj_t *self = self_in;
    uint8_t b[4];
    if (statr(self->i2ch, b, 1) == 1) {
//printf("statr: %d\n", b[0]);
        return mp_obj_new_int(b[0]);
    } else {
//printf("statr: -1\n");
        return mp_obj_new_int(-1);
    }
}
//STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_stat_obj, pyb_oled_stat);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_oled_stat_obj, pyb_oled_stat);

/*
 * cmd
 */
STATIC mp_obj_t pyb_oled_cmd(mp_obj_t self_in, mp_obj_t p1) {
    pyb_oled_obj_t *self = self_in;
    int v = mp_obj_get_int(p1);
    uint8_t b[4];
    b[0] = 2;
    b[1] = v & 0xff;
    oledw(self->i2ch, b, 2);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_cmd_obj, pyb_oled_cmd);

/*
 * pos
 */
STATIC mp_obj_t pyb_oled_pos(mp_obj_t self_in, mp_obj_t p1, mp_obj_t p2) {
    pyb_oled_obj_t *self = self_in;
    int x = mp_obj_get_int(p1);
    int y = mp_obj_get_int(p2);
    uint8_t b[4];
    b[0] = 2;
    b[1] = 0x58;
    b[2] = x & 0x7f;
    b[3] = y & 0x7f;
    oledw(self->i2ch, b, 4);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_oled_pos_obj, pyb_oled_pos);

/*
 * wdat
 */
STATIC mp_obj_t pyb_oled_wdat(mp_obj_t self_in, mp_obj_t p1) {
    pyb_oled_obj_t *self = self_in;
    int v = mp_obj_get_int(p1);
    uint8_t b[4];
    b[0] = v & 0xff;
    b[1] = (v >> 8) & 0xff;
    oledw(self->i2ch, b, 2);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_wdat_obj, pyb_oled_wdat);

/*
 * onoff
 */
STATIC mp_obj_t pyb_oled_onoff(mp_obj_t self_in, mp_obj_t p1) {
    pyb_oled_obj_t *self = self_in;
    int onoff = mp_obj_get_int(p1);
    uint8_t b[4];
    b[0] = 2;
    b[1] = 0x76 ^ ((onoff & 1) << 5);
    oledw(self->i2ch, b, 2);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_onoff_obj, pyb_oled_onoff);

/*
 * res
 */
STATIC mp_obj_t pyb_oled_res(mp_obj_t self_in) {
    pyb_oled_obj_t *self = self_in;
    uint8_t b[8];
    b[0] = 2;
    b[1] = 0x59;
    b[2] = 0xef;
    b[3] = 0xbe;
    b[4] = 0xad;
    b[5] = 0xde;

    oledw(self->i2ch, b, 6);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_oled_res_obj, pyb_oled_res);

/*
 * colors
 */
STATIC mp_obj_t pyb_oled_colors(mp_obj_t self_in, mp_obj_t p1, mp_obj_t p2) {
    pyb_oled_obj_t *self = self_in;
    int fg = mp_obj_get_int(p1);
    int bg = mp_obj_get_int(p2);
    uint8_t b[8];
    b[0] = 2;
    b[1] = 0x63;
    b[2] = fg & 0xff;
    b[3] = (fg >> 8) & 0xff;
    b[4] = bg & 0xff;
    b[5] = (bg >> 8) & 0xff;
    oledw(self->i2ch, b, 6);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_oled_colors_obj, pyb_oled_colors);

/*
 * pen
 */
STATIC mp_obj_t pyb_oled_pen(mp_obj_t self_in, mp_obj_t p1, mp_obj_t p2) {
    pyb_oled_obj_t *self = self_in;
    int pen = mp_obj_get_int(p1);
    int fill = mp_obj_get_int(p2);
    uint8_t b[8];
    b[0] = 2;
    b[1] = 0x50;
    b[2] = pen & 0xff;
    b[3] = (pen >> 8) & 0xff;
    b[4] = fill & 0xff;
    b[5] = (fill >> 8) & 0xff;
    oledw(self->i2ch, b, 6);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_oled_pen_obj, pyb_oled_pen);

/*
 * line
 */
STATIC mp_obj_t pyb_oled_line(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_y, MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_xe, MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_ye, MP_ARG_INT, {.u_int = 0 } },
    };
    pyb_oled_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t buf[8];
    buf[0] = 2;
    buf[1] = 0x4c;
    buf[2] = args[0].u_int;
    buf[3] = args[1].u_int;
    buf[4] = args[2].u_int;
    buf[5] = args[3].u_int;

    oledw(self->i2ch, buf, 6);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_oled_line_obj, 0, pyb_oled_line);

/*
 * clw
 */
STATIC mp_obj_t pyb_oled_clw(mp_obj_t self_in) {
    pyb_oled_obj_t *self = self_in;
    uint8_t b[4];
    b[0] = 2;
    b[1] = 0x45;
    oledw(self->i2ch, b, 2);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_oled_clw_obj, pyb_oled_clw);

/*
 * poly
 */
STATIC mp_obj_t pyb_oled_poly(mp_obj_t self_in, mp_obj_t p1) {
    pyb_oled_obj_t *self = self_in;

    if (MP_OBJ_IS_TYPE(p1, &mp_type_bytearray)) {
        mp_buffer_info_t bufinfo_trace;
        mp_get_buffer(p1, &bufinfo_trace, MP_BUFFER_RW);
        if ( bufinfo_trace.typecode == 0) {
            uint8_t b[4];
            b[0] = 2;
            b[1] = 0x6c;
            b[2] = bufinfo_trace.len & 0xff;
            oledw(self->i2ch, b, 3);
            oledw(self->i2ch, bufinfo_trace.buf, b[2]);
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_poly_obj, pyb_oled_poly);

/*
 * rect
 */
STATIC mp_obj_t pyb_oled_rect(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_y, MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_w, MP_ARG_INT, {.u_int = 127 } },
        { MP_QSTR_h, MP_ARG_INT, {.u_int = 95 } },
    };
    pyb_oled_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t buf[8];
    buf[0] = 2;
    buf[1] = 0x72;
    buf[2] = args[0].u_int;
    buf[3] = args[1].u_int;
    buf[4] = args[2].u_int;
    buf[5] = args[3].u_int;

    oledw(self->i2ch, buf, 6);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_oled_rect_obj, 0, pyb_oled_rect);


/*
 * win
 */
STATIC mp_obj_t pyb_oled_win(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_y, MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_w, MP_ARG_INT, {.u_int = 128 } },
        { MP_QSTR_h, MP_ARG_INT, {.u_int = 96 } },
    };
    pyb_oled_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t buf[8];
    buf[0] = 2;
    buf[1] = 0x79;
    buf[2] = args[0].u_int;
    buf[3] = args[1].u_int;
    buf[4] = args[2].u_int;
    buf[5] = args[3].u_int;

    oledw(self->i2ch, buf, 6);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_oled_win_obj, 0, pyb_oled_win);

/*
 * font
 */
STATIC mp_obj_t pyb_oled_font(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_font, MP_ARG_INT, {.u_int = 1} },
        { MP_QSTR_scale, MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_bold, MP_ARG_INT, {.u_int = 0 } },
        { MP_QSTR_fdir, MP_ARG_INT, {.u_int = 0 } },
    };
    pyb_oled_obj_t *self = pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t buf[4];
    int font = args[0].u_int;
    int scale = args[1].u_int;
    int bold = args[2].u_int;
    int fdir = args[3].u_int;
    buf[0] = 2;
    buf[1] = 0x46;
    buf[2] = (( fdir & 3 ) << 6) | ((font & 3) << 4) | (bold & 0xf);
    buf[3] = scale & 0xff;

    oledw(self->i2ch, buf, 4);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_oled_font_obj, 0, pyb_oled_font);

/*
 */
STATIC mp_obj_t pyb_oled_led(mp_uint_t n_args, const mp_obj_t *args)
{
    pyb_oled_obj_t *self = args[0];
    if (n_args == 2) {
        led_buf = mp_obj_get_int(args[1]) << 5;
    }
    mmiw(self->i2ch, led_buf, 0x75);

    return mp_obj_new_int((led_buf >> 5) & 7);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_oled_led_obj, 1, 2, pyb_oled_led);

/*
 */
STATIC mp_obj_t pyb_mmi_cnt(mp_uint_t n_args, const mp_obj_t *args)
{
    //pyb_oled_obj_t *self = args[0];
    if (n_args == 2) {
        mmi_cnt = mp_obj_get_int(args[1]);
    }
    //mmiw(self->i2ch, led_buf, 0x75);

    return mp_obj_new_int(mmi_cnt);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_mmi_cnt_obj, 1, 2, pyb_mmi_cnt);

/// \method string(str, x, y, colour)
///
/// Draw the given string to the curent position current color.
///
/// This method writes to the hidden buffer.  Use `show()` to show the buffer.
STATIC mp_obj_t pyb_oled_string(mp_obj_t self_in, mp_obj_t str) {
    // extract arguments
    pyb_oled_obj_t *self = self_in;
#if 0
    {
        printf("%08x %08x\n", (unsigned int)self_in, (unsigned int)str );
    }
#endif
    uint len;
    const char *data = mp_obj_str_get_data(str, &len);
    oled_write_strn(self, data, len);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_oled_string_obj, pyb_oled_string);

/// \method show()
///
/// Show the hidden buffer on the screen.
STATIC mp_obj_t pyb_oled_show(mp_obj_t self_in) {
    //pyb_oled_obj_t *self = self_in;
#if 1
    //printf("oled_show %p\n", self);
#else
    memcpy(self->pix_buf, self->pix_buf2, OLED_PIX_BUF_BYTE_SIZE);
    for (uint page = 0; page < 4; page++) {
        oled_out(self, OLED_INSTR, 0xb0 | page); // page address set
        oled_out(self, OLED_INSTR, 0x10); // column address set upper; 0
        oled_out(self, OLED_INSTR, 0x00); // column address set lower; 0
        for (uint i = 0; i < 128; i++) {
            oled_out(self, OLED_DATA, self->pix_buf[128 * page + 127 - i]);
        }
    }
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_oled_show_obj, pyb_oled_show);

typedef struct _oled_mem_obj_t {
    mp_obj_base_t base;
    uint32_t elem_type;
} oled_mem_obj_t;

STATIC uint32_t rgbcol( int r, int g, int b) {
    return ((b & 0xf8) << 8) | ((g & 0xfc) << 3) | (r >> 3);
}

//STATIC void oled_mem_print(void (*print)(void *env, const char *fmt, ...), void *env, mp_obj_t self_in, mp_print_kind_t kind)
STATIC void oled_mem_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    oled_mem_obj_t *self = self_in;
    switch (self->elem_type) {
    case 0: mp_printf(print, "%d", 0); break;
    //case 1: mp_printf(print, "%d", led_buf >> 5); break;
    default: mp_printf(print, "%d", self->elem_type); break;
    }
}

STATIC mp_obj_t oled_mem_subscr(mp_obj_t self_in, mp_obj_t index, mp_obj_t value) {
    // TODO support slice index to read/write multiple values at once
    oled_mem_obj_t *self = self_in;
    if (value == MP_OBJ_NULL) {
        // delete
        return MP_OBJ_NULL; // op not supported
    } else if (value == MP_OBJ_SENTINEL) {
        if (self->elem_type == 0) {
        // load
        uint32_t v;

        switch ( mp_obj_get_int(index) ) {
        case 0: v=rgbcol(0,0,0); break;
        case 1: v=rgbcol(48,0,0); break;
        case 2: v=rgbcol(200,0,0); break;
        case 3: v=rgbcol(200,100,0); break;
        case 4: v=rgbcol(200,200,0); break;
        case 5: v=rgbcol(0,200,0); break;
        case 6: v=rgbcol(0,0,200); break;
        case 7: v=rgbcol(120,0,120); break;
        case 8: v=rgbcol(100,100,100); break;
        case 9:
        default: v=rgbcol(200,200,200); break;
        }
        return mp_obj_new_int(v);
        } else {
            return MP_OBJ_NULL; // op not supported
        }
    } else {
        return MP_OBJ_NULL; // op not supported
    }
}

STATIC const mp_obj_type_t oled_mem_type = {
    { &mp_type_type },
    .name = MP_QSTR_mem,
    .print = oled_mem_print,
    .subscr = oled_mem_subscr,
};

STATIC const oled_mem_obj_t oled_color10_obj = {{&oled_mem_type}, (uint32_t)0 };
//STATIC const oled_mem_obj_t oled_led_buf_obj = {{&oled_mem_type}, (uint32_t)1 };

STATIC const mp_map_elem_t pyb_oled_locals_dict_table[] = {
    // instance methods
    { MP_OBJ_NEW_QSTR(MP_QSTR_command), (mp_obj_t)&pyb_oled_command_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_contrast), (mp_obj_t)&pyb_oled_contrast_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_light), (mp_obj_t)&pyb_oled_light_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_write), (mp_obj_t)&pyb_oled_write_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_fill), (mp_obj_t)&pyb_oled_fill_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get), (mp_obj_t)&pyb_oled_get_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pixel), (mp_obj_t)&pyb_oled_pixel_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_show), (mp_obj_t)&pyb_oled_show_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_mmi_read), (mp_obj_t)&pyb_mmi_read_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mmi_send), (mp_obj_t)&pyb_mmi_send_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mmi_baseline), (mp_obj_t)&pyb_mmi_baseline_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_mmi_cnt), (mp_obj_t)&pyb_mmi_cnt_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_keypressed), (mp_obj_t)&pyb_keypressed_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_reset), (mp_obj_t)&pyb_oled_reset_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_i2c), (mp_obj_t)&pyb_oled_i2c_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_power), (mp_obj_t)&pyb_oled_power_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_rgbcol), (mp_obj_t)&pyb_oled_rgbcol_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_dat), (mp_obj_t)&pyb_oled_dat_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_cmd), (mp_obj_t)&pyb_oled_cmd_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pos), (mp_obj_t)&pyb_oled_pos_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_wdat), (mp_obj_t)&pyb_oled_wdat_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_onoff), (mp_obj_t)&pyb_oled_onoff_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_res), (mp_obj_t)&pyb_oled_res_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_colors), (mp_obj_t)&pyb_oled_colors_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_pen), (mp_obj_t)&pyb_oled_pen_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_string), (mp_obj_t)&pyb_oled_string_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_line), (mp_obj_t)&pyb_oled_line_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_clw), (mp_obj_t)&pyb_oled_clw_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_poly), (mp_obj_t)&pyb_oled_poly_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_rect), (mp_obj_t)&pyb_oled_rect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_win), (mp_obj_t)&pyb_oled_win_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_font), (mp_obj_t)&pyb_oled_font_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_led), (mp_obj_t)&pyb_oled_led_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_stat), (mp_obj_t)&pyb_oled_stat_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_color10), (mp_obj_t)&oled_color10_obj },
    //{ MP_OBJ_NEW_QSTR(MP_QSTR_led_buf), (mp_obj_t)&oled_led_buf_obj },
};

STATIC MP_DEFINE_CONST_DICT(pyb_oled_locals_dict, pyb_oled_locals_dict_table);

const mp_obj_type_t pyb_oled_type = {
    { &mp_type_type },
    .name = MP_QSTR_OLED,
    .make_new = pyb_oled_make_new,
    .locals_dict = (mp_obj_t)&pyb_oled_locals_dict,
};

void pyb_oled_exti_handler(uint32_t line) {
    //uint32_t xts = TIM5->CNT;
    //__HAL_GPIO_EXTI_CLEAR_FLAG(1 << line);   //  XXX
    mmi_cnt++;
}

#endif // MICROPY_HW_HAS_OLED
