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

extern SPI_HandleTypeDef SPIHandle1;
extern SPI_HandleTypeDef SPIHandle2;
extern SPI_HandleTypeDef SPIHandle3;
extern SPI_HandleTypeDef SPIHandle4;
extern SPI_HandleTypeDef SPIHandle5;
extern SPI_HandleTypeDef SPIHandle6;
extern const mp_obj_type_t pyb_spi_type;
extern const mp_obj_type_t machine_soft_spi_type;
extern const mp_obj_type_t machine_hard_spi_type;

void spi_init0(void);
void spi_init(SPI_HandleTypeDef *spi, bool enable_nss_pin);
#if 1       // DPG_20170305
void spi_deinit(SPI_HandleTypeDef *spi);
#endif
SPI_HandleTypeDef *spi_get_handle(mp_obj_t o);

#if 1       // DPG_20170305
struct _pyb_spi_obj_t;
const struct _pyb_spi_obj_t *spi_get_obj_from_handle(SPI_HandleTypeDef *spi);
void spi_set_params(SPI_HandleTypeDef *spi, uint32_t prescale, int32_t baudrate,
    int32_t polarity, int32_t phase, int32_t bits, int32_t firstbit);
void spi_transfer(const struct _pyb_spi_obj_t *self, size_t len, const uint8_t *src, uint8_t *dest, uint32_t timeout);
void spi_print(const mp_print_t *print, SPI_HandleTypeDef *spi, bool legacy);
#endif
