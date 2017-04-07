#include "py/mphal.h"
#include "genhdr/pins.h"

void MOKCAM_board_early_init(void) {
#if 0
    GPIO_InitTypeDef GPIO_InitStructure;

    __GPIOK_CLK_ENABLE();

    // Turn off the backlight. LCD_BL_CTRL = PK3
    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOK, &GPIO_InitStructure);
    HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_RESET);
#endif
#if defined(MICROPY_HW_SPIFLASH_SIZE_BITS)
    // set SPI flash WP and HOLD pins high
    mp_hal_pin_output(&MICROPY_HW_SPIFLASH_HOLD);
    mp_hal_pin_output(&MICROPY_HW_SPIFLASH_WP);
    mp_hal_pin_write(&MICROPY_HW_SPIFLASH_HOLD, 1);
    mp_hal_pin_write(&MICROPY_HW_SPIFLASH_WP, 1);
#endif
#if defined(MICROPY_HW_I2C2_EN)
    mp_hal_pin_output(&MICROPY_HW_I2C2_EN);
    mp_hal_pin_write(&MICROPY_HW_I2C2_EN, 1);
#endif

    #ifdef MICROPY_HW_CC3100_HIB
    //Set HIB high
    /*
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pin = MICROPY_HW_CC3100_HIB.pin_mask;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(MICROPY_HW_CC3100_HIB.gpio, &GPIO_InitStructure);
    GPIO_set_pin(MICROPY_HW_CC3100_HIB.gpio, MICROPY_HW_CC3100_HIB.pin_mask);
    */
    mp_hal_pin_output(&MICROPY_HW_CC3100_HIB);
    mp_hal_pin_low(&MICROPY_HW_CC3100_HIB);
    #endif

}
