#include STM32_HAL_H

#define MICROPY_BOARD_EARLY_INIT    MOKCAM_board_early_init
void MOKCAM_board_early_init(void);

#define MICROPY_HW_BOARD_NAME       "MOKCAM_V3"
#define MICROPY_HW_MCU_NAME         "STM32F7xxI"

#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (1)
#define MICROPY_HW_HAS_SDCARD       (1)
#define MICROPY_HW_HAS_MMA7660      (0)
#define MICROPY_HW_HAS_LIS3DSH      (0)
#define MICROPY_HW_HAS_LCD          (0)
#define MICROPY_HW_HAS_MMI          (0)
#define MICROPY_HW_HAS_USB_FIFO     (1)
#define MICROPY_HW_HAS_WDG          (0)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_TIMER     (1)
#define MICROPY_HW_ENABLE_SERVO     (0)
#define MICROPY_HW_ENABLE_DAC       (1)
#define MICROPY_HW_ENABLE_CAN       (0)

#define MICROPY_HW_HAS_CC3100       (1)

#if 0

#define MICROPY_HW_CC3100_HIB       (pin_A7)
#define MICROPY_HW_CC3100_IRQ       (pin_C5)
#define MICROPY_HW_CC3100_CS        (pin_E11)
#define MICROPY_HW_CC3100_AP        (pin_A1)
#define MICROPY_HW_CC3100_RST       (pin_A3)
#define MICROPY_HW_CC3100_SPI       (SPIHandle4)

#else

//#define MICROPY_HW_CC3100_AP        (pin_E8)      // X1
#define MICROPY_HW_CC3100_RST       (pin_E7)        // X2
#define MICROPY_HW_CC3100_HIB       (pin_D5)        // X3
#define MICROPY_HW_CC3100_IRQ       (pin_B0)        // X4   LCD_EN
#define MICROPY_HW_CC3100_CS        (pin_E11)       // X5
#define MICROPY_HW_CC3100_SPI       (SPIHandle4)

#endif

// use external SPI flash for storage
#if 0
#define MICROPY_HW_SPIFLASH_SIZE_BITS (16 * 1024 * 1024)
#define MICROPY_HW_SPIFLASH_CS      (pin_B6)
#define MICROPY_HW_SPIFLASH_SCK     (pin_B2)
#define MICROPY_HW_SPIFLASH_MOSI    (pin_D11)
#define MICROPY_HW_SPIFLASH_MISO    (pin_D12)
#define MICROPY_HW_SPIFLASH_HOLD    (pin_D13)
#define MICROPY_HW_SPIFLASH_WP      (pin_E2)
#define MICROPY_HW_SPIFLASH_NEED_WBPR   (1)
#define MICROPY_HW_SPIFLASH_QSPI    (1)
#endif
#if 1
#define MICROPY_HW_QSPIFLASH_SIZE_BITS (16 * 1024 * 1024)
#define MICROPY_HW_QSPIFLASH_CS      (pin_B6)
#define MICROPY_HW_QSPIFLASH_SCK     (pin_B2)
#define MICROPY_HW_QSPIFLASH_SIO0    (pin_D11)
#define MICROPY_HW_QSPIFLASH_SIO1    (pin_D12)
#define MICROPY_HW_QSPIFLASH_SIO3    (pin_D13)
#define MICROPY_HW_QSPIFLASH_SIO2    (pin_E2)
#define MICROPY_HW_QSPIFLASH_QSPI    (2)
#define MICROPY_HW_QSPIFLASH_NEED_WBPR   (0)
#define MICROPY_HW_QSPIFLASH_TURBO   (1)
#define MICROPY_HW_QSPI_PRIMARY      (1)
//#define MICROPY_HW_QSPI_QE_MASK      0x40
#define MICROPY_HW_QSPIFLASH_CR      0x0200
#define MICROPY_HW_QSPIFLASH_CR16   (1)
#endif

#define MICROPY_HW_HAS_MRT          (0)
#define MICROPY_HW_HAS_QSPI         (0)

#define MICROPY_HW_ENABLE_DCMI      (1)

//#define MICROPY_MODULE_FROZEN_STR   (0)

// HSE is 16MHz
#define MICROPY_HW_CLK_PLLM (25)
#define MICROPY_HW_CLK_PLLN (432)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ (9)
#define MICROPY_HW_CLK_LAST_FREQ (1)

// The WIM-DSP-32 has a 32kHz crystal for the RTC
#define MICROPY_HW_RTC_USE_LSE      (1)     // TSP_20150827
#define MICROPY_HW_RTC_USE_BYPASS   (1)
#define MICROPY_HW_RTC_USE_EXT_WAKEUP       (1)
#define MICROPY_HW_RTC_USE_US       (1)
#define MICROPY_HW_RTC_USE_CALOUT   (1)
#define RTC_ASYNCH_PREDIV (0x0)
#define RTC_SYNCH_PREDIV  (0x7fff)

// QSPI config
#if MICROPY_HW_HAS_QSPI
#define MICROPY_HW_QSPI_NAME "QSPI"
#define MICROPY_HW_QSPI_CS_GPIO_PORT (GPIOB)
#define MICROPY_HW_QSPI_CLK_GPIO_PORT (GPIOB)
#define MICROPY_HW_QSPI_D0_GPIO_PORT (GPIOD)
#define MICROPY_HW_QSPI_D1_GPIO_PORT (GPIOD)
#define MICROPY_HW_QSPI_D2_GPIO_PORT (GPIOE)
#define MICROPY_HW_QSPI_D3_GPIO_PORT (GPIOD)
#define MICROPY_HW_QSPI_CS_PIN (GPIO_PIN_6)
#define MICROPY_HW_QSPI_CLK_PIN (GPIO_PIN_2)
#define MICROPY_HW_QSPI_D0_PIN (GPIO_PIN_11)
#define MICROPY_HW_QSPI_D1_PIN (GPIO_PIN_12)
#define MICROPY_HW_QSPI_D2_PIN (GPIO_PIN_2)
#define MICROPY_HW_QSPI_D3_PIN (GPIO_PIN_13)
#endif

// From the reference manual, for 2.7V to 3.6V
// 151-180 MHz => 5 wait states
// 181-210 MHz => 6 wait states
// 211-216 MHz => 7 wait states
#define MICROPY_HW_FLASH_LATENCY    FLASH_LATENCY_7 // 210-216 MHz needs 7 wait states

// UART config
#define MICROPY_HW_UART2_TX     (pin_A2)
#define MICROPY_HW_UART2_RX     (pin_A3)

#define MICROPY_HW_UART3_NAME   "WIFI3"
#define MICROPY_HW_UART3_TX     (pin_D8)
#define MICROPY_HW_UART3_RX     (pin_D9)

#define MICROPY_HW_UART7_NAME   "XA"
#define MICROPY_HW_UART7_TX     (pin_E8)
#define MICROPY_HW_UART7_RX     (pin_E7)

#define MICROPY_HW_ENABLE_DUP_REPL  (0)
//#define MICROPY_HW_UART_REPL        PYB_UART_2
#define MICROPY_HW_UART_REPL_BAUD   115200

// I2C busses
#define MICROPY_HW_I2C1_NAME "CAM"
#define MICROPY_HW_I2C1_SCL         (pin_B8)
#define MICROPY_HW_I2C1_SDA         (pin_B9)

#define MICROPY_HW_I2C2_NAME "X"
#define MICROPY_HW_I2C2_EN          (pin_E9)
#define MICROPY_HW_I2C2_SCL         (pin_B10)
#define MICROPY_HW_I2C2_SDA         (pin_B11)


// The STM32F7 uses a TIMINGR register which is configured using an Excel
// Spreadsheet from AN4235: http://www.st.com/web/en/catalog/tools/PF258335
// We use an array of baudrates and corresponding TIMINGR values.
//
// The value 0x40912732 was obtained from the DISCOVERY_I2Cx_TIMING constant
// defined in the STM32F7Cube file Drivers/BSP/STM32F746G-Discovery/stm32f7456g_discovery.h
// SPI
#define MICROPY_HW_SPI2_NAME "X"
#define MICROPY_HW_SPI2_NSS         (pin_B4)
#define MICROPY_HW_SPI2_SCK         (pin_B13)
#define MICROPY_HW_SPI2_MISO        (pin_C2)
#define MICROPY_HW_SPI2_MOSI        (pin_C3)

#define MICROPY_HW_SPI4_NAME "CAM"
#define MICROPY_HW_SPI4_NSS         (pin_E11)
#define MICROPY_HW_SPI4_SCK         (pin_E12)
#define MICROPY_HW_SPI4_MISO        (pin_E13)
#define MICROPY_HW_SPI4_MOSI        (pin_E14)

// USRSW has no pullup or pulldown, and pressing the switch makes the input go low
#define MICROPY_HW_USRSW_PIN        (pin_B3)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)

// LEDs

#define MICROPY_HW_LED1             (pin_A15) //LED_RED
#define MICROPY_HW_LED2             (pin_E3) //LED_GREEN
#define MICROPY_HW_LED3             (pin_D0)  //LED_YELLOW
#define MICROPY_HW_LED4             (pin_D4) //LED_BLUE
//#define MICROPY_HW_LED3_PWM         { TIM2, 2, TIM_CHANNEL_1, GPIO_AF1_TIM2 }
#define MICROPY_HW_LED_OTYPE        (GPIO_MODE_OUTPUT_PP)
#define MICROPY_HW_LED_ON(pin)      (pin->gpio->BSRR = pin->pin_mask)
#define MICROPY_HW_LED_OFF(pin)     (pin->gpio->BSRR = (pin->pin_mask << 16))

// SD card detect switch
#define MICROPY_HW_SDCARD_POWER_PIN         (pin_E9)
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_A0)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)

// USB config (CN13 - USB OTG FS)
// The Hardware VBUS detect only works on pin PA9. The STM32F7 Discovery uses
// PA9 for VCP_TX functionality and connects the VBUS to pin J12 (so software
// only detect). So we don't define the VBUS detect pin since that requires PA9.

/*#define MICROPY_HW_USB_VBUS_DETECT_PIN (pin_J12)*/
//#define MICROPY_HW_USB_OTG_ID_PIN      (pin_A10)
