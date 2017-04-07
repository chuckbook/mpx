//#define TSP_20170314
/*
 * Micro Python CC3100 Driver
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Kimball Johnson
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

#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "py/nlr.h"
#include "py/objtuple.h"
#include "py/objlist.h"
#include "py/stream.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/machine_spi.h"

#include "lib/netutils/netutils.h"
#include "modnetwork.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "dma.h"
#include "irq.h"
#include "spi.h"
#include "netcfg.h"

#include "extint.h"

#include "modnwcc3100.h"
#include "drivers/cc3100-1.0.1.6/include/socket.h"

#define LOG_ERR(str) printf("Error: %s\n",str)
#define LOG_INFO(str) printf("Info: %s\n",str)

#define USE_HARD_SPI (0)
#define IO_TRACE (0)

#if IO_TRACE
#define IO_TRACE_PREFIX(msg) printf("[IO %010u %u] " msg, (uint)mp_hal_ticks_us(), (uint)mp_hal_pin_read(PIN_IRQ))
#else
#define IO_TRACE_PREFIX(msg)
#endif

// *** Begin simplelink interface functions
STATIC volatile irq_handler_t cc3100_IrqHandler = 0;

STATIC SPI_HandleTypeDef *SPI_HANDLE = NULL;
STATIC const pin_obj_t *PIN_CS = NULL;
STATIC const pin_obj_t *PIN_EN = NULL;
STATIC const pin_obj_t *PIN_IRQ = NULL;

#if !USE_HARD_SPI
STATIC mp_machine_soft_spi_obj_t soft_spi;
#endif

// External CC3100
Fd_t spi_Open(char* pIfName, unsigned long flags)
{
    #if USE_HARD_SPI

    //spi_set_params(SPI_HANDLE, -1, 21000000, 0, 0, 8, SPI_FIRSTBIT_MSB);
    spi_set_params(SPI_HANDLE, -1, -1, 0, 0, 8, SPI_FIRSTBIT_MSB);
    SPI_HANDLE->Init.Mode = SPI_MODE_MASTER;
    SPI_HANDLE->Init.Direction = SPI_DIRECTION_2LINES;
    SPI_HANDLE->Init.NSS = SPI_NSS_SOFT;
    SPI_HANDLE->Init.TIMode = SPI_TIMODE_DISABLED;
    SPI_HANDLE->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SPI_HANDLE->Init.CRCPolynomial = 0;
    #if defined(MCU_SERIES_L4) || defined(MCU_SERIES_F7)
    SPI_HANDLE->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    #endif

    // init the SPI bus
    spi_init(SPI_HANDLE, false);

    spi_print(&mp_plat_print, SPI_HANDLE, true);        // TSP_20170309

    #else

    soft_spi.delay_half = MICROPY_PY_MACHINE_SPI_MIN_DELAY;
    soft_spi.polarity = 0;
    soft_spi.phase = 0;
    if (0) {
    #if defined(MICROPY_HW_CC3100_SPI) && defined(MICROPY_HW_SPI1_NAME)
    } else if (&MICROPY_HW_CC3100_SPI == &SPIHandle1) {
        soft_spi.sck = &MICROPY_HW_SPI1_SCK;
        soft_spi.mosi = &MICROPY_HW_SPI1_MOSI;
        soft_spi.miso = &MICROPY_HW_SPI1_MISO;
    #endif
    #if defined(MICROPY_HW_CC3100_SPI) && defined(MICROPY_HW_SPI2_NAME)
    } else if (&MICROPY_HW_CC3100_SPI == &SPIHandle2) {
        soft_spi.sck = &MICROPY_HW_SPI2_SCK;
        soft_spi.mosi = &MICROPY_HW_SPI2_MOSI;
        soft_spi.miso = &MICROPY_HW_SPI2_MISO;
    #endif
    #if defined(MICROPY_HW_CC3100_SPI) && defined(MICROPY_HW_SPI3_NAME)
    } else if (&MICROPY_HW_CC3100_SPI == &SPIHandle3) {
        soft_spi.sck = &MICROPY_HW_SPI3_SCK;
        soft_spi.mosi = &MICROPY_HW_SPI3_MOSI;
        soft_spi.miso = &MICROPY_HW_SPI3_MISO;
    #endif
    #if defined(MICROPY_HW_CC3100_SPI) && defined(MICROPY_HW_SPI4_NAME)
    } else if (&MICROPY_HW_CC3100_SPI == &SPIHandle4) {
        soft_spi.sck = &MICROPY_HW_SPI4_SCK;
        soft_spi.mosi = &MICROPY_HW_SPI4_MOSI;
        soft_spi.miso = &MICROPY_HW_SPI4_MISO;
    #endif
    #if defined(MICROPY_HW_CC3100_SPI) && defined(MICROPY_HW_SPI5_NAME)
    } else if (&MICROPY_HW_CC3100_SPI == &SPIHandle5) {
        soft_spi.sck = &MICROPY_HW_SPI5_SCK;
        soft_spi.mosi = &MICROPY_HW_SPI5_MOSI;
        soft_spi.miso = &MICROPY_HW_SPI5_MISO;
    #endif
    #if defined(MICROPY_HW_CC3100_SPI) && defined(MICROPY_HW_SPI6_NAME)
    } else if (&MICROPY_HW_CC3100_SPI == &SPIHandle6) {
        soft_spi.sck = &MICROPY_HW_SPI6_SCK;
        soft_spi.mosi = &MICROPY_HW_SPI6_MOSI;
        soft_spi.miso = &MICROPY_HW_SPI6_MISO;
    #endif
    } else {
        return 0;
    }
    mp_hal_pin_write(soft_spi.sck, soft_spi.polarity);
    mp_hal_pin_output(soft_spi.sck);
    mp_hal_pin_output(soft_spi.mosi);
    mp_hal_pin_input(soft_spi.miso);

    #endif

    #ifdef MICROPY_HW_CC3100_AP
    // take chip out of reset
    mp_hal_pin_output(&MICROPY_HW_CC3100_AP);
    mp_hal_pin_low(&MICROPY_HW_CC3100_AP);
    #endif
    #ifdef MICROPY_HW_CC3100_RST
    // take chip out of reset
    mp_hal_pin_output(&MICROPY_HW_CC3100_RST);
    mp_hal_pin_high(&MICROPY_HW_CC3100_RST);
    #endif

    // CS Pin
    mp_hal_pin_output(PIN_CS);
    mp_hal_pin_high(PIN_CS);

    HAL_Delay(50);
    return 1;
}

int spi_Close(Fd_t Fd)
{
    #if USE_HARD_SPI
    spi_deinit(SPI_HANDLE);
    #endif
    return 0;
}

int spi_TransmitReceive(unsigned char* txBuff, unsigned char* rxBuff, int Len)
{
    IO_TRACE_PREFIX("SPI ");
#if 1
    if (Len <= 0) {
        //#if IO_TRACE
        printf("len=%d\n", Len);
        //#endif
        return HAL_OK;
    }
#endif
    mp_hal_pin_low(PIN_CS);
    #if USE_HARD_SPI
    if (txBuff == NULL) {
        // It seems that "receive only" SPI transfer does not work, at least
        // not on the STM32L475.  So for such a case we do a full tx+rx, with
        // the tx buffer being the same as the rx buffer (since we don't care
        // what data is actually sent).
//        txBuff = rxBuff;
    }
    spi_transfer(spi_get_obj_from_handle(SPI_HANDLE), Len, txBuff, rxBuff, 0x1000);
    #else
    mp_machine_soft_spi_transfer(&soft_spi.base, Len, txBuff, rxBuff);
    #endif
    mp_hal_pin_high(PIN_CS);

    #if IO_TRACE
    {
        int dir;
        unsigned char *buf;
        if (rxBuff == NULL) {
            dir = 'T';
            buf = txBuff;
        } else {
            dir = 'R';
            buf = rxBuff;
        }
        printf("%cX len=%d buf=", dir, Len);
        for (int i = 0; i < 16 && i < Len; ++i) {
            printf(" %02x", buf[i]);
        }
        printf("\n");
    }
    #endif

    return HAL_OK;
}

#ifdef TSP_20170314   // await IRQ inactive
volatile int in_sync = 0;
#endif

int spi_Read(Fd_t Fd, unsigned char* pBuff, int Len)
{
    HAL_StatusTypeDef status;
#ifdef TSP_20170314   // await IRQ inactive
    if (in_sync) {
        while (mp_hal_pin_read(PIN_IRQ) && in_sync) {
        }
        in_sync = 0;
    }
#endif
    status = spi_TransmitReceive(NULL, pBuff, Len);
    if (status != HAL_OK)
        return(0);

    return Len;
}

int spi_Write(Fd_t Fd, unsigned char* pBuff, int Len)
{
    HAL_StatusTypeDef status;
#ifdef TSP_20170314
    if (Len == 4) {
        uint32_t *p = (uint32_t *)pBuff;
        if (*p == 0x56788765) {
//printf("%s SYNC\n", __func__);
            in_sync = 1;
        }
    }
#endif
    status = spi_TransmitReceive(pBuff, NULL, Len);
    if (status != HAL_OK)
        return(0);

    return Len;
}

void NwpPowerOnPreamble(void){

// IRQ pin
    mp_hal_pin_config(PIN_IRQ, MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_DOWN, 0);

//nHib pin
    mp_hal_pin_output(PIN_EN);
    mp_hal_pin_low(PIN_EN);
}
void NwpPowerOn(void){
    IO_TRACE_PREFIX("HIB=1\n");
    mp_hal_pin_high(PIN_EN);
}
void NwpPowerOff(void){
    IO_TRACE_PREFIX("HIB=0\n");
    mp_hal_pin_low(PIN_EN);
}

_u32 NwpSystemTicks(void)
{
	return HAL_GetTick();
}

STATIC mp_obj_t cc3100_callback(mp_obj_t line) {
    if (cc3100_IrqHandler != 0) {
        IO_TRACE_PREFIX("edge IRQ ");
        #if IO_TRACE
        printf("%p %p\n", cc3100_IrqHandler, line);
        #endif
#ifdef TSP_20170314
        in_sync = 0;
#endif
        (cc3100_IrqHandler)(line);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_callback_obj, cc3100_callback);

int NwpRegisterInterruptHandler(SL_P_EVENT_HANDLER InterruptHdl, void* pValue){
    extint_disable(PIN_IRQ->pin);
    cc3100_IrqHandler = InterruptHdl;
    extint_register((mp_obj_t)PIN_IRQ,
                    GPIO_MODE_IT_RISING,
                    GPIO_PULLDOWN,
                    (mp_obj_t)&cc3100_callback_obj,
                    true);
    extint_enable(PIN_IRQ->pin);
    return 0;
}

void NwpMaskInterrupt(){
    // only needed if IRQ is level-triggered
}

void NwpUnMaskInterrupt(){
    // It's possible that the CC3100 IRQ pin remains high because it still has
    // a pending event to proccess.  One situation when this can happen is when
    // a new event becomes available while doing a read of the previous event.
    // In that case we never get an edge and so must check if the IRQ line is
    // still high, and if so register a callback.
    if (mp_hal_pin_read(PIN_IRQ)) {
        IO_TRACE_PREFIX("level IRQ\n");
        cc3100_callback(MP_OBJ_NEW_SMALL_INT(0));
    }
}

// *** END simplelink interface functions



#define MAKE_SOCKADDR(addr, ip, port) \
    SlSockAddr_t addr; \
    addr.sa_family = SL_AF_INET; \
    addr.sa_data[0] = port >> 8; \
    addr.sa_data[1] = port; \
    addr.sa_data[2] = ip[0]; \
    addr.sa_data[3] = ip[1]; \
    addr.sa_data[4] = ip[2]; \
    addr.sa_data[5] = ip[3];

#define UNPACK_SOCKADDR(addr, ip, port) \
    port = (addr.sa_data[0] << 8) | addr.sa_data[1]; \
    ip[0] = addr.sa_data[2]; \
    ip[1] = addr.sa_data[3]; \
    ip[2] = addr.sa_data[4]; \
    ip[3] = addr.sa_data[5];


#define MAX_RX_PACKET       16000
#define MAX_TX_PACKET       1460

// This structure has the same first 3 entries as a mod_network_socket_obj_t.
// The latter entries are different so we can reuse that memory for our own purposes.
typedef struct _cc3100_socket_obj_t {
    mp_obj_base_t base;
    mp_obj_t nic;
    mod_network_nic_type_t *nic_type;
    int16_t s_fd;
    bool s_nonblocking_connect;
    uint32_t s_timeout;
} cc3100_socket_obj_t;

STATIC int cc3100_socket_settimeout(mod_network_socket_obj_t *socket, mp_uint_t timeout_ms, int *_errno);

STATIC volatile uint32_t fd_closed_state = 0;
STATIC volatile bool wlan_connected = false;
STATIC volatile bool ip_obtained = false;

STATIC int cc3100_get_fd_closed_state(int fd) {
    return fd_closed_state & (1 << fd);
}

STATIC void cc3100_set_fd_closed_state(int fd) {
    fd_closed_state |= 1 << fd;
}

STATIC void cc3100_reset_fd_closed_state(int fd) {
    fd_closed_state &= ~(1 << fd);
}


// Socket functions

// gethostbyname
STATIC int cc3100_gethostbyname(mp_obj_t nic, const char *name, mp_uint_t len, uint8_t *out_ip) {
    uint32_t ip;

    int rc = sl_NetAppDnsGetHostByName((signed char *)name, (uint16_t)len, &ip, SL_AF_INET);
    if (rc != 0) {
        return rc;
    }

    out_ip[0] = ip >> 24;
    out_ip[1] = ip >> 16;
    out_ip[2] = ip >> 8;
    out_ip[3] = ip;

    return 0;
}

STATIC void cc3100_activate(void) {
    if (cc3100_IrqHandler) {
        // Here we assume the driver was already started and just reinitialise the
        // low-level interface.
        mp_hal_pin_config(PIN_IRQ, MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_DOWN, 0);
        mp_hal_pin_output(PIN_EN);
        spi_Open(NULL, 0);
        NwpRegisterInterruptHandler(cc3100_IrqHandler, NULL);
        return;
    }

    wlan_connected = false;
    ip_obtained    = false;

    int32_t ret = -1;

    int32_t mode = sl_Start(NULL,NULL,NULL);

    static const unsigned char defaultcountry[2] = "EU";
    if (sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_COUNTRY_CODE, 2, defaultcountry)) {
        LOG_ERR("failed to set country code!");
    }

    if (mode != ROLE_STA) {
        // Configure the device into station mode
        if (mode == ROLE_AP) {
            LOG_INFO("mode: ROLE_AP");
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while (ip_obtained == false) {
                _SlNonOsMainLoopTask();
            }
        }

        // Select station mode, and restart to activate it
        ret = sl_WlanSetMode(ROLE_STA);
        ret = sl_Stop(100);
        mode = sl_Start(0, 0, 0);
        if (mode != ROLE_STA) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "failed to init CC3100 in STA mode"));
        }
    }

    /* Set connection policy to nothing magic  */
    LOG_INFO("Set connection policy");
    ret = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(0, 0, 0, 0, 0), NULL, 0);

    /* Remove all profiles */
    LOG_INFO("Remove all profiles");
    ret = sl_WlanProfileDel(0xFF);

    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already disconnected
    // Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
    LOG_INFO("Disconnect from AP");
    ret = sl_WlanDisconnect();
    if (ret == 0) {
        // wait for disconnection
        while (wlan_connected == true) {
            _SlNonOsMainLoopTask();
        }
    }

    // Enable DHCP client
    LOG_INFO("Enable DHCP");
    uint8_t val = 1;
    ret = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,(uint8_t *)&val);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set maximum power
    uint8_t power = 0;
    ret = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&power);

    // Set PM policy to normal
    ret = sl_WlanPolicySet(SL_POLICY_PM, SL_NORMAL_POLICY, NULL, 0);

    // Unregister mDNS services
    ret = sl_NetAppMDNSUnRegisterService(0, 0);

    ret = sl_Stop(100);

    // Initializing the CC3100 device
    ret = sl_Start(0, 0, 0);
    if (ret < 0 || ret != ROLE_STA) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "failed to init CC3100"));
    }
}

STATIC void cc3100_deactivate(void) {
    // it doesn't seem we can call sl_Stop if the device is already stopped
    if (cc3100_IrqHandler != NULL) {
        sl_Stop(100);
        // the above call should have cleared the IRQ handler, but we do it
        // anyway because it's used to tell if we are active or not
        cc3100_IrqHandler = NULL;
    }
}

STATIC mp_obj_t cc3100_active(size_t n_args, const mp_obj_t *pos_args) {
    if (n_args == 1) {
        return mp_obj_new_bool(cc3100_IrqHandler != NULL);
    }

    if (mp_obj_is_true(pos_args[1])) {
        cc3100_activate();
    } else {
        cc3100_deactivate();
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cc3100_active_obj, 1, 2, cc3100_active);

// Additional interface functions
// method connect(ssid, key=None, *, security=WPA2, bssid=None, timeout=90)
STATIC mp_obj_t cc3100_connect(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_key, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_security, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA_WPA2)} },
        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_SMALL_INT(90) } },
        { MP_QSTR_eapmethod, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_TTLS_MSCHAPv2)} },
        { MP_QSTR_username,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_anonname,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get ssid
    mp_uint_t ssid_len;
    const char *ssid = mp_obj_str_get_data(args[0].u_obj, &ssid_len);

    // get key and sec
    mp_uint_t key_len = 0;
    const char *key = NULL;
    mp_uint_t sec = SL_SEC_TYPE_OPEN;
    if (args[1].u_obj != mp_const_none) {
        key = mp_obj_str_get_data(args[1].u_obj, &key_len);

        if (!MP_OBJ_IS_INT(args[2].u_obj))
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "invalid 'security' parameter\n"));
        sec = mp_obj_get_int(args[2].u_obj);
    }

    // get bssid
    const char *bssid = NULL;
    if (args[3].u_obj != mp_const_none) {
        bssid = mp_obj_str_get_str(args[3].u_obj);
    }

    mp_int_t timeout = -1;
    if (MP_OBJ_IS_INT(args[4].u_obj)) {
      timeout = mp_obj_get_int(args[4].u_obj) * 1000;
    }

    SlSecParams_t sec_params;
    sec_params.Type = sec;
    sec_params.Key = (int8_t*)key;
    sec_params.KeyLen = key_len;

    SlSecParamsExt_t sec_ext_params, *use_ext = NULL;
    if (sec == SL_SEC_TYPE_WPA_ENT) {
        mp_uint_t len;

        memset(&sec_ext_params, 0, sizeof(sec_ext_params));
        if (!MP_OBJ_IS_INT(args[5].u_obj))
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "invalid 'eapmethod' parameter\n"));
        sec_ext_params.EapMethod = mp_obj_get_int(args[5].u_obj);
        if (args[6].u_obj != mp_const_none) {
            sec_ext_params.User = (_i8*)mp_obj_str_get_data(args[6].u_obj, &len);
            sec_ext_params.UserLen = len;
        }
        if (args[7].u_obj != mp_const_none) {
            sec_ext_params.AnonUser = (_i8*)mp_obj_str_get_data(args[7].u_obj, &len);
            sec_ext_params.AnonUserLen = len;
        }
        use_ext = &sec_ext_params;
    }

    // connect to AP
    printf("Connect to AP\n");
    if (sl_WlanConnect((int8_t*)ssid, ssid_len, (uint8_t*)bssid, &sec_params, use_ext)!= 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError,
          "could not connect to ssid=%s, sec=%d, key=%s\n", ssid, sec, key));
    }

    if (timeout >= 0) {
      // Wait until connected or timeout, calling simplelink loop
      uint32_t start = HAL_GetTick();
      while (!(ip_obtained && wlan_connected) && ((HAL_GetTick() - start) < timeout) ){
        _SlNonOsMainLoopTask();
      }
      if (!wlan_connected || !ip_obtained) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError,
          "timed out connecting to ssid=%s, sec=%d, key=%s\n", ssid, sec, key));
      }
      sl_WlanRxStatStart();
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(cc3100_connect_obj, 1, cc3100_connect);

STATIC mp_obj_t cc3100_disconnect(mp_obj_t self_in) {
    sl_WlanRxStatStop(); 
    sl_WlanDisconnect();
    while ((wlan_connected)){
      _SlNonOsMainLoopTask();
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_disconnect_obj, cc3100_disconnect);

STATIC mp_obj_t cc3100_isconnected(mp_obj_t self_in) {
    return mp_obj_new_bool(wlan_connected && ip_obtained);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_isconnected_obj, cc3100_isconnected);

STATIC mp_obj_t cc3100_ifconfig(mp_obj_t self_in) {
    unsigned char iplen = sizeof(SlNetCfgIpV4Args_t);
    unsigned char dhcpIsOn = 0;
    SlNetCfgIpV4Args_t ipV4 = {0};
    // Ignore the return value of the next call.  It seems that it can return non-zero
    // values even if it succeeds.  Examples of such values are: 0xff52, 0xff91.
    sl_NetCfgGet(SL_IPV4_STA_P2P_CL_GET_INFO, &dhcpIsOn, &iplen, (unsigned char *)&ipV4);
    mp_obj_t tuple[4];
    tuple[0] = netutils_format_ipv4_addr((uint8_t*)&ipV4.ipV4, NETUTILS_LITTLE);
    tuple[1] = netutils_format_ipv4_addr((uint8_t*)&ipV4.ipV4Mask, NETUTILS_LITTLE);
    tuple[2] = netutils_format_ipv4_addr((uint8_t*)&ipV4.ipV4Gateway, NETUTILS_LITTLE);
    tuple[3] = netutils_format_ipv4_addr((uint8_t*)&ipV4.ipV4DnsServer, NETUTILS_LITTLE);
    return mp_obj_new_tuple(MP_ARRAY_SIZE(tuple), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_ifconfig_obj, cc3100_ifconfig);

STATIC mp_obj_t cc3100_update(mp_obj_t self_in) {

    _SlNonOsMainLoopTask();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_update_obj, cc3100_update);

STATIC mp_obj_t cc3100_sleep(mp_obj_t self_in) {

    sl_Stop(100);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_sleep_obj, cc3100_sleep);

STATIC mp_obj_t cc3100_scan(mp_obj_t self_in) {
  /* Enable scan */ //TODO: check if already enabled
  #define SL_SCAN_ENABLE 1 
  int retVal;
  uint8_t configOpt = SL_SCAN_POLICY(1);
  uint8_t configVal = 60;
  LOG_INFO("Enable Scan");
  retVal = sl_WlanPolicySet(SL_POLICY_SCAN, configOpt, &configVal, sizeof(configVal));
  mp_hal_delay_ms(1000); // Wait 1 second to ensure scan starts
  
  int runningIdx, numOfEntries, idx;
  Sl_WlanNetworkEntry_t netentry = {0};
  Sl_WlanNetworkEntry_t netEntries[20];
  memset(netEntries, 0, sizeof(netEntries));
  
  numOfEntries = 20;
  runningIdx = 0;
  idx = 0;
  retVal = sl_WlanGetNetworkList(runningIdx,numOfEntries,&netEntries[runningIdx]);
  
  /*
   * Because of a bug user should either read the maximum entries or read
   * entries one by one from the end and check for duplicates. Once a duplicate
   * is found process should be stopped.
   */
  /* get scan results - one by one */
  runningIdx = 20;
  numOfEntries = 1;
  memset(netEntries, 0, sizeof(netEntries));
  
  do
  { 
    runningIdx--;
    retVal = sl_WlanGetNetworkList(runningIdx, numOfEntries, &netentry);
    if(retVal < numOfEntries)
      nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error getting AP list"));

    if(idx > 0)
    {
        if(0 == memcmp(netentry.bssid,
                  netEntries[idx - 1].bssid, SL_BSSID_LENGTH))
        {
            /* Duplicate entry */
            break;
        }
    }

    memcpy(&netEntries[idx], &netentry, sizeof(Sl_WlanNetworkEntry_t));
    idx++;

} while (runningIdx > 0);
  mp_obj_t returnVal = mp_obj_new_list(0, NULL);
  for(int i = 0; i < idx; i++) {
    mp_obj_t tuple[5];
    tuple[0] = mp_obj_new_str((const char*)netEntries[i].ssid, netEntries[i].ssid_len, false);
    tuple[1] = mp_obj_new_bytes(netEntries[i].bssid, SL_BSSID_LENGTH);
    tuple[2] = MP_OBJ_NEW_SMALL_INT(0); // channel not available on CC3100
    tuple[3] = MP_OBJ_NEW_SMALL_INT(netEntries[i].rssi);
    tuple[4] = MP_OBJ_NEW_SMALL_INT(netEntries[i].sec_type);
    mp_obj_list_append(returnVal, mp_obj_new_tuple(5, tuple));
  }
  return returnVal;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_scan_obj, cc3100_scan);

STATIC mp_obj_t cc3100_get_rssi(mp_obj_t self_in) {
  SlGetRxStatResponse_t rxStatResp;
  _i16 avgRssi, retVal;
  retVal = sl_WlanRxStatGet(&rxStatResp, 0);
  if (retVal == 0) {
    avgRssi = rxStatResp.AvarageMgMntRssi;
    return MP_OBJ_NEW_SMALL_INT(avgRssi);
  }
  nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error getting RSSI"));
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_get_rssi_obj, cc3100_get_rssi);

STATIC mp_obj_t cc3100_settime(mp_obj_t self_in, mp_obj_t tuple) {
  mp_uint_t len;
  mp_obj_t *elem;
  _i32 retVal = -1;
  SlDateTime_t dateTime= {0};

  mp_obj_get_array(tuple, &len, &elem);

  // localtime generates a tuple of len 8. CPython uses 9, so we accept both, also we only need 6, so thats fine too.
  if (len < 6 || len > 9) {
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "CC3100.settime needs a tuple of length 6-9 (%d given)", len));
  }

  dateTime.sl_tm_day = mp_obj_get_int(elem[2]);
  dateTime.sl_tm_mon = mp_obj_get_int(elem[1]);
  dateTime.sl_tm_year = mp_obj_get_int(elem[0]);
  dateTime.sl_tm_hour = mp_obj_get_int(elem[3]);
  dateTime.sl_tm_min = mp_obj_get_int(elem[4]);
  dateTime.sl_tm_sec = mp_obj_get_int(elem[5]);

  retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
        sizeof(SlDateTime_t),(_u8 *)(&dateTime));
  if (retVal != 0)
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Error setting time"));

  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc3100_settime_obj, cc3100_settime);

STATIC mp_obj_t cc3100_setcountry(mp_obj_t self_in, mp_obj_t cc) {
  _i32 retVal = -1;
  mp_uint_t country_len;
  const unsigned char *country = (const unsigned char *)mp_obj_str_get_data(cc, &country_len);

  if (!country || country_len != 2)
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Invalid country code given"));

  retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_COUNTRY_CODE, 2, country);
  if (retVal != 0)
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Error %d setting country code", retVal));

  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc3100_setcountry_obj, cc3100_setcountry);

STATIC mp_obj_t cc3100_file_open(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
  static const mp_arg_t allowed_args[] = {
      { MP_QSTR_filename, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
      { MP_QSTR_mode,  MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = FS_MODE_OPEN_READ} },
      { MP_QSTR_size,  MP_ARG_INT, {0} },
  };

  // parse args
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
  
  _i32 retVal;
  _i32 pFileHandle = 0;
  _i32 mode = args[1].u_int;
  _i32 size = args[2].u_int;
  
  if (mode == MP_SL_FS_CREATE && size <= 0) {
      // Create with no size
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Create mode needs a valid size"));
  }
  // TODO: check max size
  
  _u32 cc_fs_mode;
  switch(mode){
    case MP_SL_FS_READ:
      cc_fs_mode = FS_MODE_OPEN_READ;
      break;
    case MP_SL_FS_WRITE:
      cc_fs_mode = FS_MODE_OPEN_WRITE;
      break;
    case MP_SL_FS_CREATE:
      cc_fs_mode = FS_MODE_OPEN_CREATE(size, 0); // No special flags
      break;
    default: 
      nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_TypeError, "Invalid Mode"));
      break;
  }
  
  retVal = sl_FsOpen((const uint8_t*)mp_obj_str_get_str(args[0].u_obj),cc_fs_mode,NULL,&pFileHandle);
  if (!retVal == 0)
  {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error opening file: %d", retVal));
  }
  return mp_obj_new_int_from_uint(pFileHandle);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(cc3100_file_open_obj, 1, cc3100_file_open);


STATIC mp_obj_t cc3100_file_close(mp_obj_t self_in, mp_obj_t fh) {
  _i32 retVal;
  _i32 pFileHandle = mp_obj_get_int(fh);
  retVal = sl_FsClose(pFileHandle,0,0,0);
  if (!retVal == 0)
  {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error closing file: %d", retVal));
  }
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc3100_file_close_obj, cc3100_file_close);


STATIC mp_obj_t cc3100_file_write(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
  static const mp_arg_t allowed_args[] = {
      { MP_QSTR_fh, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
      { MP_QSTR_offset,  MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
      { MP_QSTR_data,  MP_ARG_REQUIRED | MP_ARG_OBJ, {0} },
  };

  // parse args
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);


  mp_buffer_info_t bufinfo;
  mp_get_buffer_raise(args[2].u_obj, &bufinfo, MP_BUFFER_READ);
  _i32 retVal;
  _i32 fileHandle = args[0].u_int;
  _i32 offset = args[1].u_int;
  retVal = sl_FsWrite(fileHandle, offset, (_u8 *)bufinfo.buf, bufinfo.len);
  if (retVal <= 0)
  {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error writing to file: %d", retVal));
  }  
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(cc3100_file_write_obj, 1,cc3100_file_write);


STATIC mp_obj_t cc3100_file_read(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
  static const mp_arg_t allowed_args[] = {
      { MP_QSTR_fh,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
      { MP_QSTR_offset, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
      { MP_QSTR_len,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 1024} },
  };
  unsigned char buf[1024];

  // parse args
  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
  mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

  _i32 fileHandle = args[0].u_int;
  _u32 offset = args[1].u_int;
  _u32 len = args[2].u_int;
  _i32 retVal;

  if (len > sizeof(buf))
    len = sizeof(buf);

  retVal = sl_FsRead(fileHandle, offset, buf, len);
  if (retVal <= 0)
  {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error reading from file: %d", retVal));
  }
  return mp_obj_new_bytearray(retVal, buf);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(cc3100_file_read_obj, 1,cc3100_file_read);


STATIC mp_obj_t cc3100_file_del(mp_obj_t self, mp_obj_t nameobj) {
  _i32 retVal;
  unsigned char *name = (unsigned char *)mp_obj_str_get_str(nameobj);

  retVal = sl_FsDel(name, 0);
  if (!retVal == 0)
  {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error deleting file: %d", retVal));
  }
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc3100_file_del_obj, cc3100_file_del);


STATIC mp_obj_t cc3100_fw_open(mp_obj_t self_in) {
  _i32 retVal;
  _i32 pFileHandle = 0;
  _u32 token;
  
  retVal = sl_FsOpen((const uint8_t*)"/sys/servicepack.ucf", FS_MODE_OPEN_CREATE(131072,_FS_FILE_OPEN_FLAG_SECURE|_FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE), &token, &pFileHandle);
  
  if (!retVal == 0)
  {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error opening file: %d", retVal));
  }
  return mp_obj_new_int_from_uint(pFileHandle);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_fw_open_obj, cc3100_fw_open);

STATIC mp_obj_t cc3100_fw_close(mp_obj_t self_in, mp_obj_t fh, mp_obj_t sig) {
  _i32 retVal;
  _i32 pFileHandle = mp_obj_get_int(fh);
  
  mp_buffer_info_t sigbufinfo;
  mp_get_buffer_raise(sig, &sigbufinfo, MP_BUFFER_READ);
  
  retVal = sl_FsClose(pFileHandle,0,(_u8 *)sigbufinfo.buf, sigbufinfo.len);
  if (!retVal == 0)
  {
    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error closing file: %d", retVal));
  }
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(cc3100_fw_close_obj, cc3100_fw_close);

STATIC mp_obj_t cc3100_get_mac(mp_obj_t self_in) {
  _i32 retVal;
  unsigned char dummy;
  unsigned char mac[6];
  unsigned char len = sizeof(mac);

  retVal = sl_NetCfgGet(SL_MAC_ADDRESS_GET, &dummy, &len, mac);
  if (retVal == 0) {
    return mp_obj_new_bytearray(sizeof(mac), mac);
  }
  nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error %d getting MAC address", retVal));
  return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_get_mac_obj, cc3100_get_mac);

STATIC mp_obj_t cc3100_version(mp_obj_t self_in) {
    int32_t retVal = -1;
    uint8_t pConfigOpt;
    uint8_t pConfigLen;
    SlVersionFull ver;

    pConfigLen = sizeof(ver);
    pConfigOpt = SL_DEVICE_GENERAL_VERSION;
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &pConfigOpt, &pConfigLen, (_u8 *)(&ver));
 
    if (retVal >= 0) {
        STATIC const qstr version_fields[] = {
            MP_QSTR_chip, MP_QSTR_mac, MP_QSTR_phy, MP_QSTR_nwp, MP_QSTR_rom, MP_QSTR_host
        };
        mp_obj_t tuple[6];
        tuple[0] = mp_obj_new_int_from_uint(ver.ChipFwAndPhyVersion.ChipId);
        
        vstr_t vstr;
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "31.%lu.%lu.%lu.%lu", ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3]);
        tuple[1] = mp_obj_new_str(vstr.buf, vstr.len, false);
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "%d.%d.%d.%d", ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);
        tuple[2] = mp_obj_new_str(vstr.buf, vstr.len, false);
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "%lu.%lu.%lu.%lu", ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3]);
        tuple[3] = mp_obj_new_str(vstr.buf, vstr.len, false);
        tuple[4] = mp_obj_new_int_from_uint(ver.RomVersion);
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "%lu.%lu.%lu.%lu", SL_MAJOR_VERSION_NUM,SL_MINOR_VERSION_NUM,SL_VERSION_NUM,SL_SUB_VERSION_NUM);
        tuple[5] = mp_obj_new_str(vstr.buf, vstr.len, false);

        return mp_obj_new_attrtuple(version_fields, 6, tuple);
    } 
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3100_version_obj, cc3100_version);

/******************************************************************************/
// Micro Python bindings; CC3100 class

typedef struct _cc3100_obj_t {
    mp_obj_base_t base;
} cc3100_obj_t;

STATIC const cc3100_obj_t cc3100_obj = {{(mp_obj_type_t*)&mod_network_nic_type_cc3100}};

STATIC mp_obj_t cc3100_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {

    // Either defaults, or SPI Obj, IRQ Pin, nHIB Pin
    mp_arg_check_num(n_args, n_kw, 0, 4, false);

    //If no args given, we setup from coded defaults
    if (n_args == 0) {
#ifdef MICROPY_HW_CC3100_SPI
      SPI_HANDLE = &MICROPY_HW_CC3100_SPI;
      PIN_CS = &MICROPY_HW_CC3100_CS;
      PIN_EN = &MICROPY_HW_CC3100_HIB;
      PIN_IRQ = &MICROPY_HW_CC3100_IRQ;
#else
      nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "No Default CC3100 definition"));
#endif
    } else {
    //Else we use the given args
       //TODO should verify the argument types
       SPI_HANDLE = spi_get_handle(args[0]);
       PIN_CS = pin_find(args[1]);
       PIN_EN = pin_find(args[2]);
       PIN_IRQ = pin_find(args[3]);
    }

    cc3100_activate();

    // register with network module
    mod_network_register_nic((mp_obj_t)&cc3100_obj);
    return (mp_obj_t)&cc3100_obj;

}

STATIC const mp_map_elem_t cc3100_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_active),          (mp_obj_t)&cc3100_active_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_connect),         (mp_obj_t)&cc3100_connect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_disconnect),      (mp_obj_t)&cc3100_disconnect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_isconnected),    (mp_obj_t)&cc3100_isconnected_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ifconfig),        (mp_obj_t)&cc3100_ifconfig_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_update),          (mp_obj_t)&cc3100_update_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sleep),           (mp_obj_t)&cc3100_sleep_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_scan),            (mp_obj_t)&cc3100_scan_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_rssi),        (mp_obj_t)&cc3100_get_rssi_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_settime),         (mp_obj_t)&cc3100_settime_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_setcountry),      (mp_obj_t)&cc3100_setcountry_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_file_open),       (mp_obj_t)&cc3100_file_open_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_file_close),      (mp_obj_t)&cc3100_file_close_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_file_write),      (mp_obj_t)&cc3100_file_write_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_file_read),       (mp_obj_t)&cc3100_file_read_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_file_del),        (mp_obj_t)&cc3100_file_del_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_fw_open),         (mp_obj_t)&cc3100_fw_open_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_fw_close),        (mp_obj_t)&cc3100_fw_close_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_mac),         (mp_obj_t)&cc3100_get_mac_obj}, 
    { MP_OBJ_NEW_QSTR(MP_QSTR_version),         (mp_obj_t)&cc3100_version_obj}, 

    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_OPEN), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_OPEN) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WEP), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA2), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA_WPA2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA_ENT), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA_ENT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WEP), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WPA), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WPA2), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA_WPA2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WPA_ENT), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA_ENT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_OPEN), MP_OBJ_NEW_SMALL_INT(SL_SCAN_SEC_TYPE_OPEN) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_WEP), MP_OBJ_NEW_SMALL_INT(SL_SCAN_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_WPA), MP_OBJ_NEW_SMALL_INT(SL_SCAN_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_WPA2), MP_OBJ_NEW_SMALL_INT(SL_SCAN_SEC_TYPE_WPA2) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_TLS),                      MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_TLS) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_TTLS_TLS),                 MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_TTLS_TLS) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_TTLS_MSCHAPv2),            MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_TTLS_MSCHAPv2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_TTLS_PSK),                 MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_TTLS_PSK) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_PEAP0_TLS),                MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_PEAP0_TLS) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_PEAP0_MSCHAPv2),           MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_PEAP0_MSCHAPv2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_PEAP0_PSK),                MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_PEAP0_PSK) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_PEAP1_TLS),                MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_PEAP1_TLS) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_PEAP1_MSCHAPv2),           MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_PEAP1_MSCHAPv2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_PEAP1_PSK),                MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_PEAP1_PSK) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_FAST_AUTH_PROVISIONING),   MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_FAST_AUTH_PROVISIONING) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_FAST_UNAUTH_PROVISIONING), MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_FAST_UNAUTH_PROVISIONING) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_EAP_METHOD_FAST_NO_PROVISIONING),     MP_OBJ_NEW_SMALL_INT(SL_ENT_EAP_METHOD_FAST_NO_PROVISIONING) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_SOL_SOCKET), MP_OBJ_NEW_SMALL_INT(SL_SOL_SOCKET)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_RCVBUF), MP_OBJ_NEW_SMALL_INT(SL_SO_RCVBUF)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_KEEPALIVE) , MP_OBJ_NEW_SMALL_INT(SL_SO_KEEPALIVE )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_RCVTIME0), MP_OBJ_NEW_SMALL_INT(SL_SO_RCVTIMEO)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_NONBLOCKING), MP_OBJ_NEW_SMALL_INT(SL_SO_NONBLOCKING)},

    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SECMETHOD), MP_OBJ_NEW_SMALL_INT(SL_SO_SECMETHOD )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SEC_METHOD_SSLV3), MP_OBJ_NEW_SMALL_INT(SL_SO_SEC_METHOD_SSLV3 )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SEC_METHOD_TLSV1), MP_OBJ_NEW_SMALL_INT(SL_SO_SEC_METHOD_TLSV1 )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SEC_METHOD_TLSV1_1), MP_OBJ_NEW_SMALL_INT(SL_SO_SEC_METHOD_TLSV1_1 )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SEC_METHOD_TLSV1_2), MP_OBJ_NEW_SMALL_INT(SL_SO_SEC_METHOD_TLSV1_2 )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SEC_METHOD_SSLv3_TLSV1_2), MP_OBJ_NEW_SMALL_INT(SL_SO_SEC_METHOD_SSLv3_TLSV1_2 )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SEC_METHOD_DLSV1), MP_OBJ_NEW_SMALL_INT(SL_SO_SEC_METHOD_DLSV1 )},


    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SECURE_MASK), MP_OBJ_NEW_SMALL_INT(SL_SO_SECURE_MASK)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256), MP_OBJ_NEW_SMALL_INT(SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256)},

    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_CHANGE_CHANNEL), MP_OBJ_NEW_SMALL_INT(SL_SO_CHANGE_CHANNEL)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME), MP_OBJ_NEW_SMALL_INT(SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SECURE_FILES_CERTIFICATE_FILE_NAME), MP_OBJ_NEW_SMALL_INT(SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SECURE_FILES_CA_FILE_NAME), MP_OBJ_NEW_SMALL_INT(SL_SO_SECURE_FILES_CA_FILE_NAME)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_SECURE_FILES_DH_KEY_FILE_NAME), MP_OBJ_NEW_SMALL_INT(SL_SO_SECURE_FILES_DH_KEY_FILE_NAME)},

    { MP_OBJ_NEW_QSTR(MP_QSTR_FILE_MODE_READ), MP_OBJ_NEW_SMALL_INT(MP_SL_FS_READ)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_FILE_MODE_WRITE), MP_OBJ_NEW_SMALL_INT(MP_SL_FS_WRITE)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_FILE_MODE_CREATE), MP_OBJ_NEW_SMALL_INT(MP_SL_FS_CREATE)},

};

STATIC MP_DEFINE_CONST_DICT(cc3100_locals_dict, cc3100_locals_dict_table);

STATIC int cc3100_socket_socket(mod_network_socket_obj_t *socket_in, int *_errno) {
    if (socket_in->u_param.domain != MOD_NETWORK_AF_INET) {
        *_errno = MP_EAFNOSUPPORT;
        return -1;
    }

    mp_uint_t type;
    mp_uint_t proto = 0;
    switch (socket_in->u_param.type) {
        case MOD_NETWORK_SOCK_STREAM: type = SL_SOCK_STREAM; break;
        case MOD_NETWORK_SOCK_DGRAM: type = SL_SOCK_DGRAM; break;
        case MOD_NETWORK_SOCK_RAW: type = SL_SOCK_RAW; break;
        default: *_errno = MP_EINVAL; return -1;
    }

    /* TODO use ussl module
    if (socket_in->u_param.proto == MOD_NETWORK_SEC_SOCKET)
    {
        // SSL Socket
        if (socket_in->u_param.type != MOD_NETWORK_SOCK_STREAM ){
          *_errno = MP_EINVAL; return -1; // Only support TCP SSL
        }
        // To start we will setup ssl sockets ignoring certificates
        proto = SL_SEC_SOCKET;
    }
    */

    // open socket
    int fd = sl_Socket(SL_AF_INET, type, proto);
    if (fd < 0) {
        *_errno = -fd;
        return -1;
    }

    // clear socket state
    cc3100_reset_fd_closed_state(fd);

    // get the timeout that we need to configure the socket to
    uint32_t timeout = socket_in->u_param.timeout;

    // re-cast the socket object to use our custom fields
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;

    // store state of this socket
    socket->s_fd = fd;
    socket->s_timeout = timeout;
    socket->s_nonblocking_connect = false;

    // configure the timeout
    if (cc3100_socket_settimeout(socket_in, timeout, _errno) != 0) {
        return -1;
    }

    return 0;
}

STATIC void cc3100_socket_close(mod_network_socket_obj_t *socket_in) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    if (!cc3100_get_fd_closed_state(socket->s_fd)) {
        sl_Close(socket->s_fd);
        cc3100_set_fd_closed_state(socket->s_fd);
    }
}

STATIC int cc3100_socket_bind(mod_network_socket_obj_t *socket_in, byte *ip, mp_uint_t port, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    MAKE_SOCKADDR(addr, ip, port)
    int ret = sl_Bind(socket->s_fd, &addr, sizeof(addr));
    if (ret != 0) {
        *_errno = -ret;
        return -1;
    }
    return 0;
}

STATIC int cc3100_socket_listen(mod_network_socket_obj_t *socket_in, mp_int_t backlog, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    int ret = sl_Listen(socket->s_fd, backlog);
    if (ret != 0) {
        *_errno = -ret;
        return -1;
    }
    return 0;
}

STATIC int cc3100_socket_accept(mod_network_socket_obj_t *socket_in, mod_network_socket_obj_t *socket2_in, byte *ip, mp_uint_t *port, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    cc3100_socket_obj_t *socket2 = (cc3100_socket_obj_t*)socket2_in;

    // accept incoming connection
    int fd;
    SlSockAddr_t addr;
    SlSocklen_t addr_len = sizeof(addr);
    if ((fd = sl_Accept(socket->s_fd, &addr, &addr_len)) < 0) {
        if (fd == SL_EAGAIN) {
            *_errno = MP_EAGAIN;
        } else {
            *_errno = -fd;
        }
        return -1;
    }

    // clear socket state
    cc3100_reset_fd_closed_state(fd);

    // store state in new socket object
    socket2->s_fd = fd;
    socket2->s_timeout = -1; // TODO inherit timeout value?
    socket2->s_nonblocking_connect = false;

    // return ip and port
    UNPACK_SOCKADDR(addr, ip, *port);

    return 0;
}

STATIC int cc3100_socket_connect(mod_network_socket_obj_t *socket_in, byte *ip, mp_uint_t port, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;

    if (cc3100_get_fd_closed_state(socket->s_fd)) {
        cc3100_socket_socket(socket_in, _errno); // Socket has been closed, we need to recreate it
    }

    MAKE_SOCKADDR(addr, ip, port)
    int ret = sl_Connect(socket->s_fd, &addr, sizeof(addr));
    if (ret != 0 && ret != SL_ESECSNOVERIFY) {
        if (ret == SL_EALREADY && socket->s_timeout == 0) {
            // For a non-blocking connect the CC3100 will return EALREADY the
            // first time.  Calls to sl_Select for writing can be used to poll
            // whether the connection is completed and then sl_Connect should be
            // called once more to finish the connection.  To match BSD we return
            // EINPROGRESS here and set a flag to indicate the connection is in
            // progress.
            socket->s_nonblocking_connect = true;
            *_errno = MP_EINPROGRESS;
        } else {
            *_errno = -ret;
        }
        return -1;
    }
    return 0;
}

STATIC mp_uint_t cc3100_socket_send(mod_network_socket_obj_t *socket_in, const byte *buf, mp_uint_t len, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;

    if (cc3100_get_fd_closed_state(socket->s_fd)) {
        sl_Close(socket->s_fd);
        *_errno = MP_EPIPE;
        return -1;
    }

    // CC3100 does not handle fragmentation, and will overflow,
    // split the packet into smaller ones and send them out.
    mp_int_t bytes = 0;
    while (bytes < len) {
        int n = MIN((len - bytes), MAX_TX_PACKET);
        n = sl_Send(socket->s_fd, (uint8_t*)buf + bytes, n, 0);
        if (n <= 0) {
            *_errno = -n;
            return -1;
        }
        bytes += n;
    }

    return bytes;
}

STATIC mp_uint_t cc3100_socket_recv(mod_network_socket_obj_t *socket_in, byte *buf, mp_uint_t len, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;

    // check the socket is open
    if (cc3100_get_fd_closed_state(socket->s_fd)) {
        // socket is closed, but CC3100 may have some data remaining in buffer, so check
        SlFdSet_t rfds;
        SL_FD_ZERO(&rfds);
        SL_FD_SET(socket->s_fd, &rfds);
        SlTimeval_t tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        int nfds = sl_Select(socket->s_fd + 1, &rfds, NULL, NULL, &tv);
        if (nfds == -1 || !SL_FD_ISSET(socket->s_fd, &rfds)) {
            // no data waiting, so close socket and return 0 data
            sl_Close(socket->s_fd);
            return 0;
        }
    }

    // cap length at MAX_RX_PACKET
    len = MIN(len, MAX_RX_PACKET);

    // do the recv
    int ret = sl_Recv(socket->s_fd, buf, len, 0);
    if (ret < 0) {
        *_errno = -ret;
        return -1;
    }

    return ret;
}

STATIC mp_uint_t cc3100_socket_sendto(mod_network_socket_obj_t *socket_in, const byte *buf, mp_uint_t len, byte *ip, mp_uint_t port, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    MAKE_SOCKADDR(addr, ip, port)
    int ret = sl_SendTo(socket->s_fd, (byte*)buf, len, 0, (SlSockAddr_t*)&addr, sizeof(addr));
    if (ret < 0) {
        *_errno = -ret;
        return -1;
    }
    return ret;
}

STATIC mp_uint_t cc3100_socket_recvfrom(mod_network_socket_obj_t *socket_in, byte *buf, mp_uint_t len, byte *ip, mp_uint_t *port, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    SlSockAddr_t addr;
    SlSocklen_t addr_len = sizeof(addr);
    mp_int_t ret = sl_RecvFrom(socket->s_fd, buf, len, 0, &addr, &addr_len);
    if (ret < 0) {
        *_errno = -ret;
        return -1;
    }
    UNPACK_SOCKADDR(addr, ip, *port);
    return ret;
}

STATIC int cc3100_socket_setsockopt(mod_network_socket_obj_t *socket_in, mp_uint_t level, mp_uint_t opt, const void *optval, mp_uint_t optlen, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;

    if (level == MOD_NETWORK_SOL_SOCKET && opt == MOD_NETWORK_SO_REUSEADDR) {
        // It seems that CC3100 always has the behaviour of SO_REUSEADDR
        return 0;
    }

  int ret;
  // Todo : Review and Clean this up
  if (opt == SL_SO_SECMETHOD) {
    SlSockSecureMethod method;
    method.secureMethod = (unsigned int) *(unsigned int *)optval;
    ret = sl_SetSockOpt(socket->s_fd, level, opt, (_u8 *) &method, sizeof(method));
  } else {
    ret = sl_SetSockOpt(socket->s_fd, level, opt, optval, optlen);
  }

  if (ret < 0) {
    *_errno = -ret;
      return -1;
  }
  return 0;
}

STATIC int cc3100_socket_settimeout(mod_network_socket_obj_t *socket_in, mp_uint_t timeout_ms, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    int ret;
    if (timeout_ms == 0 || timeout_ms == -1) {
        SlSockNonblocking_t optval;
        SlSocklen_t optlen = sizeof(optval);
        if (timeout_ms == 0) {
            // set non-blocking mode
            optval.NonblockingEnabled = 1;
        } else {
            // set blocking mode
            optval.NonblockingEnabled = 0;
        }
        ret = sl_SetSockOpt(socket->s_fd, SL_SOL_SOCKET, SL_SO_NONBLOCKING , &optval, optlen);

    } else {
        // set timeout
        SlTimeval_t timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = timeout_ms % 1000;
        SlSocklen_t optlen = sizeof(timeout);
        ret = sl_SetSockOpt(socket->s_fd, SL_SOL_SOCKET, SL_SO_RCVTIMEO , &timeout, optlen);
    }

    if (ret != 0) {
        *_errno = -ret;
        return -1;
    }

    return 0;
}

STATIC int cc3100_socket_ioctl(mod_network_socket_obj_t *socket_in, mp_uint_t request, mp_uint_t arg, int *_errno) {
    cc3100_socket_obj_t *socket = (cc3100_socket_obj_t*)socket_in;
    mp_uint_t ret;
    if (request == MP_STREAM_POLL) {
        mp_uint_t flags = arg;
        ret = 0;
        int fd = socket->s_fd;

        // init fds
        SlFdSet_t rfds, wfds;
        SL_FD_ZERO(&rfds);
        SL_FD_ZERO(&wfds);

        // set fds if needed
        if (flags & MP_STREAM_POLL_RD) {
            SL_FD_SET(fd, &rfds);

            // A socked that just closed is available for reading.  A call to
            // recv() returns 0 which is consistent with BSD.
            if (cc3100_get_fd_closed_state(fd)) {
                ret |= MP_STREAM_POLL_RD;
            }
        }
        if (flags & MP_STREAM_POLL_WR) {
            SL_FD_SET(fd, &wfds);
        }

        // call cc3100 select with minimum timeout
        SlTimeval_t tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        // xfds not supported by simplelink
        int nfds = sl_Select(fd + 1, &rfds, &wfds, NULL, &tv);

        // check for error
        if (nfds < 0) {
            *_errno = -nfds;
            return -1;
        }

        // check return of select
        if (nfds > 0) {
            if (SL_FD_ISSET(fd, &rfds)) {
                ret |= MP_STREAM_POLL_RD;
            }
            if (SL_FD_ISSET(fd, &wfds)) {
                ret |= MP_STREAM_POLL_WR;
                if (socket->s_nonblocking_connect) {
                    // CC3100 is in progress of a non-blocking connect, and since
                    // the socket is now ready for writing that means it should
                    // have completed the connect.  We need to call sl_Connect once
                    // more to actually finish the connection.
                    SlSockAddr_t addr;
                    addr.sa_family = SL_AF_INET;
                    addr.sa_data[0] = 0;
                    addr.sa_data[1] = 0;
                    addr.sa_data[2] = 0;
                    addr.sa_data[3] = 0;
                    addr.sa_data[4] = 0;
                    addr.sa_data[5] = 0;
                    int ret = sl_Connect(socket->s_fd, &addr, sizeof(addr));
                    (void)ret; // TODO probably should check the return value
                    socket->s_nonblocking_connect = false; // now connected
                }
            }
        }
    } else {
        *_errno = MP_EINVAL;
        ret = -1;
    }
    return ret;
}


const mod_network_nic_type_t mod_network_nic_type_cc3100 = {
    .base = {
        { &mp_type_type },
        .name = MP_QSTR_CC3100,
        .make_new = cc3100_make_new,
        .locals_dict = (mp_obj_t)&cc3100_locals_dict,
    },
    .gethostbyname = cc3100_gethostbyname,
    .socket = cc3100_socket_socket,
    .close = cc3100_socket_close,
    .bind = cc3100_socket_bind,
    .listen = cc3100_socket_listen,
    .accept = cc3100_socket_accept,
    .connect = cc3100_socket_connect,
    .send = cc3100_socket_send,
    .recv = cc3100_socket_recv,
    .sendto = cc3100_socket_sendto,
    .recvfrom = cc3100_socket_recvfrom,
    .setsockopt = cc3100_socket_setsockopt,
    .settimeout = cc3100_socket_settimeout,
    .ioctl = cc3100_socket_ioctl,
};

// --------------------------------------------------------------------------------------
// ASYNCHRONOUS EVENT HANDLERS
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
//    This function handles WLAN events
//
//    pWlanEvent is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Event)
    {
    case SL_WLAN_CONNECT_EVENT:
    {
        LOG_INFO("[WLAN EVENT] connect");
        wlan_connected = true;
        /*
         * Information about the connected AP (like name, MAC etc) will be
         * available in 'slWlanConnectAsyncResponse_t' - Applications
         * can use it if required
         *
         * slWlanConnectAsyncResponse_t *pEventData = NULL;
         * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
         *
         */
    }
    break;

    case SL_WLAN_DISCONNECT_EVENT:
    {
        //LOG_INFO("[WLAN EVENT] disconnect");
        // link down
        wlan_connected = false;
        ip_obtained = false;
        // Get the reason code
        slWlanConnectAsyncResponse_t*  pEventData = NULL;
        pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

        if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
        {           // user initiated a disconnect request
                    //LOG_INFO("Device disconnected from the AP on application's request");
        }
        else
        {
            LOG_INFO("Device disconnected from the AP on an ERROR..!!");
            printf("pEventData->reason_code: %d\n", (int)pEventData->reason_code);
        }
    }
    break;

    case SL_WLAN_STA_CONNECTED_EVENT:
        LOG_INFO("[WLAN EVENT] SL_WLAN_STA_CONNECTED_EVENT");
        break;
    case SL_WLAN_STA_DISCONNECTED_EVENT:
        LOG_INFO("[WLAN EVENT] SL_WLAN_STA_DISCONNECTED_EVENT");
        break;
    case SL_WLAN_CONNECTION_FAILED_EVENT:
        LOG_INFO("[WLAN EVENT] SL_WLAN_CONNECTION_FAILED_EVENT");
        break;
    default:
        LOG_INFO("[WLAN EVENT] Unexpected event");
        printf("pWlanEvent->Event: %d\n", (int)pWlanEvent->Event);
        break;
    }
}

// --------------------------------------------------------------------------------------
//    This function handles events for IP address acquisition via DHCP
//
//    pNetAppEvent is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
    case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        LOG_INFO("[NETAPP EVENT] IP acquired");
        ip_obtained = true;
        /*
         * Information about the connection (like IP, gateway address etc)
         * will be available in 'SlIpV4AcquiredAsync_t'
         * Applications can use it if required
         *
         * SlIpV4AcquiredAsync_t *pEventData = NULL;
         * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
         *
         */

         SlIpV4AcquiredAsync_t *pEventData = NULL;
         pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
         uint8_t ip1,ip2,ip3,ip4;
         ip1 = pEventData->ip & 0xff;
         ip2 = (pEventData->ip >> 8) & 0xff;
         ip3 = (pEventData->ip >> 16) & 0xff;
         ip4 = (pEventData->ip >> 24) & 0xff;
         printf("IP:  %d.%d.%d.%d\n",ip4,ip3,ip2,ip1);
         ip1 = pEventData->gateway & 0xff;
         ip2 = (pEventData->gateway >> 8) & 0xff;
         ip3 = (pEventData->gateway >> 16) & 0xff;
         ip4 = (pEventData->gateway >> 24) & 0xff;
         printf("GW:  %d.%d.%d.%d\n",ip4,ip3,ip2,ip1);
         ip1 = pEventData->dns & 0xff;
         ip2 = (pEventData->dns >> 8) & 0xff;
         ip3 = (pEventData->dns >> 16) & 0xff;
         ip4 = (pEventData->dns >> 24) & 0xff;
         printf("DNS: %d.%d.%d.%d\n",ip4,ip3,ip2,ip1);

        break;
    case SL_NETAPP_IP_LEASED_EVENT:
        LOG_INFO("[NETAPP EVENT] SL_NETAPP_IP_LEASED");
        break;
    case SL_NETAPP_IP_RELEASED_EVENT:
        LOG_INFO("[NETAPP EVENT] IP released");
        // mark socket for closure

        break;
    default:
        LOG_INFO("[NETAPP EVENT] Unexpected event");
        printf("pWlanEvent->Event: %d\n", (int)pNetAppEvent->Event);
        break;
    }
}

// --------------------------------------------------------------------------------------
//    This function handles callback for the HTTP server events
//
//    pServerEvent - Contains the relevant event information
//    pServerResponse - Should be filled by the user with the
//    relevant response information
//
// --------------------------------------------------------------------------------------
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application */
    LOG_INFO("[HTTP EVENT] Unexpected event");
}

// --------------------------------------------------------------------------------------
//    This function handles general error events indication
//
//    pDevEvent is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */

    switch(pDevEvent->Event)
    {
        case SL_DEVICE_FATAL_ERROR_EVENT:
            LOG_INFO("[GENERAL EVENT] Fatal error: reset device");
            break;
        case SL_DEVICE_DRIVER_TIMEOUT_ASYNC_EVENT:
            printf("[GENERAL EVENT] timeout async event info=%d\n", (int)pDevEvent->EventData.deviceDriverReport.info);
            break;
        default:
            LOG_INFO("[GENERAL EVENT]");
            printf("%d\n", (int)pDevEvent->Event);
            break;
    }
}

// --------------------------------------------------------------------------------------
//    This function handles socket events indication
//
//    pSock is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    switch(pSock->Event)
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            LOG_INFO("[SOCK EVENT] SL_NETAPP_SOCKET_TX_FAILED");
            /*
             * TX Failed
             *
             * Information about the socket descriptor and status will be
             * available in 'SlSockEventData_t' - Applications can use it if
             * required
             *
             * SlSockEventData_t *pEventData = NULL;
             * pEventData = & pSock->EventData;
             */
            switch(pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE:
                    LOG_INFO("[SOCK EVENT] Close socket operation failed to transmit all queued packets");
                    break;
                default:
                    printf("[SOCK EVENT] Unexpected status %d", pSock->socketAsyncEvent.SockTxFailData.status);
                    break;
            }
            break;

        /*case SL_SOCKET_ASYNC_EVENT:
            LOG_INFO("[SOCK EVENT] SL_SOCKET_ASYNC_EVENT");
            break;*/

        default:
            printf("[SOCK EVENT] Unexpected event %d\n", (int)pSock->Event);
            break;
    }
}
