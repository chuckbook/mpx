#define _i8 char

/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Damien P. George
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

// We can't include stdio.h because it defines _types_fd_set, but we
// need to use the CC3100 version of this type.

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
//#include <errno.h>

//#include "stm32f4xx_hal.h"
//#include "mpconfig.h"
//#include "nlr.h"
//#include "qstr.h"
#include "py/obj.h"
#include "py/misc.h"
#include "py/mphal.h"
#include "py/objtuple.h"
#include "py/objlist.h"
#include "py/stream.h"
#include "py/runtime.h"
#include "py/mpprint.h"
#include "modnetwork.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "spi.h"
#include "pybioctl.h"


//*****************************************************************************
// Prefix exported names to avoid name clash
//*****************************************************************************
#define CC3100_EXPORT(name) cc3100_ ## name

#include "simplelink.h"
#include "inet_ntop.h"
#include "inet_pton.h"

//#include "log.h"
int snprintf(char *str, size_t size, const char *fmt, ...);
int printf(const char *fmt, ...);

//#define LOG_ERR(x) printf("ERR: %s %s\n", __func__, x)
//#define LOG_INFO(x) printf("INFO: %s %s\n", __func__, x)
//#define LOG_COND_RET(a, b) if (a) { LOG_INFO("cond"); return b; }

  #define LOG_INFO(msg)\
  {\
      if(msg)\
      {\
          printf("INFO: [%s:%d] %s() [%s]\n", __FILE__, __LINE__, __FUNCTION__, (#msg));\
      }\
      else\
      {\
        printf("INFO: [%s:%d] %s()\n", __FILE__, __LINE__, __FUNCTION__);\
      }\
  }

  #define LOG_ERR(msg)\
  {\
      if(msg)\
      {\
          printf("ERR: [%s:%d] %s() [%s]\n", __FILE__, __LINE__, __FUNCTION__, (#msg));\
      }\
      else\
      {\
        printf("ERR: [%s:%d] %s()\n", __FILE__, __LINE__, __FUNCTION__);\
      }\
  }

  // Log on a condition without a return, just continue execution
  #define LOG_COND_CONT(condition)\
  {\
      if(!(condition))\
      {\
          printf("Condition [%s] failed in [%s:%d] %s() \n", (#condition), __FILE__, __LINE__, __FUNCTION__);\
      }\
  }
  // Log on a condition with a return code
  #define LOG_COND_RET(condition, rc)\
  {\
      if(!(condition))\
      {\
          printf("Condition [%s] failed in [%s:%d] %s()\n", (#condition), __FILE__, __LINE__, __FUNCTION__);\
          return(rc);\
      }\
  }

#define RAISE_EXCEP(a, b, c) if (a) { LOG_ERR(c); }
// CC3100 defines (different from standard one!)
#define ENOENT  -2
#define EPIPE   -32

#define CC31K_SOCKET_MAX     SL_MAX_SOCKETS   // the maximum number of sockets that the CC31K could support
#define CC31K_MAX_RX_PACKET  (16000)
#define CC31K_MAX_TX_PACKET  (1460)

typedef struct _netapp_ipconfig_ret_args_t
{
    unsigned char aucIP[4];
    unsigned char aucSubnetMask[4];
    unsigned char aucDefaultGateway[4];
    unsigned char aucDHCPServer[4];
    unsigned char aucDNSServer[4];
    unsigned char uaMacAddr[6];
    unsigned char uaSSID[32];
}tNetappIpconfigRetArgs;

/// \moduleref network

int CC3100_EXPORT(errno); // for cc3100 driver

STATIC int cc31k_init(void);

STATIC mp_obj_t cc31k_socket_new(mp_uint_t family, mp_uint_t type, mp_uint_t protocol, int *_errno);

STATIC volatile uint32_t fd_closed_state = 0;
STATIC volatile bool wlan_connected      = false;
STATIC volatile bool ip_obtained         = false;

STATIC int cc31k_get_fd_closed_state(int fd) {
    return fd_closed_state & (1 << fd);
}

STATIC void cc31k_set_fd_closed_state(int fd) {
    fd_closed_state |= 1 << fd;
}

STATIC void cc31k_reset_fd_closed_state(int fd) {
    fd_closed_state &= ~(1 << fd);
}

STATIC mp_obj_t cc31k_socket(mp_obj_t nic, int domain, int type, int fileno, int *_errno) {
printf("%s\n", __func__);
    switch (domain) {
        case MOD_NETWORK_AF_INET: domain = AF_INET; break;
        case MOD_NETWORK_AF_INET6: domain = AF_INET6; break;
        default: *_errno = EAFNOSUPPORT; return MP_OBJ_NULL;
    }

    switch (type) {
        case MOD_NETWORK_SOCK_STREAM: type = SOCK_STREAM; break;
        case MOD_NETWORK_SOCK_DGRAM: type = SOCK_DGRAM; break;
        case MOD_NETWORK_SOCK_RAW: type = SOCK_RAW; break;
        default: *_errno = EINVAL; return MP_OBJ_NULL;
    }

    return cc31k_socket_new(domain, type, 0, _errno);
}
STATIC int cc31k_socket_socket(mod_network_socket_obj_t *socket, int *_errno) {
    int err = 0;
    cc31k_socket(NULL, MOD_NETWORK_AF_INET, MOD_NETWORK_SOCK_STREAM, 0, &err);
    return err;
}

STATIC int cc31k_gethostbyname(mp_obj_t nic, const char *name, mp_uint_t len, uint8_t *out_ip) {
    uint32_t ip;

    int rc = sl_NetAppDnsGetHostByName((_i8 *)name, (uint16_t)len, &ip, SL_AF_INET);
    if (rc < 0) {
        return CC3100_EXPORT(errno);
    }

    if (ip == 0) {
        // unknown host
        return ENOENT;
    }

    out_ip[0] = ip >> 24;
    out_ip[1] = ip >> 16;
    out_ip[2] = ip >> 8;
    out_ip[3] = ip;

    return 0;
}

/******************************************************************************/
// Micro Python bindings; CC31k class

/// \class CC31k - driver for CC3100 Wifi modules

typedef struct _cc31k_obj_t {
    mp_obj_base_t base;
} cc31k_obj_t;

/// \classmethod \constructor(spi, pin_cs, pin_en, pin_irq)
/// Initialise the CC3100 using the given SPI bus and pins and return a CC31k object.
//
// Note: pins were originally hard-coded to:
//      PYBv1.0: init(pyb.SPI(2), pyb.Pin.board.Y5, pyb.Pin.board.Y4, pyb.Pin.board.Y3)
//        [SPI on Y position; Y6=B13=SCK, Y7=B14=MISO, Y8=B15=MOSI]
//
//      STM32F4DISC: init(pyb.SPI(2), pyb.Pin.cpu.A15, pyb.Pin.cpu.B10, pyb.Pin.cpu.B11)
STATIC mp_obj_t cc31k_make_new(mp_obj_t type_in, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 4, 4, false);

    // set the pins to use
    /*SpiInit(
        spi_get_handle(args[0]),
        pin_find(args[1]),
        pin_find(args[2]),
        pin_find(args[3])
    );*/

    wlan_connected = false;
    ip_obtained    = false;

    #if 1
    int rc = cc31k_init();
    if(rc != 0)
    {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed to init cc31k module"));
    }
    #endif

    cc31k_obj_t *cc31k = m_new_obj(cc31k_obj_t);
    cc31k->base.type = (mp_obj_type_t*)&mod_network_nic_type_cc31k;

    // register with network module
    mod_network_register_nic(cc31k);

    return cc31k;
}

/// \method connect(ssid, key=None, *, security=WPA2, bssid=None)
STATIC mp_obj_t cc31k_connect(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid, MP_ARG_REQUIRED | MP_ARG_OBJ,       {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_key, MP_ARG_OBJ,                          {.u_obj = mp_const_none} },
        { MP_QSTR_security, MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = SL_SEC_TYPE_WPA} },
        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ,       {.u_obj = mp_const_none} },
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
        sec = args[2].u_int;
    }

#if 0
    // get bssid
    const char *bssid = NULL;
    if (args[3].u_obj != mp_const_none) {
        bssid = mp_obj_str_get_str(args[3].u_obj);
    }
#endif

    SlSecParams_t secParams = {0};
    int16_t retVal = 0;

    secParams.Key       = (_i8 *)key;
    secParams.KeyLen    = key_len;
    secParams.Type      = sec;

    retVal = sl_WlanConnect((_i8 *)ssid, ssid_len, 0, &secParams, 0);
    if (retVal < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed sl_WlanConnect"));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(cc31k_connect_obj, 1, cc31k_connect);

STATIC mp_obj_t cc31k_disconnect(mp_obj_t self_in) {
    int ret = sl_WlanDisconnect();

    return mp_obj_new_int(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_disconnect_obj, cc31k_disconnect);

STATIC mp_obj_t cc31k_is_connected(mp_obj_t self_in) {

    if (wlan_connected && ip_obtained) {
        return mp_const_true;
    }
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_is_connected_obj, cc31k_is_connected);

STATIC mp_obj_t cc31k_update(mp_obj_t self_in) {

    _SlNonOsMainLoopTask();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_update_obj, cc31k_update);

STATIC mp_obj_t cc31k_sleep(mp_obj_t self_in) {

    sl_Stop(100);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_sleep_obj, cc31k_sleep);

STATIC mp_obj_t decode_addr(unsigned char *ip, int n_bytes)
{
    char data[64] = "";
    if(n_bytes == 4)
    {
        snprintf(data, 64, "%u.%u.%u.%u", ip[3], ip[2], ip[1], ip[0]);
    }
    else if(n_bytes == 6)
    {
        snprintf(data, 64, "%02x:%02x:%02x:%02x:%02x:%02x", ip[5], ip[4], ip[3], ip[2], ip[1], ip[0]);
    }
    else if(n_bytes == 32)
    {
        snprintf(data, 64, "%s", ip);
    }
    return(mp_obj_new_str(data, strlen(data), false));
}

STATIC void decode_addr_and_store(mp_obj_t object, qstr q_attr, unsigned char *ip, int n_bytes)
{
    mp_store_attr(object, q_attr, decode_addr(ip, n_bytes));
}

STATIC mp_obj_t cc31k_ifconfig(mp_obj_t self_in) {

    unsigned char len = sizeof(_NetCfgIpV4Args_t);
    unsigned char dhcpIsOn = 0;
    _NetCfgIpV4Args_t ipV4 = {0};
    STATIC mp_obj_t net_address_type = MP_OBJ_NULL;
    tNetappIpconfigRetArgs ipconfig  = {{0}};

    // NOTE: the return code upon success is not 0 (this is a bug in the documenation)
    int rc = sl_NetCfgGet(SL_IPV4_STA_P2P_CL_GET_INFO, &dhcpIsOn, &len, (uint8_t *)&ipV4);
    if (rc < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed sl_NetCfgGet ip"));
    }

    uint8_t macAddressVal[SL_MAC_ADDR_LEN] = {0};
    uint8_t macAddressLen = SL_MAC_ADDR_LEN;
    rc = sl_NetCfgGet(SL_MAC_ADDRESS_GET, NULL, &macAddressLen, (uint8_t *)macAddressVal);
    if (rc < 0) {
        //printf("WRN: negative return code, but data is valid (TI bug)\n");
        //nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed sl_NetCfgGet mac"));
    }

    //memset(ifconfig, 0, sizeof(tNetappIpconfigRetArgs));
    memcpy(ipconfig.aucIP,            &ipV4.ipV4,         4);
    memcpy(ipconfig.aucSubnetMask,    &ipV4.ipV4Mask,     4);
    memcpy(ipconfig.aucDefaultGateway,&ipV4.ipV4Gateway,  4);
    memcpy(ipconfig.aucDNSServer,     &ipV4.ipV4DnsServer,4);
    memcpy(ipconfig.uaMacAddr,        macAddressVal,      SL_MAC_ADDR_LEN);

    net_address_type = mp_obj_new_type(QSTR_FROM_STR_STATIC("NetIfConfig"), mp_const_empty_tuple, mp_obj_new_dict(0));
    // make a new NetAddress object
    mp_obj_t net_addr = mp_call_function_0(net_address_type);

    // fill the NetAddress object with data
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("ip"),      ipconfig.aucIP,             4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("subnet"),  ipconfig.aucSubnetMask,     4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("gateway"), ipconfig.aucDefaultGateway, 4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("dhcp"),    ipconfig.aucDHCPServer,     4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("dns"),     ipconfig.aucDNSServer,      4);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("mac"),     ipconfig.uaMacAddr,         6);
    decode_addr_and_store(net_addr, QSTR_FROM_STR_STATIC("ssid"),    ipconfig.uaSSID,            32);

    #if 0
    printf(("IP  : %d.%d.%d.%d\n" \
            "Mask: %d.%d.%d.%d\n" \
            "GW  : %d.%d.%d.%d\n" \
            "DHCP: %d.%d.%d.%d\n" \
            "DNS : %d.%d.%d.%d\n" \
            "MAC : %02X:%02X:%02X:%02X:%02X:%02X\n" \
            "SSID: %s\n",
            ipconfig.aucIP[3], ipconfig.aucIP[2], ipconfig.aucIP[1], ipconfig.aucIP[0],
            ipconfig.aucSubnetMask[3], ipconfig.aucSubnetMask[2], ipconfig.aucSubnetMask[1], ipconfig.aucSubnetMask[0],
            ipconfig.aucDefaultGateway[3], ipconfig.aucDefaultGateway[2], ipconfig.aucDefaultGateway[1], ipconfig.aucDefaultGateway[0],
            ipconfig.aucDHCPServer[3], ipconfig.aucDHCPServer[2], ipconfig.aucDHCPServer[1], ipconfig.aucDHCPServer[0],
            ipconfig.aucDNSServer[3], ipconfig.aucDNSServer[2], ipconfig.aucDNSServer[1], ipconfig.aucDNSServer[0],
            ipconfig.uaMacAddr[5], ipconfig.uaMacAddr[4], ipconfig.uaMacAddr[3], ipconfig.uaMacAddr[2], ipconfig.uaMacAddr[1], ipconfig.uaMacAddr[0], 
            ipconfig.uaSSID);
    #endif
    return(net_addr);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_ifconfig_obj, cc31k_ifconfig);

STATIC mp_obj_t cc31k_patch_version(mp_obj_t self_in) {
    uint8_t pver[2];
    mp_obj_tuple_t *t_pver;

    //nvmem_read_sp_version(pver);
    t_pver = mp_obj_new_tuple(2, NULL);
    t_pver->items[0] = mp_obj_new_int(pver[0]);
    t_pver->items[1] = mp_obj_new_int(pver[1]);
    return t_pver;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_patch_version_obj, cc31k_patch_version);

STATIC mp_obj_t cc31k_patch_program(mp_obj_t self_in, mp_obj_t key_in) {
    const char *key = mp_obj_str_get_str(key_in);
    if (key[0] == 'p' && key[1] == 'g' && key[2] == 'm' && key[3] == '\0') {
        //patch_prog_start();
    } else {
        printf("please pass 'pgm' as argument in order to program\n");
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_patch_program_obj, cc31k_patch_program);

STATIC const mp_map_elem_t cc31k_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_connect),         (mp_obj_t)&cc31k_connect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_disconnect),      (mp_obj_t)&cc31k_disconnect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_isconnected),    (mp_obj_t)&cc31k_is_connected_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_update),          (mp_obj_t)&cc31k_update_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sleep),           (mp_obj_t)&cc31k_sleep_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ifconfig),        (mp_obj_t)&cc31k_ifconfig_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_patch_version),   (mp_obj_t)&cc31k_patch_version_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_patch_program),   (mp_obj_t)&cc31k_patch_program_obj },

    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_WEP), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA2), MP_OBJ_NEW_SMALL_INT(SL_SEC_TYPE_WPA_ENT) },
};

STATIC MP_DEFINE_CONST_DICT(cc31k_locals_dict, cc31k_locals_dict_table);

const mod_network_nic_type_t mod_network_nic_type_cc31k = {
    .base = {
        { &mp_type_type },
        .name = MP_QSTR_CC31K,            // TSP_2015120
        //.print = cc31k_print,         // TSP_2015120
        .make_new = cc31k_make_new,
        .locals_dict = (mp_obj_t)&cc31k_locals_dict,
    },
    .socket = cc31k_socket_socket,  //cc31k_socket,     // TSP_20151201
    .gethostbyname = cc31k_gethostbyname,     // TSP_20151201
};

/******************************************************************************/
// Micro Python bindings; CC31k socket class

#define MAX_ADDRSTRLEN      (128)
#define MAX_RX_PACKET       (CC3000_RX_BUFFER_SIZE-CC3000_MINIMAL_RX_SIZE-1)
#define MAX_TX_PACKET       (CC3000_TX_BUFFER_SIZE-CC3000_MINIMAL_TX_SIZE-1)

typedef struct _cc31k_socket_obj_t {
    mp_obj_base_t base;
    int fd;
} cc31k_socket_obj_t;

STATIC const mp_obj_type_t cc31k_socket_type;

STATIC mp_obj_t cc31k_socket_new(mp_uint_t family, mp_uint_t type, mp_uint_t protocol, int *_errno) {
    // create socket object
    cc31k_socket_obj_t *s = m_new_obj_with_finaliser(cc31k_socket_obj_t);
    s->base.type = (mp_obj_t)&cc31k_socket_type;

    // open socket
    s->fd = sl_Socket(family, type, protocol);
    if (s->fd < 0) {
        m_del_obj(cc31k_socket_obj_t, s);
        *_errno = CC3100_EXPORT(errno);
        return MP_OBJ_NULL;
    }

    // clear socket state
    cc31k_reset_fd_closed_state(s->fd);

    return s;
}
#ifdef TSP_20151201
STATIC void cc31k_socket_print(void (*print)(void *env, const char *fmt, ...), void *env, mp_obj_t self_in, mp_print_kind_t kind) {
    cc31k_socket_obj_t *self = self_in;
    printf("<CC31k.socket fd=%d>", self->fd);
}
#endif
STATIC mp_obj_t cc31k_socket_send(mp_obj_t self_in, mp_obj_t buf_in) {
    cc31k_socket_obj_t *self = self_in;

    if (cc31k_get_fd_closed_state(self->fd)) {
        sl_Close(self->fd);
        nlr_raise(mp_obj_new_exception_arg1(&mp_type_OSError, MP_OBJ_NEW_SMALL_INT(EPIPE)));
    }

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    // CC31K does not handle fragmentation, and will overflow,
    // split the packet into smaller ones and send them out.
    mp_int_t bytes = 0;
    while (bytes < bufinfo.len) {
        int n = MIN((bufinfo.len - bytes), CC31K_MAX_TX_PACKET);
        n = sl_Send(self->fd, (uint8_t*)bufinfo.buf + bytes, n, 0);
        if (n <= 0) {
            nlr_raise(mp_obj_new_exception_arg1(&mp_type_OSError, MP_OBJ_NEW_SMALL_INT(CC3100_EXPORT(errno))));
        }
        bytes += n;
    }

    return MP_OBJ_NEW_SMALL_INT(bytes);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_socket_send_obj, cc31k_socket_send);

STATIC mp_obj_t cc31k_socket_recv(mp_obj_t self_in, mp_obj_t len_in) {
    cc31k_socket_obj_t *self = self_in;

    if (cc31k_get_fd_closed_state(self->fd)) {
        sl_Close(self->fd);
        nlr_raise(mp_obj_new_exception_arg1(&mp_type_OSError, MP_OBJ_NEW_SMALL_INT(EPIPE)));
    }

    // recv upto MAX_RX_PACKET
    mp_int_t len = mp_obj_get_int(len_in);
    len = MIN(len, CC31K_MAX_RX_PACKET);

    vstr_t vstr;
    vstr_init_len(&vstr, len);

    //byte *buf = mp_new(byte, len);
    //mp_obj_t ret_obj = mp_obj_str_builder_start(&mp_type_bytes, len, &buf);       // TSP_20151201
    len = sl_Recv(self->fd, (byte*)vstr.buf, len, 0);
    if (len == 0) {
        return mp_const_empty_bytes;
    } else if (len < 0) {
        nlr_raise(mp_obj_new_exception_arg1(&mp_type_OSError, MP_OBJ_NEW_SMALL_INT(CC3100_EXPORT(errno))));
    } else {
        vstr.len = len;
        vstr.buf[vstr.len] = '\0';
        return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        //return mp_obj_str_builder_end_with_len(ret_obj, len);     // TSP_20151201
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_socket_recv_obj, cc31k_socket_recv);

STATIC mp_obj_t cc31k_socket_bind(mp_obj_t self_in, mp_obj_t addr_obj) {
    cc31k_socket_obj_t *self = self_in;

    mp_obj_t *addr;
    mp_obj_get_array_fixed_n(addr_obj, 2, &addr);

    // fill sockaddr struct
    int port = mp_obj_get_int(addr[1]);
    sockaddr_in addr_in = {
        .sin_family = AF_INET,
        .sin_port   = sl_Htons(port),
        .sin_addr.s_addr = 0,// INADDR_ANY
        .sin_zero   = {0}
    };

    const char *host = mp_obj_str_get_str(addr[0]);
    if (strlen(host) && !inet_pton(AF_INET, host, &addr_in.sin_addr.s_addr)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "invalid IP address"));
    }

    // bind socket
    if (sl_Bind(self->fd, (sockaddr*) &addr_in, sizeof(sockaddr_in)) < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "bind failed"));
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_socket_bind_obj, cc31k_socket_bind);

STATIC mp_obj_t cc31k_socket_listen(mp_obj_t self_in, mp_obj_t backlog) {
    cc31k_socket_obj_t *self = self_in;
    if (sl_Listen(self->fd, mp_obj_get_int(backlog)) < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "listen failed"));
    }

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_socket_listen_obj, cc31k_socket_listen);

STATIC mp_obj_t cc31k_socket_accept(mp_obj_t self_in) {
    cc31k_socket_obj_t *self = self_in;
    int fd;

    sockaddr addr;
    socklen_t addr_len = sizeof(sockaddr);

    // accept incoming connection
    if ((fd = sl_Accept(self->fd, &addr, &addr_len)) < 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "accept failed"));
    }

    // clear socket state
    cc31k_reset_fd_closed_state(fd);

    // create new socket object
    cc31k_socket_obj_t *socket_obj = m_new_obj_with_finaliser(cc31k_socket_obj_t);
    socket_obj->base.type = (mp_obj_t)&cc31k_socket_type;
    socket_obj->fd  = fd;

    char buf[MAX_ADDRSTRLEN]={0};
    if (inet_ntop(addr.sa_family,
        &(((sockaddr_in*)&addr)->sin_addr), buf, MAX_ADDRSTRLEN) == NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "invalid IP address"));
    }

    mp_obj_tuple_t *cli = mp_obj_new_tuple(2, NULL);
    mp_obj_tuple_t *cli_addr = mp_obj_new_tuple(2, NULL);

    cli->items[0] = socket_obj;
    cli->items[1] = cli_addr;
    cli_addr->items[0] = mp_obj_new_str(buf, strlen(buf), false);
    cli_addr->items[1] = mp_obj_new_int(((sockaddr_in*)&addr)->sin_port);

    return cli;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_socket_accept_obj, cc31k_socket_accept);

STATIC mp_obj_t cc31k_socket_connect(mp_obj_t self_in, mp_obj_t addr_obj) {
    cc31k_socket_obj_t *self = self_in;

    mp_obj_t *addr;
    mp_obj_get_array_fixed_n(addr_obj, 2, &addr);

    // fill sockaddr struct
    int port = mp_obj_get_int(addr[1]);
    sockaddr_in addr_in = {
        .sin_family = AF_INET,
        .sin_port   = sl_Htons(port),
        .sin_addr.s_addr = 0, // to be filled below using inet_pton
        .sin_zero   = {0}
    };

    const char *host = mp_obj_str_get_str(addr[0]);
    if (!inet_pton(AF_INET, host, &addr_in.sin_addr.s_addr)) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "invalid IP address"));
    }

    //printf("doing connect: fd=%d, sockaddr=(%d, %d, %lu)\n", self->fd, addr_in.sin_family, addr_in.sin_port, addr_in.sin_addr.s_addr);

    int ret = sl_Connect(self->fd, (sockaddr*)&addr_in, sizeof(sockaddr_in));
    if (ret != 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "[Errno %d] connect failed", ret));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_socket_connect_obj, cc31k_socket_connect);

STATIC mp_obj_t cc31k_socket_settimeout(mp_obj_t self_in, mp_obj_t timeout_ms) {
    cc31k_socket_obj_t *self = self_in;
    int tout = mp_obj_get_int(timeout_ms);
    //socklen_t optlen = sizeof(optval);

    struct SlTimeval_t timeval;
    timeval.tv_sec =  tout / 1000;                             // Seconds
    timeval.tv_usec = (tout - (1000 * timeval.tv_sec)) * 1000; // Microseconds. 10000 microseconds resolution

    if (sl_SetSockOpt(self->fd, SOL_SOCKET, SL_SO_RCVTIMEO, &timeval, sizeof(timeval)) != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "setsockopt failed"));
    }

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_socket_settimeout_obj, cc31k_socket_settimeout);

#ifdef TSP_20151201
STATIC int cc31k_socket_ioctl(mod_network_socket_obj_t *socket, mp_uint_t request, mp_uint_t arg, int *_errno) {
    return 0;
}
#endif

STATIC mp_obj_t cc31k_socket_setblocking(mp_obj_t self_in, mp_obj_t blocking) {
    cc31k_socket_obj_t *self = self_in;
    int optval;
    socklen_t optlen = sizeof(optval);

    if (mp_obj_get_int(blocking)) {
        optval = 0; 
    } else {
        optval = 1; // Enable non-blocking
    }

    if (sl_SetSockOpt(self->fd, SOL_SOCKET, SL_SO_NONBLOCKING, &optval, optlen) != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "setsockopt failed"));
    }

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(cc31k_socket_setblocking_obj, cc31k_socket_setblocking);

STATIC mp_obj_t cc31k_socket_setsockopt(uint n_args, const mp_obj_t *args)
{
    int rc;
    cc31k_socket_obj_t *self = args[0];
    int level = MP_OBJ_SMALL_INT_VALUE(args[1]);
    int option = mp_obj_get_int(args[2]);

    const void *optval;
    socklen_t optlen;
    if (MP_OBJ_IS_INT(args[3])) {
        int val = mp_obj_get_int(args[3]);
        optval = &val;
        optlen = sizeof(val);
    } else {
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_READ);
        optval = bufinfo.buf;
        optlen = bufinfo.len;
    }

    rc = sl_SetSockOpt(self->fd, level, option, optval, optlen);
    RAISE_EXCEP(rc>=0, mp_type_BaseException, "setsockopt failed");

    return(mp_const_none);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cc31k_socket_setsockopt_obj, 4, 4, cc31k_socket_setsockopt);

STATIC mp_obj_t cc31k_socket_close(mp_obj_t self_in) {
    cc31k_socket_obj_t *self = self_in;
    sl_Close(self->fd);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc31k_socket_close_obj, cc31k_socket_close);

STATIC const mp_map_elem_t cc31k_socket_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_send),        (mp_obj_t)&cc31k_socket_send_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_recv),        (mp_obj_t)&cc31k_socket_recv_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_bind),        (mp_obj_t)&cc31k_socket_bind_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_listen),      (mp_obj_t)&cc31k_socket_listen_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_accept),      (mp_obj_t)&cc31k_socket_accept_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_connect),     (mp_obj_t)&cc31k_socket_connect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_settimeout),  (mp_obj_t)&cc31k_socket_settimeout_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_setblocking), (mp_obj_t)&cc31k_socket_setblocking_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_setsockopt),  (mp_obj_t)&cc31k_socket_setsockopt_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_close),       (mp_obj_t)&cc31k_socket_close_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__),     (mp_obj_t)&cc31k_socket_close_obj },
};

STATIC MP_DEFINE_CONST_DICT(cc31k_socket_locals_dict, cc31k_socket_locals_dict_table);

mp_uint_t cc31k_ioctl(mp_obj_t self_in, mp_uint_t request, int *errcode, ...) {
    cc31k_socket_obj_t *self = self_in;
    va_list vargs;
    va_start(vargs, errcode);
    mp_uint_t ret;
    if (request == MP_IOCTL_POLL) {
        mp_uint_t flags = va_arg(vargs, mp_uint_t);
        ret = 0;
        int fd = self->fd;

        // init fds
        /*fd_set*/ SlFdSet_t rfds, wfds, xfds;
#ifdef TSP_20151201
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_ZERO(&xfds);
#else
        SL_FD_ZERO(&rfds);
        SL_FD_ZERO(&wfds);
        SL_FD_ZERO(&xfds);
#endif
        // set fds if needed
        if (flags & MP_IOCTL_POLL_RD) {
            SL_FD_SET(fd, &rfds);

            // A socket that just closed is available for reading.  A call to
            // recv() returns 0 which is consistent with BSD.
            if (cc31k_get_fd_closed_state(fd)) {
                ret |= MP_IOCTL_POLL_RD;
            }
        }
        if (flags & MP_IOCTL_POLL_WR) {
            SL_FD_SET(fd, &wfds);
        }
        if (flags & MP_IOCTL_POLL_HUP) {
            SL_FD_SET(fd, &xfds);
        }

        // call cc3100 select with minimum timeout
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        int nfds = sl_Select(fd + 1, &rfds, &wfds, &xfds, &tv);

        // check for error
        if (nfds == -1) {
            *errcode = CC3100_EXPORT(errno);
            return -1;
        }

        // check return of select
        if (SL_FD_ISSET(fd, &rfds)) {
            ret |= MP_IOCTL_POLL_RD;
        }
        if (SL_FD_ISSET(fd, &wfds)) {
            ret |= MP_IOCTL_POLL_WR;
        }
        if (SL_FD_ISSET(fd, &xfds)) {
            ret |= MP_IOCTL_POLL_HUP;
        }
    } else {
        *errcode = EINVAL;
        ret = -1;
    }
    va_end(vargs);
    return ret;
}

STATIC const mp_stream_p_t cc31k_socket_stream_p = {
    //.ioctl = cc31k_socket_ioctl,    //cc31k_ioctl,       // TSP_20151201
    .is_text = false,
};

STATIC const mp_obj_type_t cc31k_socket_type = {
    { &mp_type_type },
    .name = MP_QSTR_socket,
    //.print = cc31k_socket_print,      // TSP_20151201
    .stream_p = &cc31k_socket_stream_p,
    .locals_dict = (mp_obj_t)&cc31k_socket_locals_dict,
};


// --------------------------------------------------------------------------------------
// CC31K Driver Interface
// --------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------
// This function configures the SimpleLink device in its default state.
// - Sets the mode to STATION
// - Configures connection policy to Auto and AutoSmartConfig
// - Deletes all the stored profiles
// - Enables DHCP
// - Disables Scan policy
// - Sets Tx power to maximum
// - Sets power policy to normal
// - Unregisters mDNS services
//
// NOTE:
// * configure the device to default state by clearing the persistent settings stored
// * in NVMEM (viz. connection profiles & policies, power policy etc)
// *
// * the applications may choose to skip this step if the developer is sure
// * that the device is in its default state at start of application
// *
// * all profiles and persistent settings will be lost
// *
// --------------------------------------------------------------------------------------
STATIC int cc31k_init(void) {
    //SlVersionFull   ver = {{0}};

    uint8_t           val = 1;
    uint8_t           configOpt = 0;
//    uint8_t           configLen = 0;
    uint8_t           power = 0;
    int32_t           retVal = -1;
    int32_t           mode = -1;
printf("%s\n", __func__);
    mode = sl_Start(0, 0, 0);
    LOG_COND_RET(mode>=0, -1);

    // Need to check for station mode
    if (ROLE_STA != mode) {
        // Configure the device into station mode
        if (ROLE_AP == mode) {
            LOG_INFO("mode: ROLE_AP");
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(ip_obtained == false) {
                _SlNonOsMainLoopTask();
            }
        }

        // Select station mode, and restart to activate it
        retVal = sl_WlanSetMode(ROLE_STA);
        LOG_COND_RET(retVal>=0, -1);
        retVal = sl_Stop(100);
        LOG_COND_RET(retVal>=0, -1);
        mode = sl_Start(0, 0, 0);
        LOG_COND_RET(mode>=0, -1);
        if (ROLE_STA != mode) {
            LOG_ERR("not in station mode");
            return(-1);
        }
    }

    #if 0
    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (unsigned char *)(&ver));
    LOG_COND_RET(retVal>=0, -1);
    #endif

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    LOG_COND_RET(retVal>=0, -1);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    LOG_COND_RET(retVal>=0, -1);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if (0 == retVal) {
        // wait for disconnection
        while (wlan_connected == true) {
            _SlNonOsMainLoopTask();
        }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,(uint8_t *)&val);
    LOG_COND_RET(retVal>=0, -1);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    LOG_COND_RET(retVal>=0, -1);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&power);
    LOG_COND_RET(retVal>=0, -1);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    LOG_COND_RET(retVal>=0, -1);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    LOG_COND_RET(retVal>=0, -1);

    retVal = sl_Stop(100);
    LOG_COND_RET(retVal>=0, -1);

    /* Initializing the CC3100 device */
    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) || (ROLE_STA != retVal)) {
        LOG_ERR("");
        return(-1);
    }

    retVal = 0; // all is ok

    return(retVal);
}

// --------------------------------------------------------------------------------------
// ASYNCHRONOUS EVENT HANDLERS
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
//    This function handles WLAN events
//
//    pWlanEvent is the event passed to the handler
//
// --------------------------------------------------------------------------------------
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
printf("%s\n", __func__);
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
            {
                //LOG_INFO("[WLAN EVENT] connect");
                wlan_connected = true;
                /*
                 * Information about the connected AP (like name, MAC etc) will be
                 * available in 'sl_protocol_wlanConnectAsyncResponse_t' - Applications
                 * can use it if required
                 *
                 * sl_protocol_wlanConnectAsyncResponse_t *pEventData = NULL;
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
                sl_protocol_wlanConnectAsyncResponse_t*  pEventData = NULL;
                pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

                if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
                {   // user initiated a disconnect request
                    //LOG_INFO("Device disconnected from the AP on application's request");
                }
                else
                {
                    LOG_INFO("Device disconnected from the AP on an ERROR..!!");
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
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    printf("%s\n", __func__);
    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_ACQUIRED:
                //LOG_INFO("[NETAPP EVENT] IP acquired");
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
            break;
        case SL_NETAPP_IP_LEASED:
            LOG_INFO("[NETAPP EVENT] SL_NETAPP_IP_LEASED");
            break;
        case SL_NETAPP_IP_RELEASED:
            LOG_INFO("[NETAPP EVENT] IP released");
            // mark socket for closure
            cc31k_set_fd_closed_state(pNetAppEvent->EventData.sd);
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
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    printf("%s\n", __func__);
    switch(pSock->Event) {
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
            switch(pSock->EventData.status)
            {
                case SL_ECLOSE:
                    LOG_INFO("[SOCK EVENT] Close socket operation failed to transmit all queued packets");
                    break;
                default:
                    printf("[SOCK EVENT] Unexpected status %d", pSock->EventData.status);
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


