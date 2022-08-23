/*
 * File: app_serial.h
 * Project: HAGateway-Zigbee
 * File Created: 20 July 2022 03:32:19
 * Author: KhangKT
 * AuthorEmail: khangkieutrong@gmail.com
 * Phone:0964991713
 */

#ifndef __APP_SERIAL_H__
#define __APP_SERIAL_H__

#include <stdint.h>

#define JOIN            'J'
#define AUTO_CONFIG     'A'
#define DEVICE_INF      'I'
#define LEAVE           'L'


void Write_serial_CMD(int fd,char CMD_Serial[]);

#endif /*__APP_SERIAL_H__*/