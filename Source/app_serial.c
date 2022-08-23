/*
 * File: app_serial.c
 * Project: HAGateway-Zigbee
 * File Created: 20 July 2022 03:31:44
 * Author: KhangKT
 * AuthorEmail: khangkieutrong@gmail.com
 * Phone:0964991713
 */

#include <stdio.h>  /* Standard I/O Definition*/
#include <stdlib.h> /* Standard Function Library Definition*/
#include <unistd.h> /*Unix Standard Function Definition*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   /* File Control Definition*/
#include <termios.h> /*PPSIX Terminal Control Definition*/
#include <errno.h>   /* Error Number Definition*/

#include "app_serial.h"
#include "serial.h"
#include "ti_znp.h"

void Write_serial_CMD(int fd,char CMD_Serial[])
{
    char buffer[255];
    int size = my_strlen(CMD_Serial);
    memcpy(buffer,CMD_Serial,size);
    Serial_write(fd, TI_ZNP_SOF, buffer, size);
}


