/*
 * File: serial.h
 * Project: HAGateway-Zigbee
 * File Created: 20 July 2022 08:39:18
 * Author: KhangKT
 * AuthorEmail: khangkieutrong@gmail.com
 * Phone:0964991713
 */


#ifndef __SERIAL_H__
#define __SERIAL_H__


#include <stdio.h>  /* Standard I/O Definition*/
#include <stdlib.h> /* Standard Function Library Definition*/
#include <unistd.h> /*Unix Standard Function Definition*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   /* File Control Definition*/
#include <termios.h> /*PPSIX Terminal Control Definition*/
#include <errno.h>   /* Error Number Definition*/
#include <string.h>
#include <stdint.h>


#define Serial_Port "/dev/ttyS5"
#define Serial_BR  B115200
#define Max_Device 10

extern uint8_t BUFF_INFO[10];
extern uint8_t NXP_chip[3];
extern uint8_t TI_chip[3]; 
extern uint8_t SiL_chip[3];

extern uint8_t CMD_ZB_LEAVE[]; 
extern uint8_t CMD_AF_REGISTER[];                                  
extern uint8_t CMD_ZB_SYSTEM_RESET[];        
extern uint8_t CMD_ZB_START_REQUEST_1[];                            
extern uint8_t CMD_ZB_START_REQUEST_2[];                            
extern uint8_t CMD_ZCD_NV_STARTUP_OPTION[];                         
extern uint8_t CMD_ZCD_NV_LOGICAL_TYPE[];                         
extern uint8_t CMD_ZCD_NV_PANID[];                                
extern uint8_t CMD_ZCD_NV_SET_CHANNEL_20[];                         
extern uint8_t CMD_ZCD_NV_SET_CHANNEL_NONE[];                      
extern uint8_t CMD_ZCD_NV_ZDO_DIRECT_CB[];                         
extern uint8_t CMD_APP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE[];
extern uint8_t CMD_ZDO_MGMT_PERMIT_JOIN_REQ[];                     
extern uint8_t CMD_UTIL_GET_DEVICE_INFO[];       


struct sInfo_device
{
    uint8_t  NwK_Address[2];
    uint8_t  Device_Address[8];
}Number_Device[Max_Device],Leave_NWK;


int Serial_write(int fd, unsigned char opcode, unsigned char *payload, unsigned char size);
static int Serial_Set(int fd, int speed, int parity);
unsigned int my_strlen(char *p);
void *Serial_receive_handle(void *arg);
int Serial_start();
void Sys_Version_CMD(int fd);
uint8_t *Buff_Nwk_Req(char Buff_Leave[]);

#endif /* __SERIAL_H__*/
