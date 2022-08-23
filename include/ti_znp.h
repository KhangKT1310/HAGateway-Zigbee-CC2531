/*
 * File: ti_znp.h
 * Project: HAGateway-Zigbee
 * File Created: 05 August 2022 03:33:22
 * Author: KhangKT
 * AuthorEmail: khangkieutrong@gmail.com
 * Phone:0964991713
 */


#ifndef __H__TI_ZNP__
#define __H__TI_ZNP__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TI_ZNP_SOF 0xFE

enum
{
    ZNP_STATUS_SUCCESS,
    ZNP_STATUS_CONTINUTE,
    ZNP_STATUS_FAIL = 0xffffff
};

enum
{
    TI_ZNP_STATE_FIRST,
    TI_ZNP_STATE_SOF,
    TI_ZNP_STATE_LEN,
    TI_ZNP_STATE_CMD,
    TI_ZNP_STATE_DATA,
    TI_ZNP_STATE_FCS,
};

#pragma pack(1)
typedef struct
{
    uint8_t sof;
    uint8_t len;
    uint16_t cmd;
    uint8_t data[256];
    uint8_t fcs;
}  ti_znp_cmd_t;


typedef struct
{
    /* data */
    uint8_t state;
    int remain;
    uint8_t rdata[256];
    uint8_t *p;

    ti_znp_cmd_t cmd;
} ti_znp_parse_t;

void ti_znp_init();
int znp_getCmd(uint8_t *data, int size);

#endif // __H__TI_ZNP__