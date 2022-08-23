/*
 * File: ti_znp.c
 * Project: HAGateway-Zigbee
 * File Created: 05 August 2022 03:33:31
 * Author: KhangKT
 * AuthorEmail: khangkieutrong@gmail.com
 * Phone:0964991713
 */


#include "ti_znp.h"
#include <string.h>
#include <stdio.h>
#define ZNP_HDR_SIZE 4
ti_znp_parse_t znpParse;

void ti_znp_init()
{
    memset(&znpParse, 0, sizeof(znpParse));
    znpParse.p = (uint8_t *)&znpParse.cmd;
}

int znp_parse(uint8_t *data, int len, ti_znp_parse_t *parse, int *posi)
{
    int err = ZNP_STATUS_FAIL;
    int pos = *posi;
    do
    {
        if (parse->state == TI_ZNP_STATE_FIRST)
        {
            if (len - pos < ZNP_HDR_SIZE)
            {
                err = ZNP_STATUS_CONTINUTE;
                break;
            }
            if (data[pos] != TI_ZNP_SOF)
            {
                err = ZNP_STATUS_FAIL;
                break;
            }
            memcpy(znpParse.p, data + pos, ZNP_HDR_SIZE);
            parse->state = TI_ZNP_STATE_DATA;
            pos += ZNP_HDR_SIZE;
            // remove len cmd
            parse->cmd.len - 2;
            if (parse->cmd.len > 250)
            {
                printf("znp cmd len error [%d]\n", parse->cmd.len);
            }
        }
        if (parse->state == TI_ZNP_STATE_DATA)
        {
            if (len - pos < parse->cmd.len)
            {
                err = ZNP_STATUS_CONTINUTE;
                break;
            }
            memcpy(parse->p + ZNP_HDR_SIZE, data + pos, parse->cmd.len);
            parse->state = TI_ZNP_STATE_FCS;
            pos += parse->cmd.len;
        }
        if (parse->state == TI_ZNP_STATE_FCS)
        {
            parse->cmd.fcs = data[pos];
            pos++;
            err = ZNP_STATUS_SUCCESS;
            parse->state = TI_ZNP_STATE_FIRST;
        }
    } while (0);
    *posi += pos;

    return err;
}

int znp_getCmd(uint8_t *data, int size)
{
    uint8_t tmp[512];
    int len = 0;
    int pos = 0;
    int err = ZNP_STATUS_FAIL;
    if (znpParse.remain > 0)
    {
        memcpy(tmp, znpParse.rdata, znpParse.remain);
        len = znpParse.remain;
        znpParse.remain = 0;
    }
    memcpy(tmp + len, data, size);
    len += size;

    // printf("data in :");
    int i;
    // for (i = 0; i < len; i++)
    // {
    //     printf("%02X ", tmp[i]);
    // }
    // printf("\n");
    while (pos < size)
    {
        // printf("do len, pos %d %d\n", len, pos);
        err = znp_parse(tmp, len, &znpParse, &pos);
        // printf("err %d\n", err);
        if (err == ZNP_STATUS_SUCCESS)
        {
            printf("\033[1;32mSOF [%02X], LEN [%02X], CMD [%04X], FCS [%02X], DATA:\033[0m ", znpParse.cmd.sof, znpParse.cmd.len, znpParse.cmd.cmd, znpParse.cmd.fcs); /* Parse data response */
            for (i = 0; i < znpParse.cmd.len; i++)
            {
                printf("\033[1;32m%02X \033[0m", znpParse.cmd.data[i]);
            }
            printf("\n\n");
        }
        if (err == ZNP_STATUS_CONTINUTE)
        {
            printf("znp continute len %d pos %d\n", len, pos);
            znpParse.remain = len - pos;
            memcpy(znpParse.rdata, tmp + pos, znpParse.remain);
            break;
        }
        else if (err == ZNP_STATUS_FAIL)
        {
            break;
        }
    }
}