/*
 * File: main.c
 * Project: HAGateway-Zigbee
 * File Created: 20 July 2022 08:39:18
 * Author: KhangKT
 * AuthorEmail: khangkieutrong@gmail.com
 * Phone:0964991713
 */

#include "serial.h"
#include "app_serial.h"
#include "ti_znp.h"

int main(int argc, char *argv[])
{
    char c;
    int fd;
    int ID = 0;

    ti_znp_init();
    fd = Serial_start();

    printf("\n\033[1;33m_______________________Option_______________________ \033[0m\n\n"); 

    printf("\033[1;33mPress key board: \033[0m\n");
    printf("\t\t\033[1;33mA : Auto config \033[0m\n");
    printf("\t\t\033[1;33mI : Get device info \033[0m\n");
    printf("\t\t\033[1;33mJ : Start permit join \033[0m\n");

    printf("\n\033[1;33m_______________________END_______________________ \033[0m\n\n");

    while (1)
    {

        scanf("%s", &c);
        switch (c)
        {
        case DEVICE_INF: /// reset
        {
            printf("\n\033[1;36mGet Device Infomation \033[0m\n");
            Write_serial_CMD(fd, CMD_UTIL_GET_DEVICE_INFO);
            break;
        }
        case AUTO_CONFIG:
        {
            printf("\n\033[1;36mConfig Startup Option \033[0m\n");
            Write_serial_CMD(fd, CMD_ZCD_NV_STARTUP_OPTION);
            sleep(1);
            printf("\n\033[1;36mConfig Reset \033[0m\n");
            Write_serial_CMD(fd, CMD_ZB_SYSTEM_RESET);
            sleep(1);
            printf("\n\033[1;36mConfig Type Device:Coordinator \033[0m\n");
            Write_serial_CMD(fd, CMD_ZCD_NV_LOGICAL_TYPE);
            sleep(1);
            printf("\n\033[1;36mConfig PANID \033[0m\n");
            Write_serial_CMD(fd, CMD_ZCD_NV_PANID);
            sleep(1);
            printf("\n\033[1;36mConfig CHANNEL \033[0m\n");
            Write_serial_CMD(fd, CMD_ZCD_NV_SET_CHANNEL_20);
            sleep(1);
            Write_serial_CMD(fd, CMD_ZCD_NV_SET_CHANNEL_NONE);
            sleep(1);
            Write_serial_CMD(fd, CMD_ZB_START_REQUEST_1);
            sleep(1);
            printf("\n\033[1;36mConfig Direction Callback \033[0m\n");
            Write_serial_CMD(fd, CMD_ZCD_NV_ZDO_DIRECT_CB);
            sleep(1);
            printf("\n\033[1;36mConfig AF Register \033[0m\n");
            Write_serial_CMD(fd, CMD_AF_REGISTER);
            sleep(1);
            printf("\n\033[1;36mConfig Start Request \033[0m\n");
            Write_serial_CMD(fd, CMD_ZB_START_REQUEST_2);
            break;
        }
        case JOIN:
        {
            printf("\n\033[1;36mAPP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE: false \033[0m\n");
            Write_serial_CMD(fd, CMD_APP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE);

            printf("\n\033[1;36mPermit join require \033[0m\n");
            Write_serial_CMD(fd, CMD_ZDO_MGMT_PERMIT_JOIN_REQ);
            break;
        }
        case LEAVE:
        {
            Buff_Nwk_Req(BUFF_INFO); // refresh buff
            Write_serial_CMD(fd, CMD_ZB_LEAVE);

        }

        default:
            break;
        }
    }

    return 0;
}