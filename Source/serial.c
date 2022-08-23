
/*
 * File: serial.c
 * Project: HAGateway-Zigbee
 * File Created: 20 July 2022 08:39:18
 * Author: KhangKT
 * AuthorEmail: khangkieutrong@gmail.com
 * Phone:0964991713
 */

#include <time.h>
#include <pthread.h>

#include "app_serial.h"
#include "serial.h"
#include "ti_znp.h"

uint8_t NXP_chip[3] = {0x8d, 0x15, 0x00};
uint8_t TI_chip[3] = {0x4b, 0x12, 0x00};
uint8_t SiL_chip[3] = {0x81, 0xf6, 0x8c};
uint8_t CMD_AF_REGISTER[] = {0x24, 0x00, 0x01, 0x04, 0x01, 0xff, 0xff, 0x00, 0x00, 0x06, 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x00, 0x05, 0x02, 0x00, 0x05, 0x00, 0x00, '\n'}; /* AppProfID = HA(0x0104)
                                                                                                                                                                                                         Cluster-in: 0x0000,0x0003,0x0004,0x0005,0x0006,0x0500.
                                                                                                                                                                                                         Cluster-out: 0x0000,0x0500.   */
uint8_t CMD_ZB_SYSTEM_RESET[] = {0x41, 0x00, '\n'};                                        /* Reset target device*/
uint8_t CMD_ZB_LEAVE[] = {0x25, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '\n'};
uint8_t CMD_ZB_START_REQUEST_1[] = {0x2f, 0x05, 0x04, '\n'};                               /* Start with Network formation */
uint8_t CMD_ZB_START_REQUEST_2[] = {0x2f, 0x05, 0x02, '\n'};                               /* Start with Network steering */
uint8_t CMD_ZCD_NV_STARTUP_OPTION[] = {0x21, 0x09, 0x03, 0x00, 0x01, 0x03, '\n'};
uint8_t CMD_ZCD_NV_LOGICAL_TYPE[] = {0x21, 0x09, 0x87, 0x00, 0x01, 0x00, '\n'};            /* Set target Coordinator */
uint8_t CMD_ZCD_NV_PANID[] = {0x26, 0x05, 0x83, 0x02, 0x22, 0x06, '\n'};                   /* Set PANID: 0622 */
uint8_t CMD_ZCD_NV_SET_CHANNEL_20[] = {0x2f, 0x08, 0x01, 0x00, 0xF8, 0xFF, 0x07, '\n'};    /* Set all channel */
uint8_t CMD_ZCD_NV_SET_CHANNEL_NONE[] = {0x2f, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, '\n'};  /* Set channel none */
uint8_t CMD_ZCD_NV_ZDO_DIRECT_CB[] = {0x21, 0x09, 0x8F, 0x00, 0x01, 0x01, '\n'};           /* Set direction  */
uint8_t CMD_APP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE[] = {0x2F, 0x09, 0x00, '\n'};          /* Request don't exchange key */
uint8_t CMD_ZDO_MGMT_PERMIT_JOIN_REQ[] = {0x25, 0x36, 0x02, 0x00, 0x00, 0x78, 0x00, '\n'}; /* Request permit join */
uint8_t CMD_UTIL_GET_DEVICE_INFO[] = {0x27, 0x00, '\n'};                                   /* Information Device */

uint8_t BUFF_INFO[10];
uint8_t num = 0;

int Serial_open()
{
    printf("Serial open port :%s \n", Serial_Port);
    int ret;
    int fd = open(Serial_Port, O_RDWR | O_NOCTTY);
    if (fd < 0) /* Check serial port */
    {
        fprintf(stderr, "\033[1;31m__________________________Error scan port__________________________\033[0m\n");
        return -1;
    }
    printf("\033[1;36m__________Serial open success________\033[0m\n\n");

    ret = Serial_Set(fd, Serial_BR, 0);
    if (ret != 0)
    {
        printf("Serial_Set ERROR \n");
        return -1;
    }

    return fd;
}

void Serial_Close(int fd)
{
    close(fd);
}

static int Serial_Set(int fd, int speed, int parity)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;        // no signaling chars, no echo,
    // / setup for non-canonical mode /
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag = 0;     // no remapping, no delays
    tty.c_cc[VMIN] = 0;  // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                       // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        return -1;
    }
    return 0;
}
unsigned int my_strlen(char *p) /* Check buffer len */
{
    unsigned int count = 0;
    while (*p != '\n')
    {
        count++;
        p++;
    }
    return count;
}

void Print_Device(uint8_t num)/* Print SrcAddr and IEEE Address of new device */
{
    if (num > 0)
    {
        for (int count = 0; count < num; count++)
        {
            printf("\033[1;32mSrc_Address: \033[0m");
            for (int i = 0; i < 2; i++)
            {
                BUFF_INFO[i] = Number_Device[count].NwK_Address[i];
                printf("\033[1;32m%02X \033[0m", Number_Device[count].NwK_Address[i]);
            }
            printf("\n");
            printf("\033[1;32mDevice_Address: \033[0m");
            for (int i = 0; i < 8; i++)
            {
                BUFF_INFO[i + 2] = Number_Device[count].Device_Address[i];
                printf("\033[1;32m%02X \033[0m", Number_Device[count].Device_Address[i]);
            }
            printf("\n");
        }
    }
}

uint8_t *Buff_Nwk_Req(char Buff_Leave[])
{
    for (int i = 0; i < 10; i++)
    {
        CMD_ZB_LEAVE[i + 2] = Buff_Leave[i];
    }
    for (int i = 0; i < 13; i++)
    {
        printf("\033[1;31m%02X \033[0m", CMD_ZB_LEAVE[i]);
    }
    printf("\n");
    return CMD_ZB_LEAVE;
}

void Get_Info_ZED(uint8_t Buff_receive[])
{
    uint8_t Check_Rsp[] = {0xfe, 0x0c, 0x45, 0xca};
    if (memcmp(Buff_receive, Check_Rsp, 4) == 0)
    {
        uint8_t Check_chip[3];
        Number_Device[num].NwK_Address[0] = Buff_receive[4];
        Number_Device[num].NwK_Address[1] = Buff_receive[5];
        for (int i = 0; i < 8; i++)
        {
            Number_Device[num].Device_Address[i] = Buff_receive[i + 6];
        }
        Check_chip[0] = Buff_receive[11];
        Check_chip[1] = Buff_receive[12];
        Check_chip[2] = Buff_receive[13];
        printf("\033[1;32m___________NEW DEVICE INFO ___________\033[0m\n");
        if (memcmp(Check_chip, TI_chip, 3) == 0)
        {
            printf("\033[1;32mChip: Texas_Instruments\033[0m\n");
        }
        else if (memcmp(Check_chip, SiL_chip, 3) == 0)
        {
            printf("\033[1;32mChip: Silicon_Labs\033[0m\n");
        }
        else if (memcmp(Check_chip, NXP_chip, 3) == 0)
        {
            printf("\033[1;32mChip: NXP\033[0m\n");
        }
        //num++;
        Print_Device(1);
    }
}

void Data_Rsp(uint8_t Buff_receive[])
{

    if ((Buff_receive[2] == 0x44) && (Buff_receive[3] == 0x81))
    {
        printf("\033[1;32m__________AF_INCOMING_MSG__________\033[0m\n");

        printf("\033[1;32mGroupID:   %02X\033[0m", Buff_receive[4]);
        printf("\033[1;32m %02X\033[0m\n", Buff_receive[5]);

        printf("\033[1;32mClusterID: \033[0m");                     /* cluster name */
        if ((Buff_receive[6] == 0x00) && (Buff_receive[7] == 0x05)) /* IAS Cluster */
        {
            printf("\033[1;32mIAS Zone(%02X\033[0m", Buff_receive[6]);
            printf("\033[1;32m %02X)\033[0m\n", Buff_receive[7]);
        }
        else if ((Buff_receive[6] == 0x00) && (Buff_receive[7] == 0x00)) /* Basis cluster */
        {
            printf("\033[1;32mBASIC(%02X\033[0m", Buff_receive[6]);
            printf("\033[1;32m %02X)\033[0m\n", Buff_receive[7]);
        }
        else if ((Buff_receive[6] == 0x06) && (Buff_receive[7] == 0x04)) /* Occupancy Sensing */
        {
            printf("\033[1;32mOccupancy(%02X\033[0m", Buff_receive[6]);
            printf("\033[1;32m %02X)\033[0m\n", Buff_receive[7]);
        }
        else if ((Buff_receive[6] == 0x01) && (Buff_receive[7] == 0x00)) /* Power Configuration */
        {
            printf("\033[1;32mPower Configuration(%02X\033[0m", Buff_receive[6]);
            printf("\033[1;32m %02X)\033[0m\n", Buff_receive[7]);
            printf("\033[1;33mPower:%.2f %% \033[0m\n\n", ((float)Buff_receive[27] * 100) / 200);
        }
        else if ((Buff_receive[6] == 0x05) && (Buff_receive[7] == 0x04)) /* Humidity Measurement Cluster */
        {
            printf("\033[1;32mHumidity Measurement(%02X\033[0m", Buff_receive[6]);
            printf("\033[1;32m %02X)\033[0m\n", Buff_receive[7]);
            printf("\033[1;33mValue:%d.\033[0m", (((Buff_receive[28] << 8) | Buff_receive[27])) / 100);
            printf("\033[1;33m%d %% \033[0m", (((Buff_receive[28] << 8) | Buff_receive[27])) % 100);
            printf("\033[1;33m(%4X) \033[0m\n", ((Buff_receive[28] << 8) | Buff_receive[27]));
        }
        else if ((Buff_receive[6] == 0x02) && (Buff_receive[7] == 0x04))
        {
            printf("\033[1;32mTemperature Measurement (%02X\033[0m", Buff_receive[6]); /* Temperature Measurement cluster */
            printf("\033[1;32m %02X)\033[0m\n", Buff_receive[7]);
            printf("\033[1;33mValue:%d.\033[0m", (((Buff_receive[28] << 8) | Buff_receive[27])) / 100);
            printf("\033[1;33m%d °C \033[0m", (((Buff_receive[28] << 8) | Buff_receive[27])) % 100);
            printf("\033[1;33m(%4X) \033[0m\n", ((Buff_receive[28] << 8) | Buff_receive[27]));
        }

        printf("\033[1;32mSrcAddr:   %02X\033[0m", Buff_receive[8]); /* Source Address */
        printf("\033[1;32m %02X\033[0m\n", Buff_receive[9]);
        printf("\033[1;32mSrcEndpoint:  %02X\033[0m\n", Buff_receive[10]); /* Source Endpoint */
        printf("\033[1;32mDstEndpoint:  %02X\033[0m\n", Buff_receive[11]); /* Destination Endpoint*/
        printf("\033[1;32mWasBroadcast: %02X\033[0m\n", Buff_receive[12]); /* WasBroadcast */
        printf("\033[1;32mLinkQuality:  %02X\033[0m\n", Buff_receive[13]); /* LinkQuality */
        printf("\033[1;32mSecurityUse:  %02X\033[0m\n", Buff_receive[14]); /* SecurityUse  */
        printf("\033[1;32mTimestamp:    %02X\033[0m", Buff_receive[15]);   /* 32mTimestamp  */
        printf("\033[1;32m %02X\033[0m", Buff_receive[16]);
        printf("\033[1;32m %02X\033[0m", Buff_receive[17]);
        printf("\033[1;32m %02X\033[0m\n", Buff_receive[18]);
        printf("\033[1;32mTransSeqNumber: %02X\033[0m\n", Buff_receive[19]);/* TransSeqNumber */
        printf("\033[1;32mLen_Data: %02X\033[0m\n", Buff_receive[20]);
        printf("\033[1;32mData: \033[0m"); 
        for (int i = 0; i < Buff_receive[20]; i++)
        {
            printf("\033[1;32m%02X \033[0m", Buff_receive[21 + i]);
        }
        printf("\n");
        if ((Buff_receive[6] == 0x00) && (Buff_receive[7] == 0x05))
        {
            if (((Buff_receive[24] >> 0) & 0x01) != 1)
                printf("\033[1;33mNot Alarmed:%02X \033[0m\n\n", ((Buff_receive[24] >> 0) & 0x01));
            else
                printf("\033[1;33mAlarmed:%02X \033[0m\n\n", ((Buff_receive[24] >> 0) & 0x01));
        }
        else if ((Buff_receive[6] == 0x06) && (Buff_receive[7] == 0x00))
        {
            if (((Buff_receive[27] >> 0) & 0x01) != 1)
                printf("\033[1;33mNot Alarmed:%02X \033[0m\n\n", ((Buff_receive[27] >> 0) & 0x01));
            else
                printf("\033[1;33mAlarmed:%02X \033[0m\n\n", ((Buff_receive[27] >> 0) & 0x01));
        }
        else if ((Buff_receive[6] == 0x06) && (Buff_receive[7] == 0x04) && ((Buff_receive[27] >> 0) & 0x01) == 1)
            printf("\033[1;33mDetect motion(%02X)\033[0m\n\n", (Buff_receive[27] >> 0) & 0x01);
    }
    else if ((Buff_receive[2] == 0x45) && (Buff_receive[3] == 0xc9))
    {
        printf("\033[1;32m__________LEAVE DEVICE__________\033[0m\n");

        printf("\033[1;32mSrcAddr: %02X\033[0m", Buff_receive[4]);
        printf("\033[1;32m %02X\033[0m\n", Buff_receive[5]);
        printf("\033[1;32mDeviceAddr: %02X\033[0m", Buff_receive[6]);
        printf("\033[1;32m %02X\033[0m", Buff_receive[7]);
        printf("\033[1;32m %02X\033[0m", Buff_receive[8]);
        printf("\033[1;32m %02X\033[0m", Buff_receive[9]);
        printf("\033[1;32m %02X\033[0m", Buff_receive[10]);
        printf("\033[1;32m %02X\033[0m", Buff_receive[11]);
        printf("\033[1;32m %02X\033[0m", Buff_receive[12]);
        printf("\033[1;32m %02X\033[0m\n", Buff_receive[13]);
    }
}

void *Serial_receive_handle(void *arg)
{
    int i;
    int fd = *(int *)arg;
    unsigned char buf[256];
    int len;
    time_t ltime;                                                                        /* calendar time */
    char Reset[11] = {0xfe, 0x06, 0x41, 0x80, 0x00, 0x02, 0x00, 0x02, 0x07, 0x02, 0xc2}; // Response reset
    while (1)
    {
        ltime = time(NULL);
        len = read(fd, buf, sizeof(buf));
        if (len == 1 && buf[0] == 0)
        {
            printf("Zigbee Reinstall UART!!! Maybe device just restarted!!!\n");
            continue;
        }
        if (len > 0)
        {
            printf("\033[1;33m%sSerial response:\033[0m", asctime(localtime(&ltime)));
            /* get current cal time */
            for (i = 0; i < len; i++)
            {
                printf("\033[1;35m%02X \033[0m", buf[i]);
            }
            printf("\n\n");
            int err = znp_getCmd(buf, len);
            // Auto restart if lost power
            if (memcmp(buf, Reset, 11) == 0)
            {
                printf("\n\033[1;36mAuto Restart Network \033[0m\n");
                Write_serial_CMD(fd, CMD_ZB_START_REQUEST_1);
                Write_serial_CMD(fd, CMD_AF_REGISTER);
            }
            /* process response */
            Get_Info_ZED(buf);
            Data_Rsp(buf);

        }
    }
}

int Serial_write(int fd, unsigned char opcode, unsigned char *payload, unsigned char size)
{
    int i;
    unsigned char buf[255];
    int len = size + 3;
    buf[0] = opcode;
    buf[1] = size - 2;
    int FCS = 0;
    if (size > 0)
    {
        memcpy(buf + 2, payload, size);

        for (int i = 1; i < size + 2; i++)
        {
            FCS ^= buf[i];
        }
        buf[size + 2] = FCS; /// FCS is Frame Check Sequence
    }
    printf("\033[1;33mSerial request: \033[0m");
    for (i = 0; i < len; i++)
    {
        printf("\033[1;32m%02X \033[0m", buf[i]);
    }
    printf("\n");

    return write(fd, buf, len);
}

int Serial_start()
{
    pthread_t thread;
    int fd;

    fd = Serial_open(); // check open status

    sleep(2); // sleep 2s for receive reset

    if (pthread_create(&thread, NULL, Serial_receive_handle, (void *)&fd) != 0)
    {

        printf("Error thread...\n");
        return -1;
    }
    printf("\033[1;36mSerial write reset success \033[0m\n");

    return fd;
}
