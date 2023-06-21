/*  send a frame from can bus
    support@longan-labs.cc

    CAN Baudrate,

    #define CAN_5KBPS           1
    #define CAN_10KBPS          2
    #define CAN_20KBPS          3
    #define CAN_25KBPS          4
    #define CAN_31K25BPS        5
    #define CAN_33KBPS          6
    #define CAN_40KBPS          7
    #define CAN_50KBPS          8
    #define CAN_80KBPS          9
    #define CAN_83K3BPS         10
    #define CAN_95KBPS          11
    #define CAN_100KBPS         12
    #define CAN_125KBPS         13
    #define CAN_200KBPS         14
    #define CAN_250KBPS         15
    #define CAN_500KBPS         16
    #define CAN_666KBPS         17
    #define CAN_1000KBPS        18
*/

#include <mcp_can.h>
#include <SPI.h>
#include <SBWire.h>
#include <EEPROM.h>
#include "I2C_CAN_dfs.h"

#define  MAX_RECV_CAN_LEN       16          // BUF FOR CAN FRAME RECEIVING

unsigned char can_frame_dta[MAX_RECV_CAN_LEN][16]; // Buffer allows for 16 received can messages 
int cnt_can_frame_dta = 0;
int index_can_frame_dta = 0;
int index_can_frame_read = 0;

const int SPI_CS_PIN = 9;            // CAN Bus Shield

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

#define LEDON()     digitalWrite(3, HIGH)
#define LEDOFF()    digitalWrite(3, LOW)
#define LEDTOGGLE() digitalWrite(3, 1 - digitalRead(3))

unsigned char i2cWriteDataLength = 0;
unsigned char i2cWriteData[20];
unsigned char i2cWritePending = 0;

unsigned char reg_read = 0;

unsigned char makeCheckSum(unsigned char* dta, int len)
{
    unsigned long sum = 0;
    for (int i = 0; i < len; i++)sum += dta[i];

    if (sum > 0xff)
    {
        sum = ~sum;
        sum += 1;
    }

    sum = sum & 0xff;
    return sum;
}

void setup()
{
    pinMode(3, OUTPUT);
    Serial.begin(115200);

    for (int i = 0; i < 20; i++)
    {
        LEDTOGGLE();
        delay(20);
    }

    int __baud = EEPROM.read(REG_BAUD);

    if (__baud == 0 || __baud > 18)          // not setting baud rate
    {
        __baud = 16;
    }

    if (0x5a != EEPROM.read(REG_ADDR_SET))       // if not 0x5a, no set yet, set to default
    {
        EEPROM.write(REG_ADDR_SET, 0x5a);
        EEPROM.write(REG_ADDR, DEFAULT_I2C_ADDR);
    }

    // I2C setup
    Wire.begin(EEPROM.read(REG_ADDR));
    Wire.onReceive(handleI2CWrite);
    Wire.onRequest(handleI2CRead);

    while (CAN_OK != CAN.begin(__baud))    // init can bus : baudrate = 500k
    {
        delay(100);
        digitalWrite(3, 1 - digitalRead(3));
        Serial.println("CAN FAIL");
    }

    LEDON();

    WD_SET(WD_RST, WDTO_1S);
}

unsigned char stmp[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int cntBlink = 0;
void blink()
{
    static unsigned long timer_s = millis();
    if (millis() - timer_s < 100)return;
    timer_s = millis();

    if (cntBlink > 0)
    {
        digitalWrite(3, cntBlink % 2);
        cntBlink--;
    }
}

void loop()
{
    taskCANRecv();

    blink();

    WDR();

    if (i2cWritePending)
    {
        if (cntBlink == 0)cntBlink = 2;

        i2cWritePending = 0;
        if (i2cWriteDataLength > 0)

            switch (i2cWriteData[0])
            {
                //***********************SEND A FRAME***********************************
            case REG_SEND:              // send

                if (17 == i2cWriteDataLength)
                {
                    unsigned long id = 0;

                    id = i2cWriteData[1];
                    id <<= 8;
                    id += i2cWriteData[2];
                    id <<= 8;
                    id += i2cWriteData[3];
                    id <<= 8;
                    id += i2cWriteData[4];

                    int __len = i2cWriteData[7];
                    int __ext = i2cWriteData[5];

                    unsigned char __checksum = makeCheckSum(&i2cWriteData[1], 15);

                    if (__checksum == i2cWriteData[16])       // check sum ok
                    {
                        if (__len <= 8)
                        {
                            CAN.sendMsgBuf(id, __ext, __len, &i2cWriteData[8]);
                        }
                    }
                }

                break;
                //***********************SET BAUD******************************************
            case REG_BAUD:
                if (1 == i2cWriteDataLength)                // read
                {
                    reg_read = REG_BAUD;
                }
                else if (2 == i2cWriteDataLength)           // write
                {
                    if (i2cWriteData[1] >= 1 && i2cWriteData[1] <= 18)
                    {
                        EEPROM.write(REG_BAUD, i2cWriteData[1]);
                        while (CAN_OK != CAN.begin(i2cWriteData[1]))    // init can bus : baudrate = 500k
                        {
                            delay(100);
                        }
                    }
                }

                break;
                /************************GET CAN FRAME NUMBER******************************/
            case REG_DNUM:

                if (1 == i2cWriteDataLength)
                {
                    reg_read = REG_DNUM;
                }

                break;

                //***********************SET ADDR*******************************************
            case REG_ADDR:

                if (2 == i2cWriteDataLength)
                {
                    EEPROM.write(REG_ADDR, i2cWriteData[1]);
                    while (1);
                }

                break;
                //***********************GET CAN FRAME**************************************
            case REG_RECV:

                if (1 == i2cWriteDataLength)
                {
                    reg_read = REG_RECV;
                }

                break;

                //***********************MASK0*********************************************
            case REG_MASK0:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_MASK0;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_MASK0 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long mask = i2cWriteData[2];
                    mask <<= 8;
                    mask += i2cWriteData[3];
                    mask <<= 8;
                    mask += i2cWriteData[4];
                    mask <<= 8;
                    mask += i2cWriteData[5];

                    CAN.init_Mask(0, i2cWriteData[1], mask);
                }

                break;
                //***********************MASK1*********************************************
            case REG_MASK1:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_MASK1;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_MASK1 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long mask = i2cWriteData[2];
                    mask <<= 8;
                    mask += i2cWriteData[3];
                    mask <<= 8;
                    mask += i2cWriteData[4];
                    mask <<= 8;
                    mask += i2cWriteData[5];

                    CAN.init_Mask(1, i2cWriteData[1], mask);
                }

                break;
                //***********************FILTER 0*********************************************
            case REG_FILT0:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_FILT0;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT0 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long filt = i2cWriteData[2];
                    filt <<= 8;
                    filt += i2cWriteData[3];
                    filt <<= 8;
                    filt += i2cWriteData[4];
                    filt <<= 8;
                    filt += i2cWriteData[5];

                    CAN.init_Filt(0, i2cWriteData[1], filt);
                }

                break;

                //***********************FILTER 1*********************************************
            case REG_FILT1:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_FILT1;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT1 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long filt = i2cWriteData[2];
                    filt <<= 8;
                    filt += i2cWriteData[3];
                    filt <<= 8;
                    filt += i2cWriteData[4];
                    filt <<= 8;
                    filt += i2cWriteData[5];

                    CAN.init_Filt(1, i2cWriteData[1], filt);
                }

                break;
                //***********************FILTER 2*********************************************
            case REG_FILT2:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_FILT2;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT2 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long filt = i2cWriteData[2];
                    filt <<= 8;
                    filt += i2cWriteData[3];
                    filt <<= 8;
                    filt += i2cWriteData[4];
                    filt <<= 8;
                    filt += i2cWriteData[5];

                    CAN.init_Filt(2, i2cWriteData[1], filt);
                }

                break;
                //***********************FILTER 3*********************************************
            case REG_FILT3:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_FILT3;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT3 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long filt = i2cWriteData[2];
                    filt <<= 8;
                    filt += i2cWriteData[3];
                    filt <<= 8;
                    filt += i2cWriteData[4];
                    filt <<= 8;
                    filt += i2cWriteData[5];

                    CAN.init_Filt(3, i2cWriteData[1], filt);
                }

                break;
                //***********************FILTER 4*********************************************
            case REG_FILT4:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_FILT4;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT4 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long filt = i2cWriteData[2];
                    filt <<= 8;
                    filt += i2cWriteData[3];
                    filt <<= 8;
                    filt += i2cWriteData[4];
                    filt <<= 8;
                    filt += i2cWriteData[5];

                    CAN.init_Filt(4, i2cWriteData[1], filt);
                }

                break;
                //***********************FILTER 5*********************************************
            case REG_FILT5:

                if (1 == i2cWriteDataLength)            // read mask0
                {
                    reg_read = REG_FILT5;
                }
                else if (6 == i2cWriteDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT5 + i, i2cWriteData[1 + i]);
                    }

                    unsigned long filt = i2cWriteData[2];
                    filt <<= 8;
                    filt += i2cWriteData[3];
                    filt <<= 8;
                    filt += i2cWriteData[4];
                    filt <<= 8;
                    filt += i2cWriteData[5];

                    CAN.init_Filt(5, i2cWriteData[1], filt);
                }

                break;


            default:;
            }

        i2cWriteDataLength = 0;
    }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void handleI2CWrite(int howMany)
{
    while (0 < Wire.available()) { // loop through all but the last
        i2cWriteData[i2cWriteDataLength++] = Wire.read();
    }

    if (i2cWriteDataLength > 0) i2cWritePending = 1;

}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void handleI2CRead() {

    switch (reg_read)
    {
    case REG_BAUD:
        Wire.write(EEPROM.read(REG_BAUD));
        break;

    case REG_DNUM:

        Wire.write(cnt_can_frame_dta);
        break;

    case REG_RECV:

        if (cnt_can_frame_dta > 0)
        {
            for (int i = 0; i < 16; i++)
            {
                Wire.write(can_frame_dta[index_can_frame_read][i]);
            }

            index_can_frame_read++;
            if (index_can_frame_read >= MAX_RECV_CAN_LEN)index_can_frame_read = 0;

            cnt_can_frame_dta--;

        }

        break;

    case REG_MASK0:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_MASK0 + i));
        }
        break;

    case REG_MASK1:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_MASK1 + i));
        }
        break;

    case REG_FILT0:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_FILT0 + i));
        }
        break;

    case REG_FILT1:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_FILT1 + i));
        }
        break;

    case REG_FILT2:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_FILT1 + i));
        }
        break;

    case REG_FILT3:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_FILT3 + i));
        }
        break;

    case REG_FILT4:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_FILT4 + i));
        }
        break;

    case REG_FILT5:

        for (int i = 0; i < 5; i++)
        {
            Wire.write(EEPROM.read(REG_FILT5 + i));
        }
        break;

    default:;
    }
}

/*

unsigned char can_frame_dta[MAX_RECV_CAN_LEN][16];
int cnt_can_frame_dta   = 0;
int index_can_frame_dta = 0;

*/
void taskCANRecv()
{
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned long canId = CAN.getCanId();


        if (cnt_can_frame_dta < MAX_RECV_CAN_LEN)
        {
            cnt_can_frame_dta++;
        }
        else
        {
            index_can_frame_read++;
            if (index_can_frame_read >= MAX_RECV_CAN_LEN)index_can_frame_read = 0;
        }

        can_frame_dta[index_can_frame_dta][0] = (canId >> 24) & 0xff;
        can_frame_dta[index_can_frame_dta][1] = (canId >> 16) & 0xff;
        can_frame_dta[index_can_frame_dta][2] = (canId >> 8) & 0xff;
        can_frame_dta[index_can_frame_dta][3] = (canId >> 0) & 0xff;

        can_frame_dta[index_can_frame_dta][4] = CAN.isExtendedFrame();          // extend frame?
        can_frame_dta[index_can_frame_dta][5] = CAN.isRemoteRequest();          // remote frame?

        can_frame_dta[index_can_frame_dta][6] = len;

        for (int i = 0; i < len; i++)
        {
            can_frame_dta[index_can_frame_dta][7 + i] = buf[i];
        }

        can_frame_dta[index_can_frame_dta][15] = makeCheckSum(&can_frame_dta[index_can_frame_dta][0], 15);

        index_can_frame_dta++;
        if (index_can_frame_dta >= (MAX_RECV_CAN_LEN)) index_can_frame_dta = 0;

    }
}

// END FILE
