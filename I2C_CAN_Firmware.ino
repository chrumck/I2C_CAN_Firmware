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

unsigned char canFrames[MAX_RECV_CAN_LEN][16]; // Buffer allows for 16 received can messages 
int canFramesCount = 0;
int canFramesWriteIndex = 0;
int canFramesReadIndex = 0;

const int SPI_CS_PIN = 9;            // CAN Bus Shield
MCP_CAN CAN(SPI_CS_PIN);             // Set CS pin

#define LEDON()     digitalWrite(3, HIGH)
#define LEDOFF()    digitalWrite(3, LOW)
#define LEDTOGGLE() digitalWrite(3, 1 - digitalRead(3))

unsigned char i2cDataLength = 0;
unsigned char i2cData[20];
unsigned char i2cDataReceived = FALSE;

unsigned char i2cReadRequest = 0;

unsigned char getCheckSum(unsigned char* dta, int len)
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

int blinkCount = 0;
void blink()
{
    static unsigned long lastBlinkTime = millis();

    if (millis() - lastBlinkTime < 100) return;

    lastBlinkTime = millis();
    if (blinkCount > 0)
    {
        digitalWrite(3, blinkCount % 2);
        blinkCount--;
    }
}

void loop()
{
    receiveCanFrame();

    blink();

    WDR();

    if (i2cDataReceived)
    {
        if (blinkCount == 0) blinkCount = 2;

        i2cDataReceived = 0;

        if (i2cDataLength > 0) {

            switch (i2cData[0])
            {
                //***********************SEND A FRAME***********************************
            case REG_SEND:              // send

                if (i2cDataLength != 17) break;

                unsigned long frameId = 0;

                frameId = i2cData[1];
                frameId <<= 8;
                frameId += i2cData[2];
                frameId <<= 8;
                frameId += i2cData[3];
                frameId <<= 8;
                frameId += i2cData[4];

                int __len = i2cData[7];
                int __ext = i2cData[5];

                unsigned char __checksum = getCheckSum(&i2cData[1], 15);

                if (__checksum == i2cData[16])       // check sum ok
                {
                    if (__len <= 8)
                    {
                        CAN.sendMsgBuf(frameId, __ext, __len, &i2cData[8]);
                    }
                }

                break;

                //***********************SET BAUD******************************************
            case REG_BAUD:
                if (i2cDataLength == 1)                // read
                {
                    i2cReadRequest = REG_BAUD;
                }
                else if (2 == i2cDataLength)           // write
                {
                    if (i2cData[1] >= 1 && i2cData[1] <= 18)
                    {
                        EEPROM.write(REG_BAUD, i2cData[1]);
                        while (CAN_OK != CAN.begin(i2cData[1]))    // init can bus : baudrate = 500k
                        {
                            delay(100);
                        }
                    }
                }

                break;

                /************************GET CAN FRAME NUMBER******************************/
            case REG_DNUM:
                if (i2cDataLength == 1) i2cReadRequest = REG_DNUM;
                break;

                //***********************SET ADDR*******************************************
            case REG_ADDR:

                if (2 == i2cDataLength)
                {
                    EEPROM.write(REG_ADDR, i2cData[1]);
                    while (1);
                }

                break;
                //***********************GET CAN FRAME**************************************
            case REG_RECV:

                if (i2cDataLength == 1)
                {
                    i2cReadRequest = REG_RECV;
                }

                break;

                //***********************MASK0*********************************************
            case REG_MASK0:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_MASK0;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_MASK0 + i, i2cData[1 + i]);
                    }

                    unsigned long mask = i2cData[2];
                    mask <<= 8;
                    mask += i2cData[3];
                    mask <<= 8;
                    mask += i2cData[4];
                    mask <<= 8;
                    mask += i2cData[5];

                    CAN.init_Mask(0, i2cData[1], mask);
                }

                break;
                //***********************MASK1*********************************************
            case REG_MASK1:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_MASK1;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_MASK1 + i, i2cData[1 + i]);
                    }

                    unsigned long mask = i2cData[2];
                    mask <<= 8;
                    mask += i2cData[3];
                    mask <<= 8;
                    mask += i2cData[4];
                    mask <<= 8;
                    mask += i2cData[5];

                    CAN.init_Mask(1, i2cData[1], mask);
                }

                break;
                //***********************FILTER 0*********************************************
            case REG_FILT0:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_FILT0;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT0 + i, i2cData[1 + i]);
                    }

                    unsigned long filt = i2cData[2];
                    filt <<= 8;
                    filt += i2cData[3];
                    filt <<= 8;
                    filt += i2cData[4];
                    filt <<= 8;
                    filt += i2cData[5];

                    CAN.init_Filt(0, i2cData[1], filt);
                }

                break;

                //***********************FILTER 1*********************************************
            case REG_FILT1:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_FILT1;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT1 + i, i2cData[1 + i]);
                    }

                    unsigned long filt = i2cData[2];
                    filt <<= 8;
                    filt += i2cData[3];
                    filt <<= 8;
                    filt += i2cData[4];
                    filt <<= 8;
                    filt += i2cData[5];

                    CAN.init_Filt(1, i2cData[1], filt);
                }

                break;
                //***********************FILTER 2*********************************************
            case REG_FILT2:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_FILT2;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT2 + i, i2cData[1 + i]);
                    }

                    unsigned long filt = i2cData[2];
                    filt <<= 8;
                    filt += i2cData[3];
                    filt <<= 8;
                    filt += i2cData[4];
                    filt <<= 8;
                    filt += i2cData[5];

                    CAN.init_Filt(2, i2cData[1], filt);
                }

                break;
                //***********************FILTER 3*********************************************
            case REG_FILT3:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_FILT3;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT3 + i, i2cData[1 + i]);
                    }

                    unsigned long filt = i2cData[2];
                    filt <<= 8;
                    filt += i2cData[3];
                    filt <<= 8;
                    filt += i2cData[4];
                    filt <<= 8;
                    filt += i2cData[5];

                    CAN.init_Filt(3, i2cData[1], filt);
                }

                break;
                //***********************FILTER 4*********************************************
            case REG_FILT4:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_FILT4;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT4 + i, i2cData[1 + i]);
                    }

                    unsigned long filt = i2cData[2];
                    filt <<= 8;
                    filt += i2cData[3];
                    filt <<= 8;
                    filt += i2cData[4];
                    filt <<= 8;
                    filt += i2cData[5];

                    CAN.init_Filt(4, i2cData[1], filt);
                }

                break;
                //***********************FILTER 5*********************************************
            case REG_FILT5:

                if (i2cDataLength == 1)            // read mask0
                {
                    i2cReadRequest = REG_FILT5;
                }
                else if (6 == i2cDataLength)       // set mask0
                {
                    for (int i = 0; i < 5; i++)
                    {
                        EEPROM.write(REG_FILT5 + i, i2cData[1 + i]);
                    }

                    unsigned long filt = i2cData[2];
                    filt <<= 8;
                    filt += i2cData[3];
                    filt <<= 8;
                    filt += i2cData[4];
                    filt <<= 8;
                    filt += i2cData[5];

                    CAN.init_Filt(5, i2cData[1], filt);
                }

                break;


            default:;
            }
        }

        i2cDataLength = 0;
    }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void handleI2CWrite(int howMany)
{
    while (0 < Wire.available()) { // loop through all but the last
        i2cData[i2cDataLength++] = Wire.read();
    }

    if (i2cDataLength > 0) i2cDataReceived = TRUE;

}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void handleI2CRead() {

    switch (i2cReadRequest)
    {
    case REG_BAUD:
        Wire.write(EEPROM.read(REG_BAUD));
        break;

    case REG_DNUM:

        Wire.write(canFramesCount);
        break;

    case REG_RECV:

        if (canFramesCount > 0)
        {
            for (int i = 0; i < 16; i++)
            {
                Wire.write(canFrames[canFramesReadIndex][i]);
            }

            canFramesReadIndex++;
            if (canFramesReadIndex >= MAX_RECV_CAN_LEN)canFramesReadIndex = 0;

            canFramesCount--;

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

void receiveCanFrame()
{
    unsigned char frameLength = 0;
    unsigned char frameBuffer[8];

    if (CAN.checkReceive() == CAN_MSGAVAIL)
    {
        CAN.readMsgBuf(&frameLength, frameBuffer);

        unsigned long canId = CAN.getCanId();

        if (canFramesCount < MAX_RECV_CAN_LEN)
        {
            canFramesCount++;
        }
        else
        {
            canFramesReadIndex++;
            if (canFramesReadIndex >= MAX_RECV_CAN_LEN) canFramesReadIndex = 0;
        }

        canFrames[canFramesWriteIndex][0] = (canId >> 24) & 0xff;
        canFrames[canFramesWriteIndex][1] = (canId >> 16) & 0xff;
        canFrames[canFramesWriteIndex][2] = (canId >> 8) & 0xff;
        canFrames[canFramesWriteIndex][3] = (canId >> 0) & 0xff;

        canFrames[canFramesWriteIndex][4] = CAN.isExtendedFrame();
        canFrames[canFramesWriteIndex][5] = CAN.isRemoteRequest();

        canFrames[canFramesWriteIndex][6] = frameLength;

        for (int i = 0; i < frameLength; i++)
        {
            canFrames[canFramesWriteIndex][7 + i] = frameBuffer[i];
        }

        canFrames[canFramesWriteIndex][15] = getCheckSum(&canFrames[canFramesWriteIndex][0], 15);

        canFramesWriteIndex++;
        if (canFramesWriteIndex >= (MAX_RECV_CAN_LEN)) canFramesWriteIndex = 0;

    }
}
