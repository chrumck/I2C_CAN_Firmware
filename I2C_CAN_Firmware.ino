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

#define readWriteMaskOrFilter(regIndex) {\
    if (1 == i2cDataLength) i2cReadRequest = regIndex;\
    if (6 != i2cDataLength) break;\
    for (int i = 0; i < 5; i++) EEPROM.write(regIndex + i, i2cData[1 + i]);\
    unsigned long newMask = i2cData[5] << 24 | i2cData[4] << 16 | i2cData[3] << 8 | i2cData[2];\
    CAN.init_Mask(0, i2cData[1], newMask);\
    break;\
}\

void loop()
{
    receiveCanFrame();

    blink();

    WDR();

    if (i2cDataReceived == FALSE) return;

    if (blinkCount == 0) blinkCount = 2;

    i2cDataReceived = FALSE;

    if (i2cDataLength == 0) return;

    switch (i2cData[0]) {

    case REG_ADDR: { // set i2c address
        if (i2cDataLength != 2) break;

        EEPROM.write(REG_ADDR, i2cData[1]);
        while (TRUE);
        break;
    }

    case REG_DNUM: { // get number of CAN frames available
        if (i2cDataLength == 1) i2cReadRequest = REG_DNUM;
        break;
    }

    case REG_BAUD: { // get/set CAN baud rate
        if (i2cDataLength == 1) i2cReadRequest = REG_BAUD;
        if (i2cDataLength != 2) break;
        if (i2cData[1] < 1 || i2cData[1] > 18) break;

        EEPROM.write(REG_BAUD, i2cData[1]);
        while (CAN_OK != CAN.begin(i2cData[1])) delay(100);
        break;
    }

    case REG_SEND: { // send CAN frame
        if (i2cDataLength != 17) break;

        int frameLength = i2cData[7];
        unsigned char checksum = getCheckSum(&i2cData[1], 15);
        if (checksum != i2cData[16] || frameLength > 8) break;

        unsigned long frameId = i2cData[4] << 24 | i2cData[3] << 16 | i2cData[2] << 8 | i2cData[1];
        CAN.sendMsgBuf(frameId, i2cData[5], frameLength, &i2cData[8]);
        break;
    }

    case REG_RECV: { // get CAN frame
        if (1 == i2cDataLength) i2cReadRequest = REG_RECV;
        break;
    }

    case REG_MASK0: readWriteMaskOrFilter(REG_MASK0);
    case REG_MASK1: readWriteMaskOrFilter(REG_MASK1);

    case REG_FILT0: readWriteMaskOrFilter(REG_FILT0);
    case REG_FILT1: readWriteMaskOrFilter(REG_FILT1);
    case REG_FILT2: readWriteMaskOrFilter(REG_FILT2);
    case REG_FILT3: readWriteMaskOrFilter(REG_FILT3);
    case REG_FILT4: readWriteMaskOrFilter(REG_FILT4);
    case REG_FILT5: readWriteMaskOrFilter(REG_FILT5);

    default: break;
    }

    i2cDataLength = 0;
}

void handleI2CWrite(int howMany)
{
    while (Wire.available() > 0) i2cData[i2cDataLength++] = Wire.read();
    if (i2cDataLength > 0) i2cDataReceived = TRUE;
}

#define writeFilterOrMask(regIndex) {\
     for (int i = 0; i < 5; i++) Wire.write(EEPROM.read(regIndex + i));\
     break;\
}\

void handleI2CRead() {

    switch (i2cReadRequest) {

    case REG_BAUD: {
        Wire.write(EEPROM.read(REG_BAUD));
        break;
    }

    case REG_DNUM: {
        Wire.write(canFramesCount);
        break;
    }

    case REG_RECV: {
        if (canFramesCount <= 0) break;

        for (int i = 0; i < 16; i++) Wire.write(canFrames[canFramesReadIndex][i]);

        canFramesReadIndex++;
        if (canFramesReadIndex >= MAX_RECV_CAN_LEN) canFramesReadIndex = 0;

        canFramesCount--;
        break;
    }

    case REG_MASK0: writeFilterOrMask(EEPROM.read(REG_MASK0 + i));
    case REG_MASK1: writeFilterOrMask(EEPROM.read(REG_MASK1 + i));
    case REG_FILT0: writeFilterOrMask(EEPROM.read(REG_FILT0 + i));
    case REG_FILT1: writeFilterOrMask(EEPROM.read(REG_FILT1 + i));
    case REG_FILT2: writeFilterOrMask(EEPROM.read(REG_FILT2 + i));
    case REG_FILT3: writeFilterOrMask(EEPROM.read(REG_FILT3 + i));
    case REG_FILT4: writeFilterOrMask(EEPROM.read(REG_FILT4 + i));
    case REG_FILT5: writeFilterOrMask(EEPROM.read(REG_FILT5 + i));

    default:
        break;
    }
}

void receiveCanFrame()
{
    unsigned char frameLength = 0;
    unsigned char buf[8];

    if (CAN.checkReceive() != CAN_MSGAVAIL) return;

    CAN.readMsgBuf(&frameLength, buf);

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

    for (int i = 0; i < frameLength; i++) canFrames[canFramesWriteIndex][7 + i] = buf[i];

    canFrames[canFramesWriteIndex][15] = getCheckSum(&canFrames[canFramesWriteIndex][0], 15);

    canFramesWriteIndex++;
    if (canFramesWriteIndex >= (MAX_RECV_CAN_LEN)) canFramesWriteIndex = 0;
}
