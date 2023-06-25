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

#define  CAN_FRAMES_BUFFER_SIZE 16          // BUF FOR CAN FRAME RECEIVING, Buffer allows for 16 received can messages

#define SPI_CS_PIN 9            // CAN Bus Shield
#define LED_PIN 3
#define SERIAL_BAUD_RATE 11520

#define LEDON()     digitalWrite(LED_PIN, HIGH)
#define LEDOFF()    digitalWrite(LED_PIN, LOW)
#define LEDTOGGLE() digitalWrite(LED_PIN, 1 - digitalRead(LED_PIN))

MCP_CAN CAN(SPI_CS_PIN);

unsigned char canFramesBuffer[CAN_FRAMES_BUFFER_SIZE][16];
int canFramesCount = 0;
int canFramesWriteIndex = 0;
int canFramesReadIndex = 0;

unsigned char i2cDataLength = 0;
unsigned char i2cData[20];
unsigned char i2cDataReceived = FALSE;

unsigned char i2cReadRequest = 0;

int blinkCount = 0;
unsigned long lastBlinkTime = millis();

unsigned char getCheckSum(unsigned char* dta, int len)
{
    unsigned long sum = 0;
    for (int i = 0; i < len; i++) sum += dta[i];

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
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(SERIAL_BAUD_RATE);

    for (int i = 0; i < 20; i++)
    {
        LEDTOGGLE();
        delay(20);
    }

    int canBaud = EEPROM.read(REG_BAUD);
    if (canBaud < CAN_5KBPS || canBaud > CAN_1000KBPS) canBaud = CAN_500KBPS;


    if (0x5a != EEPROM.read(REG_ADDR_SET))
    {
        EEPROM.write(REG_ADDR_SET, 0x5a);
        EEPROM.write(REG_ADDR, DEFAULT_I2C_ADDR);
    }

    // I2C setup
    Wire.begin(EEPROM.read(REG_ADDR));
    Wire.onReceive(handleI2CWrite);
    Wire.onRequest(handleI2CRead);

    while (CAN_OK != CAN.begin(canBaud))
    {
        delay(100);
        LEDTOGGLE();
        Serial.println("CAN FAIL");
    }

    LEDON();

    WD_SET(WD_RST, WDTO_1S);
}

#define blinkLed()\
    if (millis() - lastBlinkTime >= 100) {\
        lastBlinkTime = millis();\
        if (blinkCount > 0) {\
            digitalWrite(LED_PIN, blinkCount % 2);\
            blinkCount--;\
        }\
    }\

#define processMaskOrFilterRequest(request)\
    if (1 == i2cDataLength) i2cReadRequest = request;\
    if (6 != i2cDataLength) break;\
    for (int i = 0; i < 5; i++) EEPROM.write(request + i, i2cData[1 + i]);\
    unsigned long newMaskOrFilter = i2cData[5] << 24 | i2cData[4] << 16 | i2cData[3] << 8 | i2cData[2];\

void loop()
{
    receiveCanFrame();

    blinkLed();

    WDR();

    if (i2cDataReceived == FALSE) return;

    if (blinkCount == 0) blinkCount = 2;

    i2cDataReceived = FALSE;

    if (i2cDataLength == 0) return;

    switch (i2cData[0]) {

    case REG_ADDR: {
        if (i2cDataLength != 2) break;
        EEPROM.write(REG_ADDR, i2cData[1]);
        while (TRUE);
        break;
    }

    case REG_DNUM: {
        if (i2cDataLength == 1) i2cReadRequest = REG_DNUM;
        break;
    }

    case REG_BAUD: {
        if (i2cDataLength == 1) i2cReadRequest = REG_BAUD;
        if (i2cDataLength != 2) break;
        if (i2cData[1] < CAN_5KBPS || i2cData[1] > CAN_1000KBPS) break;

        while (CAN_OK != CAN.begin(i2cData[1])) delay(100);
        EEPROM.write(REG_BAUD, i2cData[1]);
        break;
    }

    case REG_SEND: {
        if (i2cDataLength != 17) break;

        unsigned char checksum = getCheckSum(&i2cData[1], 15);
        if (checksum != i2cData[16] || i2cData[7] > 8) break;

        unsigned long frameId = i2cData[4] << 24 | i2cData[3] << 16 | i2cData[2] << 8 | i2cData[1];
        CAN.sendMsgBuf(frameId, i2cData[5], i2cData[7], &i2cData[8]);
        break;
    }

    case REG_RECV: {
        if (1 == i2cDataLength) i2cReadRequest = REG_RECV;
        break;
    }

    case REG_MASK0: {
        processMaskOrFilterRequest(REG_MASK0);
        CAN.init_Mask(0, i2cData[1], newMaskOrFilter);
        break;
    }

    case REG_MASK1: {
        processMaskOrFilterRequest(REG_MASK1);
        CAN.init_Mask(1, i2cData[1], newMaskOrFilter);
        break;
    }

    case REG_FILT0: {
        processMaskOrFilterRequest(REG_FILT0);
        CAN.init_Filt(0, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT1: {
        processMaskOrFilterRequest(REG_FILT1);
        CAN.init_Filt(1, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT2: {
        processMaskOrFilterRequest(REG_FILT2);
        CAN.init_Filt(2, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT3: {
        processMaskOrFilterRequest(REG_FILT3);
        CAN.init_Filt(3, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT4: {
        processMaskOrFilterRequest(REG_FILT4);
        CAN.init_Filt(4, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT5: {
        processMaskOrFilterRequest(REG_FILT5);
        CAN.init_Filt(5, i2cData[1], newMaskOrFilter);
        break;
    }

    default:
        break;
    }

    i2cDataLength = 0;
}

void handleI2CWrite(int howMany)
{
    while (Wire.available() > 0) i2cData[i2cDataLength++] = Wire.read();
    if (i2cDataLength > 0) i2cDataReceived = TRUE;
}

#define sendMaskOrFilter(regIndex) {\
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

        for (int i = 0; i < 16; i++) Wire.write(canFramesBuffer[canFramesReadIndex][i]);

        canFramesReadIndex++;
        if (canFramesReadIndex >= CAN_FRAMES_BUFFER_SIZE) canFramesReadIndex = 0;

        canFramesCount--;
        break;
    }

    case REG_MASK0: sendMaskOrFilter(REG_MASK0);
    case REG_MASK1: sendMaskOrFilter(REG_MASK1);
    case REG_FILT0: sendMaskOrFilter(REG_FILT0);
    case REG_FILT1: sendMaskOrFilter(REG_FILT1);
    case REG_FILT2: sendMaskOrFilter(REG_FILT2);
    case REG_FILT3: sendMaskOrFilter(REG_FILT3);
    case REG_FILT4: sendMaskOrFilter(REG_FILT4);
    case REG_FILT5: sendMaskOrFilter(REG_FILT5);

    default:
        break;
    }
}

void receiveCanFrame()
{
    if (CAN.checkReceive() != CAN_MSGAVAIL) return;

    unsigned char frameLength = 0;
    unsigned char buf[8];
    CAN.readMsgBuf(&frameLength, buf);

    unsigned long canId = CAN.getCanId();

    if (canFramesCount < CAN_FRAMES_BUFFER_SIZE)
    {
        canFramesCount++;
    }
    else
    {
        canFramesReadIndex++;
        if (canFramesReadIndex >= CAN_FRAMES_BUFFER_SIZE) canFramesReadIndex = 0;
    }

    canFramesBuffer[canFramesWriteIndex][0] = (canId >> 24) & 0xff;
    canFramesBuffer[canFramesWriteIndex][1] = (canId >> 16) & 0xff;
    canFramesBuffer[canFramesWriteIndex][2] = (canId >> 8) & 0xff;
    canFramesBuffer[canFramesWriteIndex][3] = (canId >> 0) & 0xff;

    canFramesBuffer[canFramesWriteIndex][4] = CAN.isExtendedFrame();
    canFramesBuffer[canFramesWriteIndex][5] = CAN.isRemoteRequest();

    canFramesBuffer[canFramesWriteIndex][6] = frameLength;

    for (int i = 0; i < frameLength; i++) canFramesBuffer[canFramesWriteIndex][7 + i] = buf[i];

    canFramesBuffer[canFramesWriteIndex][15] = getCheckSum(&canFramesBuffer[canFramesWriteIndex][0], 15);

    canFramesWriteIndex++;
    if (canFramesWriteIndex >= (CAN_FRAMES_BUFFER_SIZE)) canFramesWriteIndex = 0;
}
