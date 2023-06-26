#include <mcp_can.h>
#include <SPI.h>
#include <SBWire.h>
#include <EEPROM.h>
#include "I2C_CAN_dfs.h"

#define  CAN_FRAME_SIZE 16
#define  CAN_FRAMES_BUFFER_SIZE 14

#define  CAN_FRAMES_INDEX_LOAD_FACTOR 3
#define  CAN_FRAMES_INDEX_SIZE (CAN_FRAMES_BUFFER_SIZE * CAN_FRAMES_INDEX_LOAD_FACTOR)

#define SPI_CS_PIN 9            // CAN Bus Shield
#define LED_PIN 3
#define SERIAL_BAUD_RATE 11520

#define LEDON()     digitalWrite(LED_PIN, HIGH)
#define LEDOFF()    digitalWrite(LED_PIN, LOW)
#define LEDTOGGLE() digitalWrite(LED_PIN, 1 - digitalRead(LED_PIN))

MCP_CAN CAN(SPI_CS_PIN);

CanFrame canFramesBuffer[CAN_FRAMES_BUFFER_SIZE];
CanFrameIndexEntry canFramesIndex[CAN_FRAMES_INDEX_SIZE];

int canFramesCount = 0;
int canFramesWriteIndex = 0;
int canFramesReadIndex = 0;

byte i2cDataLength = 0;
byte i2cData[20];
byte i2cDataReceived = FALSE;

byte i2cReadRequest = 0;

int blinkCount = 0;
unsigned long lastBlinkTime = millis();

#define IS_DEBUG TRUE
#define debugLog(message) if (IS_DEBUG) Serial.println(message);

byte getCheckSum(byte* data, int length)
{
    unsigned long sum = 0;
    for (int i = 0; i < length; i++) sum += data[i];

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


    if (EEPROM.read(REG_ADDR_SET) != 0x5a)
    {
        EEPROM.write(REG_ADDR_SET, 0x5a);
        EEPROM.write(REG_ADDR, DEFAULT_I2C_ADDR);
    }

    // I2C setup
    Wire.begin(EEPROM.read(REG_ADDR));
    Wire.onReceive(handleI2CWrite);
    Wire.onRequest(handleI2CRead);

    while (CAN.begin(canBaud) != CAN_OK)
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

        while (CAN.begin(i2cData[1]) != CAN_OK) delay(100);
        EEPROM.write(REG_BAUD, i2cData[1]);
        break;
    }

    case REG_SEND: {
        if (i2cDataLength != 17) break;

        byte checksum = getCheckSum(&i2cData[1], 15);
        if (checksum != i2cData[16] || i2cData[7] > 8) break;

        unsigned long frameId = i2cData[1] << 24 | i2cData[2] << 16 | i2cData[3] << 8 | i2cData[4];
        CAN.sendMsgBuf(frameId, i2cData[5], i2cData[7], &i2cData[8]);
        break;
    }

    case REG_RECV: {
        if (i2cDataLength == 1) i2cReadRequest = REG_RECV;
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

        CanFrame frameFromBuffer = canFramesBuffer[canFramesReadIndex];

        byte frameToSend[CAN_FRAME_SIZE];
        frameToSend[0] = (frameFromBuffer.canId >> 24) & 0xff;
        frameToSend[1] = (frameFromBuffer.canId >> 16) & 0xff;
        frameToSend[2] = (frameFromBuffer.canId >> 8) & 0xff;
        frameToSend[3] = (frameFromBuffer.canId >> 0) & 0xff;
        frameToSend[4] = frameFromBuffer.isExtended;
        frameToSend[5] = frameFromBuffer.isRemoteRequest;
        frameToSend[6] = frameFromBuffer.length;
        for (int i = 0; i < frameFromBuffer.length; i++) frameToSend[7 + i] = frameFromBuffer.data[i];
        frameToSend[15] = getCheckSum(frameToSend, 15);

        for (int i = 0; i < CAN_FRAME_SIZE; i++) Wire.write(frameToSend[i]);

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

void saveFrame(CanFrame* frame) {
    unsigned char indexPosition = frame->canId % CAN_FRAMES_INDEX_SIZE;
    while (canFramesIndex[indexPosition].canId != NULL && canFramesIndex[indexPosition].canId != frame->canId) {
        indexPosition = (indexPosition + 1) % CAN_FRAMES_INDEX_SIZE;
    }

    CanFrameIndexEntry indexEntry = canFramesIndex[indexPosition];

    if (indexEntry.canId == NULL && canFramesCount == CAN_FRAMES_BUFFER_SIZE) {
        debugLog("buffer full, frame dropped");
        return;
    }

    if (indexEntry.canId == NULL) {
        debugLog("Inserting new frame");

        indexEntry.canId = frame->canId;

        indexEntry.bufferPosition = 0;
        while (canFramesBuffer[indexEntry.bufferPosition].canId != NULL) indexEntry.bufferPosition++;

        canFramesCount++;
    }

    canFramesBuffer[indexEntry.bufferPosition] = *frame;
}

void receiveCanFrame()
{
    if (CAN.checkReceive() != CAN_MSGAVAIL) return;

    CanFrame frame;
    frame.timestamp = millis();
    CAN.readMsgBuf(&frame.length, frame.data);
    frame.canId = CAN.getCanId();
    frame.isExtended = CAN.isExtendedFrame();
    frame.isRemoteRequest = CAN.isRemoteRequest();

    saveFrame(&frame);
}
