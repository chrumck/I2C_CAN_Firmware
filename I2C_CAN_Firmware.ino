#include <mcp_can.h>
#include <SPI.h>
#include <SBWire.h>
#include <EEPROM.h>
#include "I2C_CAN_dfs.h"

// #define IS_DEBUG

#define CAN_FRAME_SIZE 15

#ifdef IS_DEBUG
#define CAN_FRAMES_BUFFER_SIZE 4
#else
#define CAN_FRAMES_BUFFER_SIZE 15
#endif

#define CAN_FRAMES_INDEX_LOAD_FACTOR 3
#define CAN_FRAMES_INDEX_SIZE (CAN_FRAMES_BUFFER_SIZE * CAN_FRAMES_INDEX_LOAD_FACTOR)

#ifdef IS_DEBUG
#define CAN_FRAMES_PRUNE_TIME 15000
#else
#define CAN_FRAMES_PRUNE_TIME 3000
#endif

#define NO_FRAMES_AVAILABLE_RESPONSE 0x00000000
#define RECEIVE_REJECTED_RESPONSE 0x00000001
#define RESPONSE_NOT_READY_RESPONSE 0x00000002

#define SPI_CS_PIN 9            // CAN Bus Shield
#define LED_PIN 3
#define SERIAL_BAUD_RATE 115200

#define LEDON()     digitalWrite(LED_PIN, HIGH)
#define LEDOFF()    digitalWrite(LED_PIN, LOW)
#define LEDTOGGLE() digitalWrite(LED_PIN, 1 - digitalRead(LED_PIN))

MCP_CAN CAN(SPI_CS_PIN);

CanFrame canFramesBuffer[CAN_FRAMES_BUFFER_SIZE] = { 0 };
CanFrameIndexEntry canFramesIndex[CAN_FRAMES_INDEX_SIZE] = { 0 };

u8 canFramesCount = 0;

volatile u8 i2cReceivedLength = 0;
volatile u8 i2cReceiveRejected = FALSE;
volatile u8 i2cReadRequest = NULL;
volatile CanFrame* i2cRequestedFrame = NULL;
u8 i2cData[20];

int blinkCount = 0;
u32 lastBlinkTime = millis();

u32 getMaskOrFilterValue(u8 regAddress) {
    return EEPROM.read(regAddress + 1) << 24 |
        EEPROM.read(regAddress + 2) << 16 |
        EEPROM.read(regAddress + 3) << 8 |
        EEPROM.read(regAddress + 4);
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
    Wire.onReceive(receiveFromI2C);
    Wire.onRequest(sendToI2C);

    while (CAN.begin(canBaud) != CAN_OK)
    {
        delay(100);
        LEDTOGGLE();
        Serial.println("CAN FAIL");
    }

#ifndef IS_DEBUG
    CAN.init_Mask(0, EEPROM.read(REG_MASK0), getMaskOrFilterValue(REG_MASK0));
    CAN.init_Mask(1, EEPROM.read(REG_MASK1), getMaskOrFilterValue(REG_MASK1));
    CAN.init_Filt(0, EEPROM.read(REG_FILT0), getMaskOrFilterValue(REG_FILT0));
    CAN.init_Filt(1, EEPROM.read(REG_FILT1), getMaskOrFilterValue(REG_FILT1));
    CAN.init_Filt(2, EEPROM.read(REG_FILT2), getMaskOrFilterValue(REG_FILT2));
    CAN.init_Filt(3, EEPROM.read(REG_FILT3), getMaskOrFilterValue(REG_FILT3));
    CAN.init_Filt(4, EEPROM.read(REG_FILT4), getMaskOrFilterValue(REG_FILT4));
    CAN.init_Filt(5, EEPROM.read(REG_FILT5), getMaskOrFilterValue(REG_FILT5));
#endif
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

#define processMaskOrFilterRequest(_register)\
    if (i2cReceivedLength == 1) i2cReadRequest = _register;\
    if (i2cReceivedLength != 6) break;\
    for (int i = 0; i < 5; i++) EEPROM.write(_register + i, i2cData[1 + i]);\
    u32 newMaskOrFilter = i2cData[2] << 24 | i2cData[3] << 16 | i2cData[4] << 8 | i2cData[5];\

void loop()
{
    receiveCanFrame();

    blinkLed();

    WDR();

    if (i2cReceivedLength == 0) return;

    if (blinkCount == 0) blinkCount = 2;

    switch (i2cData[0]) {

    case REG_ADDR: {
        if (i2cReceivedLength != 2) break;
        EEPROM.write(REG_ADDR, i2cData[1]);
        while (TRUE);
        break;
    }

    case REG_DNUM: {
        if (i2cReceivedLength == 1) i2cReadRequest = REG_DNUM;
        break;
    }

    case REG_BAUD: {
        if (i2cReceivedLength == 1) i2cReadRequest = REG_BAUD;
        if (i2cReceivedLength != 2) break;
        if (i2cData[1] < CAN_5KBPS || i2cData[1] > CAN_1000KBPS) break;

        while (CAN.begin(i2cData[1]) != CAN_OK) delay(100);
        EEPROM.write(REG_BAUD, i2cData[1]);
        break;
    }

    case REG_SEND: {
        if (i2cReceivedLength != 17) break;
        if (i2cData[7] > 8) break;

        u32 frameId = i2cData[1] << 24 | i2cData[2] << 16 | i2cData[3] << 8 | i2cData[4];
        CAN.sendMsgBuf(frameId, i2cData[5], i2cData[7], &i2cData[8]);
        break;
    }

    case REG_RECV: {
        if (i2cReceivedLength != 1 && i2cReceivedLength != 5) break;
        i2cReadRequest = REG_RECV;

        if (i2cReceivedLength == 5) {
            i2cRequestedFrame = getFrame(i2cData[1] << 24 | i2cData[2] << 16 | i2cData[3] << 8 | i2cData[4]);
        }
        else {
            i2cRequestedFrame = getFrame(NULL);
        }

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

    i2cReceivedLength = 0;
}

void receiveFromI2C(int howMany)
{
    if (i2cReceivedLength != 0) {
        i2cReceiveRejected = TRUE;
        return;
    }

    i2cReceiveRejected = FALSE;
    while (Wire.available() > 0) i2cData[i2cReceivedLength++] = Wire.read();
}

#define sendMaskOrFilter(_register) {\
     for (int i = 0; i < 5; i++) Wire.write(EEPROM.read(_register + i));\
     break;\
}\

void sendToI2C() {
    if (i2cReceiveRejected) {
        Wire.write((RECEIVE_REJECTED_RESPONSE >> 24) & 0xFF);
        Wire.write((RECEIVE_REJECTED_RESPONSE >> 16) & 0xFF);
        Wire.write((RECEIVE_REJECTED_RESPONSE >> 8) & 0xFF);
        Wire.write((RECEIVE_REJECTED_RESPONSE) & 0xFF);
        return;
    }

    if (i2cReceivedLength != 0) {
        Wire.write((RESPONSE_NOT_READY_RESPONSE >> 24) & 0xFF);
        Wire.write((RESPONSE_NOT_READY_RESPONSE >> 16) & 0xFF);
        Wire.write((RESPONSE_NOT_READY_RESPONSE >> 8) & 0xFF);
        Wire.write((RESPONSE_NOT_READY_RESPONSE) & 0xFF);
        return;
    }

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
        if (i2cRequestedFrame == NULL) {
            Wire.write((NO_FRAMES_AVAILABLE_RESPONSE >> 24) & 0xFF);
            Wire.write((NO_FRAMES_AVAILABLE_RESPONSE >> 16) & 0xFF);
            Wire.write((NO_FRAMES_AVAILABLE_RESPONSE >> 8) & 0xFF);
            Wire.write((NO_FRAMES_AVAILABLE_RESPONSE) & 0xFF);
            break;
        }

        Wire.write((i2cRequestedFrame->canId >> 24) & 0xff);
        Wire.write((i2cRequestedFrame->canId >> 16) & 0xff);
        Wire.write((i2cRequestedFrame->canId >> 8) & 0xff);
        Wire.write((i2cRequestedFrame->canId) & 0xff);
        Wire.write(i2cRequestedFrame->isExtended);
        Wire.write(i2cRequestedFrame->isRemoteRequest);
        Wire.write(i2cRequestedFrame->length);
        for (int i = 0; i < CAN_DATA_SIZE; i++) Wire.write(i2cRequestedFrame->data[i]);

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

    if (canFramesCount == CAN_FRAMES_BUFFER_SIZE) removeOldFrames();

    CanFrame frame = { };
    CAN.readMsgBuf(&frame.length, frame.data);
    frame.canId = CAN.getCanId();
    frame.isExtended = CAN.isExtendedFrame();
    frame.isRemoteRequest = CAN.isRemoteRequest();
    frame.timestamp = millis();

    saveFrame(&frame);
}

#define getIndexPosition(_searchedCanId)\
    u8 indexPosition = _searchedCanId % CAN_FRAMES_INDEX_SIZE;\
    while (canFramesIndex[indexPosition].canId != NULL && canFramesIndex[indexPosition].canId != _searchedCanId) {\
        indexPosition = (indexPosition + 1) % CAN_FRAMES_INDEX_SIZE;\
    }\

void removeOldFrames() {
    static u32 lastRemoveTime = millis();

    u32 currentTime = millis();
    if (currentTime - lastRemoveTime < CAN_FRAMES_PRUNE_TIME) return;
    lastRemoveTime = currentTime;

    for (u8 i = 0; i < CAN_FRAMES_BUFFER_SIZE; i++)
    {
        CanFrame* frame = &canFramesBuffer[i];
        if (frame->canId == NULL || frame->timestamp + CAN_FRAMES_PRUNE_TIME > currentTime) continue;

        getIndexPosition(frame->canId);

#ifdef IS_DEBUG
        Serial.print("removing old frame:0x");
        Serial.print(frame->canId, 16);
        Serial.print(" index:");
        Serial.print(indexPosition);
        Serial.print(" buffer:");
        Serial.println(i);
#endif

        canFramesIndex[indexPosition].canId = NULL;
        frame->canId = NULL;
        canFramesCount--;
    }
}

void saveFrame(CanFrame* frame) {
    getIndexPosition(frame->canId);
    CanFrameIndexEntry* indexEntry = &canFramesIndex[indexPosition];

    if (indexEntry->canId == NULL && canFramesCount == CAN_FRAMES_BUFFER_SIZE) {
#ifdef IS_DEBUG
        Serial.print("buffer full, dropping frame:0x");
        Serial.println(frame->canId, 16);
#endif
        return;
    }

    if (indexEntry->canId == NULL) {
        indexEntry->canId = frame->canId;

        indexEntry->bufferPosition = 0;
        while (canFramesBuffer[indexEntry->bufferPosition].canId != NULL) indexEntry->bufferPosition++;

        canFramesBuffer[indexEntry->bufferPosition] = *frame;
        canFramesCount++;

#ifdef IS_DEBUG
        Serial.print("added new frame:0x");
        Serial.print(frame->canId, 16);
        Serial.print(", index pos:");
        Serial.print(indexPosition);
        Serial.print(", buffer pos:");
        Serial.println(indexEntry->bufferPosition);
#endif

        return;
    }

    CanFrame* previousFrame = &canFramesBuffer[indexEntry->bufferPosition];
    if (previousFrame->isSent == FALSE) frame->timestamp = previousFrame->timestamp;

    canFramesBuffer[indexEntry->bufferPosition] = *frame;

#ifdef IS_DEBUG
    Serial.print("updated frame:0x");
    Serial.print(frame->canId, 16);
    Serial.print(", timestamp:");
    Serial.print(frame->timestamp);
    Serial.print(", isSent:");
    Serial.print(frame->isSent);
    Serial.print(", index pos:");
    Serial.print(indexPosition);
    Serial.print(", buffer pos:");
    Serial.println(indexEntry->bufferPosition);
#endif
}

CanFrame* getFrame(u32 frameId) {
    if (canFramesCount == 0) {
#ifdef IS_DEBUG
        Serial.println("frames count 0, nothing to send");
#endif
        return NULL;
    }


    if (frameId != NULL) {
        getIndexPosition(frameId);
        if (canFramesIndex[indexPosition].canId == NULL) {

#ifdef IS_DEBUG
            Serial.print("frame not available:0x");
            Serial.println(frameId, 16);
#endif

            return NULL;
        }

        CanFrame* frame = &canFramesBuffer[canFramesIndex[indexPosition].bufferPosition];

#ifdef IS_DEBUG
        Serial.print(frame->isSent ? "frame already sent:0x" : "sending frame:0x");
        Serial.println(frame->canId, 16);
#endif

        if (frame->isSent) return NULL;

        frame->isSent = TRUE;
        return;
    }

    CanFrame* oldestFrame = NULL;
    for (u8 i = 0; i < CAN_FRAMES_BUFFER_SIZE; i++)
    {
        u32 oldestTimestamp = oldestFrame == NULL ? ULONG_MAX : oldestFrame->timestamp;
        if (canFramesBuffer[i].canId != NULL &&
            canFramesBuffer[i].isSent == FALSE &&
            canFramesBuffer[i].timestamp < oldestTimestamp) {
            oldestFrame = &canFramesBuffer[i];
        }
    }

#ifdef IS_DEBUG
    if (oldestFrame != NULL) {
        Serial.print("sending oldest frame:0x");
        Serial.println(oldestFrame->canId, 16);
    }
    else {
        Serial.println("all frames already sent");
    }
#endif

    if (oldestFrame != NULL) oldestFrame->isSent = TRUE;
    return oldestFrame;
}