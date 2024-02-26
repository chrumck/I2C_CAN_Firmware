#include <avr/wdt.h>
#include <SPI.h>
#include <mcp2515.h>
#include <SBWire.h>
#include <EEPROM.h>
#include "I2C_CAN_dfs.h"

// #define IS_DEBUG

#ifdef IS_DEBUG
#define CAN_FRAMES_BUFFER_SIZE 4
#define CAN_FRAMES_INDEX_SIZE 16    // has to be power of two for hashing to work
#define CAN_FRAMES_PRUNE_TIME 15000
#else
#define CAN_FRAMES_BUFFER_SIZE 32
#define CAN_FRAMES_INDEX_SIZE 64    // has to be power of two for hashing to work
#define CAN_FRAMES_PRUNE_TIME 5000
#endif

#define CS_PIN 10
#define SERIAL_BAUD_RATE 115200

CanFrame canFramesBuffer[CAN_FRAMES_BUFFER_SIZE] = { 0 };
CanFrameIndexEntry canFramesIndex[CAN_FRAMES_INDEX_SIZE] = { 0 };
u8 canFramesHashBase = CAN_FRAMES_INDEX_SIZE - 1;

u8 canFramesCount = 0;

volatile u8 i2cReceivedLength = 0;
volatile u8 i2cReceiveRejected = FALSE;
volatile u8 i2cReadRequest = NULL;
u8 i2cFrameToSend[CAN_FRAME_SIZE];
u8 i2cData[I2C_DATA_LENGTH];

u8 getCheckSum(u8* data, int length)
{
    u32 sum = 0;
    for (int i = 0; i < length; i++) sum += data[i];
    if (sum > 0xff) sum = (~sum) + 1;
    sum = sum & 0xff;
    return sum;
}

u32 getMaskOrFilterValue(u8 regAddress) {
    return EEPROM.read(regAddress + 1) << 24 |
        EEPROM.read(regAddress + 2) << 16 |
        EEPROM.read(regAddress + 3) << 8 |
        EEPROM.read(regAddress + 4);
}

MCP2515 mcp2515(CS_PIN);

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    if (EEPROM.read(REG_I2C_ADDRESS_SET) != REG_I2C_ADDRESS_SET_VALUE)
    {
        EEPROM.write(REG_I2C_ADDRESS_SET, REG_I2C_ADDRESS_SET_VALUE);
        EEPROM.write(REG_I2C_ADDRESS, DEFAULT_I2C_ADDRESS);
    }

    // I2C setup
    Wire.begin(EEPROM.read(REG_I2C_ADDRESS));
    Wire.onReceive(receiveFromI2C);
    Wire.onRequest(sendToI2C);

    int eepromCanBaud = EEPROM.read(REG_CAN_BAUD_RATE);
    CAN_SPEED canBaud = (eepromCanBaud >= CAN_5KBPS && eepromCanBaud <= CAN_1000KBPS) ? (enum CAN_SPEED)eepromCanBaud : CAN_500KBPS;

    mcp2515.reset();
    mcp2515.setBitrate(canBaud, MCP_16MHZ);
    mcp2515.setNormalMode();

#ifndef IS_DEBUG
    mcp2515.setFilterMask(MCP2515::MASK0, EEPROM.read(REG_MASK0), getMaskOrFilterValue(REG_MASK0));
    mcp2515.setFilterMask(MCP2515::MASK1, EEPROM.read(REG_MASK1), getMaskOrFilterValue(REG_MASK1));
    mcp2515.setFilter(MCP2515::RXF0, EEPROM.read(REG_FILT0), getMaskOrFilterValue(REG_FILT0));
    mcp2515.setFilter(MCP2515::RXF1, EEPROM.read(REG_FILT1), getMaskOrFilterValue(REG_FILT1));
    mcp2515.setFilter(MCP2515::RXF2, EEPROM.read(REG_FILT2), getMaskOrFilterValue(REG_FILT2));
    mcp2515.setFilter(MCP2515::RXF3, EEPROM.read(REG_FILT3), getMaskOrFilterValue(REG_FILT3));
    mcp2515.setFilter(MCP2515::RXF4, EEPROM.read(REG_FILT4), getMaskOrFilterValue(REG_FILT4));
    mcp2515.setFilter(MCP2515::RXF5, EEPROM.read(REG_FILT5), getMaskOrFilterValue(REG_FILT5));

    wdt_enable(WDTO_1S);
#endif

}

#define processMaskOrFilterRequest(_register)\
    if (i2cReceivedLength == 1) i2cReadRequest = _register;\
    if (i2cReceivedLength != 7) break;\
    if (getCheckSum(i2cData + 1, 5) != i2cData[6]) break;\
    for (int i = 0; i < 5; i++) EEPROM.write(_register + i, i2cData[1 + i]);\
    u32 newMaskOrFilter = i2cData[2] << 24 | i2cData[3] << 16 | i2cData[4] << 8 | i2cData[5];\

void loop()
{
    wdt_reset();

    receiveCanFrame();

    if (i2cReceivedLength < 1 || i2cReceivedLength > I2C_DATA_LENGTH) return;

    switch (i2cData[0]) {

    case REG_I2C_ADDRESS: {
        if (i2cReceivedLength != 2) break;
        EEPROM.write(REG_I2C_ADDRESS, i2cData[1]);
        while (TRUE);
        break;
    }

    case REG_FRAMES_COUNT: {
        if (i2cReceivedLength == 1) i2cReadRequest = REG_FRAMES_COUNT;
        break;
    }

    case REG_CAN_BAUD_RATE: {
        if (i2cReceivedLength == 1) i2cReadRequest = REG_CAN_BAUD_RATE;
        if (i2cReceivedLength != 2) break;
        if (i2cData[1] < CAN_5KBPS || i2cData[1] > CAN_1000KBPS) break;

        CAN_SPEED canBaud = (enum CAN_SPEED)i2cData[1];
        mcp2515.reset();
        mcp2515.setBitrate(canBaud, MCP_16MHZ);
        mcp2515.setListenOnlyMode();

        EEPROM.write(REG_CAN_BAUD_RATE, i2cData[1]);
        break;
    }

    case REG_SEND_FRAME: {
        if (i2cReceivedLength != I2C_DATA_LENGTH) break;
        if (i2cData[CAN_FRAME_BIT_DATA_LENGTH + 1] > CAN_DATA_SIZE) break;
        if (getCheckSum(i2cData + 1, CAN_FRAME_SIZE - 1) != i2cData[CAN_FRAME_BIT_CHECKSUM + 1]) break;

        u32 frameId =
            i2cData[CAN_FRAME_BIT_ID_0 + 1] << 24 |
            i2cData[CAN_FRAME_BIT_ID_1 + 1] << 16 |
            i2cData[CAN_FRAME_BIT_ID_2 + 1] << 8 |
            i2cData[CAN_FRAME_BIT_ID_3 + 1];

        struct can_frame frame = { .can_id = frameId, .can_dlc = i2cData[CAN_FRAME_BIT_DATA_LENGTH + 1] };
        memcpy(frame.data, &i2cData[CAN_FRAME_BIT_DATA_0 + 1], CAN_DATA_SIZE);
        mcp2515.sendMessage(&frame);
        break;
    }

    case REG_RECEIVE_FRAME: {
        if (i2cReceivedLength != 1 && i2cReceivedLength != 5) break;

        i2cReadRequest = REG_RECEIVE_FRAME;
        memset(&i2cFrameToSend, 0, CAN_FRAME_SIZE);

        u32 frameId = i2cReceivedLength != 5 ? NULL :
            i2cData[CAN_FRAME_BIT_ID_0 + 1] << 24 |
            i2cData[CAN_FRAME_BIT_ID_1 + 1] << 16 |
            i2cData[CAN_FRAME_BIT_ID_2 + 1] << 8 |
            i2cData[CAN_FRAME_BIT_ID_3 + 1];

        CanFrame* frame = getFrame(frameId);

        if (frame == NULL) break;

        i2cFrameToSend[CAN_FRAME_BIT_ID_0] = (frame->canId >> 24) & 0xFF;
        i2cFrameToSend[CAN_FRAME_BIT_ID_1] = (frame->canId >> 16) & 0xFF;
        i2cFrameToSend[CAN_FRAME_BIT_ID_2] = (frame->canId >> 8) & 0xFF;
        i2cFrameToSend[CAN_FRAME_BIT_ID_3] = frame->canId & 0xFF;
        i2cFrameToSend[CAN_FRAME_BIT_DATA_LENGTH] = frame->dataLength;
        memcpy(&i2cFrameToSend[CAN_FRAME_BIT_DATA_0], frame->data, frame->dataLength);
        i2cFrameToSend[CAN_FRAME_BIT_CHECKSUM] = getCheckSum(i2cFrameToSend, CAN_FRAME_SIZE - 1);

        break;
    }

    case REG_MASK0: {
        processMaskOrFilterRequest(REG_MASK0);
        mcp2515.setFilterMask(MCP2515::MASK0, i2cData[1], newMaskOrFilter);
        break;
    }

    case REG_MASK1: {
        processMaskOrFilterRequest(REG_MASK1);
        mcp2515.setFilterMask(MCP2515::MASK1, i2cData[1], newMaskOrFilter);
        break;
    }

    case REG_FILT0: {
        processMaskOrFilterRequest(REG_FILT0);
        mcp2515.setFilter(MCP2515::RXF0, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT1: {
        processMaskOrFilterRequest(REG_FILT1);
        mcp2515.setFilter(MCP2515::RXF1, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT2: {
        processMaskOrFilterRequest(REG_FILT2);
        mcp2515.setFilter(MCP2515::RXF2, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT3: {
        processMaskOrFilterRequest(REG_FILT3);
        mcp2515.setFilter(MCP2515::RXF3, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT4: {
        processMaskOrFilterRequest(REG_FILT4);
        mcp2515.setFilter(MCP2515::RXF4, i2cData[1], newMaskOrFilter);
        break;
    }
    case REG_FILT5: {
        processMaskOrFilterRequest(REG_FILT5);
        mcp2515.setFilter(MCP2515::RXF5, i2cData[1], newMaskOrFilter);
        break;
    }

    default:
        break;
    }

    i2cReceivedLength = 0;
}

void receiveFromI2C(int howMany)
{
    if (i2cReceivedLength < 0 || i2cReceivedLength > I2C_DATA_LENGTH) i2cReceivedLength = 0;

    i2cReceiveRejected = i2cReceivedLength != 0 ? TRUE : FALSE;

    while (Wire.available() > 0) {
        if (!i2cReceiveRejected && i2cReceivedLength < I2C_DATA_LENGTH) i2cData[i2cReceivedLength++] = Wire.read();
        else Wire.read();
    }
}

#define sendMaskOrFilter(_register) {\
    u8 maskOrFilterToSend[6];\
    for (int i = 0; i < 5; i++) maskOrFilterToSend[i] = EEPROM.read(_register + i);\
    maskOrFilterToSend[5] = getCheckSum(maskOrFilterToSend, 5);\
    Wire.write(&maskOrFilterToSend[0], 6);\
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

    case REG_CAN_BAUD_RATE: {
        Wire.write(EEPROM.read(REG_CAN_BAUD_RATE));
        break;
    }

    case REG_FRAMES_COUNT: {
        Wire.write(canFramesCount);
        break;
    }

    case REG_RECEIVE_FRAME: {
        Wire.write(&i2cFrameToSend[0], CAN_FRAME_SIZE);
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
    struct can_frame received = { };
    MCP2515::ERROR readResult = mcp2515.readMessage(&received);

    if (readResult == MCP2515::ERROR_NOMSG) return;

    if (readResult != MCP2515::ERROR_OK) {
#ifdef IS_DEBUG
        Serial.print("Receiving CAN frame failed with error:");
        Serial.println(readResult);
#endif
        return;
    }

    if (canFramesCount == CAN_FRAMES_BUFFER_SIZE) removeOldFrames();

    CanFrame frame = { };
    frame.canId = received.can_id;
    frame.dataLength = received.can_dlc;
    memcpy(frame.data, received.data, CAN_DATA_SIZE);
    frame.timestamp = millis();

    saveFrame(&frame);
}

#define getIndexPosition(_searchedCanId)\
    u8 indexPosition = _searchedCanId & canFramesHashBase;\
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
        return frame;
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

