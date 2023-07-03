#include <mcp_can.h>
#include <SPI.h>
#include <SBWire.h>
#include <EEPROM.h>
#include "I2C_CAN_dfs.h"

#define IS_DEBUG

#define CAN_FRAME_SIZE 16

#ifdef IS_DEBUG
#define CAN_FRAMES_BUFFER_SIZE 2
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

#define SPI_CS_PIN 9            // CAN Bus Shield
#define LED_PIN 3
#define SERIAL_BAUD_RATE 115200

#define LEDON()     digitalWrite(LED_PIN, HIGH)
#define LEDOFF()    digitalWrite(LED_PIN, LOW)
#define LEDTOGGLE() digitalWrite(LED_PIN, 1 - digitalRead(LED_PIN))

MCP_CAN CAN(SPI_CS_PIN);

CanFrame canFramesBuffer[CAN_FRAMES_BUFFER_SIZE];
CanFrameIndexEntry canFramesIndex[CAN_FRAMES_INDEX_SIZE];

int canFramesCount = 0;

u8 i2cDataLength = 0;
u8 i2cData[20];
u8 i2cDataReceived = FALSE;

u8 i2cReadRequest = 0;
u32 readFrameId = NULL;

int blinkCount = 0;
u32 lastBlinkTime = millis();

u8 getCheckSum(u8* data, int length)
{
    u32 sum = 0;
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
    u32 newMaskOrFilter = i2cData[5] << 24 | i2cData[4] << 16 | i2cData[3] << 8 | i2cData[2];\

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

        u8 checksum = getCheckSum(&i2cData[1], 15);
        if (checksum != i2cData[16] || i2cData[7] > 8) break;

        u32 frameId = i2cData[1] << 24 | i2cData[2] << 16 | i2cData[3] << 8 | i2cData[4];
        CAN.sendMsgBuf(frameId, i2cData[5], i2cData[7], &i2cData[8]);
        break;
    }

    case REG_RECV: {
        if (i2cDataLength != 1 && i2cDataLength != 5) break;

        i2cReadRequest = REG_RECV;

        if (i2cDataLength == 5) readFrameId = i2cData[1] << 24 | i2cData[2] << 16 | i2cData[3] << 8 | i2cData[4];
        else readFrameId = NULL;
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
        CanFrame* frameFromBuffer = getFrame();
        if (frameFromBuffer == NULL) break;

        u8 frameToSend[CAN_FRAME_SIZE] = { };
        frameToSend[0] = (frameFromBuffer->canId >> 24) & 0xff;
        frameToSend[1] = (frameFromBuffer->canId >> 16) & 0xff;
        frameToSend[2] = (frameFromBuffer->canId >> 8) & 0xff;
        frameToSend[3] = (frameFromBuffer->canId >> 0) & 0xff;
        frameToSend[4] = frameFromBuffer->isExtended;
        frameToSend[5] = frameFromBuffer->isRemoteRequest;
        frameToSend[6] = frameFromBuffer->length;
        for (int i = 0; i < frameFromBuffer->length; i++) frameToSend[7 + i] = frameFromBuffer->data[i];
        frameToSend[15] = getCheckSum(frameToSend, 15);

        for (int i = 0; i < CAN_FRAME_SIZE; i++) Wire.write(frameToSend[i]);

        frameFromBuffer->isSent = TRUE;

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
    removeOldFrames();

    if (CAN.checkReceive() != CAN_MSGAVAIL) return;

    CanFrame frame = { };
    CAN.readMsgBuf(&frame.length, frame.data);
    frame.canId = CAN.getCanId();
    frame.isExtended = CAN.isExtendedFrame();
    frame.isRemoteRequest = CAN.isRemoteRequest();
    frame.timestamp = millis();

    saveFrame(&frame);
}

#define getIndexPosition(searchedCanId)\
    u8 indexPosition = searchedCanId % CAN_FRAMES_INDEX_SIZE;\
    while (canFramesIndex[indexPosition].canId != NULL && canFramesIndex[indexPosition].canId != searchedCanId) {\
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
#ifdef IS_DEBUG
    Serial.print("saving frame:0x");
    Serial.print(frame->canId, 16);
    Serial.print(" length:");
    Serial.println(frame->length);
#endif

    getIndexPosition(frame->canId);
    CanFrameIndexEntry* indexEntry = &canFramesIndex[indexPosition];

#ifdef IS_DEBUG
    Serial.print("trying to save under index:");
    Serial.println(indexPosition);
#endif

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
        Serial.print(" buffer:");
        Serial.println(indexEntry->bufferPosition);
#endif
        return;
    }

    CanFrame* previousFrame = &canFramesBuffer[indexEntry->bufferPosition];
    if (previousFrame->isSent == FALSE) {
        frame->timestamp = previousFrame->timestamp;
#ifdef IS_DEBUG
        Serial.print("overwriting unsent frame:0x");
        Serial.println(frame->canId, 16);
#endif
    }

    canFramesBuffer[indexEntry->bufferPosition] = *frame;

#ifdef IS_DEBUG
    Serial.print("updated frame:0x");
    Serial.print(frame->canId, 16);
    Serial.print(" timestamp:");
    Serial.print(frame->timestamp);
    Serial.print(" buffer:");
    Serial.println(indexEntry->bufferPosition);
#endif
}

CanFrame* getFrame() {
    if (canFramesCount == 0) {
#ifdef IS_DEBUG
        Serial.println("frames count 0, nothing to send");
#endif
        return NULL;
    }

    if (readFrameId != NULL) {
        getIndexPosition(readFrameId);
        if (canFramesIndex[indexPosition].canId == NULL) {
#ifdef IS_DEBUG
            Serial.print("frame not available:0x");
            Serial.println(readFrameId, 16);
#endif
            return NULL;
        }

        CanFrame* frame = &canFramesBuffer[canFramesIndex[indexPosition].bufferPosition];

#ifdef IS_DEBUG
        Serial.print(frame->isSent ? "frame already sent:0x" : "sending frame:0x");
        Serial.println(frame->canId, 16);
#endif

        return frame->isSent ? NULL : frame;
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

    return oldestFrame;
}