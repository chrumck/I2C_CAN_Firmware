#ifndef __I2C_CAN_DFS_H__
#define __I2C_CAN_DFS_H__

#include <mcp2515.h>

#define FALSE 0
#define TRUE 1
#define ULONG_MAX 0xffffffff

#define CAN_DATA_SIZE CAN_MAX_DLEN
#define CAN_FRAME_SIZE 14

#define CAN_FRAME_BIT_ID_0 0
#define CAN_FRAME_BIT_ID_1 1
#define CAN_FRAME_BIT_ID_2 2
#define CAN_FRAME_BIT_ID_3 3
#define CAN_FRAME_BIT_DATA_LENGTH 4
#define CAN_FRAME_BIT_DATA_0 5
#define CAN_FRAME_BIT_DATA_1 6
#define CAN_FRAME_BIT_DATA_2 7
#define CAN_FRAME_BIT_DATA_3 8
#define CAN_FRAME_BIT_DATA_4 9
#define CAN_FRAME_BIT_DATA_5 10
#define CAN_FRAME_BIT_DATA_6 11
#define CAN_FRAME_BIT_DATA_7 12
#define CAN_FRAME_BIT_CHECKSUM 13

#define I2C_DATA_LENGTH (CAN_FRAME_SIZE + 1)
#define RECEIVE_REJECTED_RESPONSE 0x01010101
#define RESPONSE_NOT_READY_RESPONSE 0x01010102

typedef struct {
  u32 canId;
  byte dataLength;
  byte data[CAN_DATA_SIZE];
  u32 timestamp;
  byte isSent;
} CanFrame;

typedef struct {
  unsigned long canId;
  byte bufferPosition;
} CanFrameIndexEntry;

#define DEFAULT_I2C_ADDRESS    0X25

#define REG_I2C_ADDRESS   0X01
#define REG_FRAMES_COUNT  0x02
#define REG_CAN_BAUD_RATE 0X03
#define REG_SEND_FRAME    0X30
#define REG_RECEIVE_FRAME 0X40
#define REG_MASK0         0X60
#define REG_MASK1         0X65
#define REG_FILT0         0X70
#define REG_FILT1         0X80
#define REG_FILT2         0X90
#define REG_FILT3         0XA0
#define REG_FILT4         0XB0
#define REG_FILT5         0XC0

#define REG_I2C_ADDRESS_SET          0X51
#define REG_I2C_ADDRESS_SET_VALUE    0x5A

#endif
