# Firmware for Longan Labs [I2C CAN Bus Module](https://www.longan-labs.cc/1030017.html) - Knurdash Fork

## Description

This firmware was forked from [SjoerdQ's fork of original firmware from Longan-Labs](https://github.com/SjoerdQ/I2C_CAN_Firmware) and heavily modified to serve frames from a dictionary instead of a FIFO type buffer.

The dictionary is a custom implementation of a [hash table with linear probing](https://en.wikipedia.org/wiki/Linear_probing).

The dictionary can hold up to 15 frames with unique frame ids. If a new frame arrives with the id already stored in the dictionary, the old frame is replaced with the new one. If the dictionary is full and a frame arrives with an id not in the dictionary, the frame is dropped.

Frames which are not retrieved through I2C for a while are periodically removed from the dictionary, giving a chance to frames ids which were not in the dictionary to take their place.

---

## Installation of Arduino libraries

This project requires two libraries to be installed prior to flashing the controller:

- **[Longan Labs CAN BUS MCP2515 Library](https://github.com/Longan-Labs/Aruino_CAN_BUS_MCP2515)** - Driver for the MPC2515 CAN Bus transceiver.
- **[SBWire](https://github.com/freespace/SBWire)** - Replacement for the default Arduino Wire (i<sup>2</sup>c) library with basic timeouts to fix the board from occasionally not correctly responding to messages.

---

## I<sup>2</sup>C Registers

The device default I<sup>2</sup>C slave address is 0x25 but can be changed by writing to address 1. Note that the new address is stored inside the internal EEPROM of the chip, and therefore changes in the bus address will remain after power loss.

The following table contains the various registers that are available on the i2c interface of the CAN Module.

| Address | Name  | Length   | Type | Default Value | Description                    |
| ------- | ----- | -------- | ---- | ------------- | ------------------------------ |
| 0x01    | Addr  | 1 Byte   | W    | 0x25          | I2C slave Address              |
| 0x02    | DNUM  | 1 Byte   | R    | 0             | No. of CAN frames in RX buffer |
| 0x03    | BAUD  | 1 Byte   | W/R  | 16(500kb/s)   | Set the CAN baudrate           |
| 0x30    | Send  | 16 Bytes | W    | -             | Send CAN frame                 |
| 0x40    | Recv  | 16 Bytes | R    | -             | Read CAN frame                 |
| 0x60    | Mask0 | 5 Bytes  | W/R  | 0             | Mask 0                         |
| 0x65    | Mask1 | 5 Bytes  | W/R  | 0             | Mask 1                         |
| 0x70    | Filt0 | 5 Bytes  | W/R  | 0             | Filter 0                       |
| 0x80    | Filt1 | 5 Bytes  | W/R  | 0             | Filter 1                       |
| 0x90    | Filt2 | 5 Bytes  | W/R  | 0             | Filter 2                       |
| 0xA0    | Filt3 | 5 Bytes  | W/R  | 0             | Filter 3                       |
| 0xB0    | Filt4 | 5 Bytes  | W/R  | 0             | Filter 4                       |
| 0xC0    | Filt5 | 5 Bytes  | W/R  | 0             | Filter 5                       |

---

## 0x40 - Read CAN frame

If data is requested without any additional bytes passed, the code defaults back to FIFO behavior and serves the oldest available frame. Conversely, if four big-endian bytes representing the 32-bit frame id are passed to the register, the latest frame with that id is returned if the frame is available in the dictionary.

---

## 0x03 - CAN baudrates

| Value  | CAN Baudrate (kb/s) |
| ------ | ------------------- |
| 1      | 5                   |
| 2      | 10                  |
| 3      | 20                  |
| 4      | 25                  |
| 5      | 31.2                |
| 6      | 33                  |
| 7      | 40                  |
| 8      | 50                  |
| 9      | 80                  |
| 10     | 83.3                |
| 11     | 95                  |
| 12     | 100                 |
| 13     | 125                 |
| 14     | 200                 |
| 15     | 250                 |
| **16** | **500** (default)   |
| 17     | 666                 |
| 18     | 1000                |
