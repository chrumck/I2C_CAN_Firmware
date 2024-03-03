# Firmware for Longan Labs [I2C CAN Bus Module](https://www.longan-labs.cc/1030017.html) - [Knurdash](https://github.com/chrumck/KnurDash) Fork

## Description

This firmware was forked from [SjoerdQ's fork of original firmware from Longan-Labs](https://github.com/SjoerdQ/I2C_CAN_Firmware) and heavily modified to serve frames from a dictionary instead of a FIFO type buffer.

The dictionary is a custom implementation of a [hash table with linear probing](https://en.wikipedia.org/wiki/Linear_probing).

The dictionary can hold up to `CAN_FRAMES_BUFFER_SIZE` frames with unique frame ids. If a new frame arrives with the id already stored in the dictionary, the old frame is replaced with the new one. If the dictionary is full and a frame arrives with an id not in the dictionary, the frame is dropped.

Frames which are not retrieved through I2C for a while are periodically removed from the dictionary, giving a chance to frames ids which were not in the dictionary to take their place.

The firmware sets the CAN controller to listen-only mode. Sending frames is not allowed.

The firmware has been developed as part of the [KnurDash](https://github.com/chrumck/KnurDash) project.

---

## Installation of Arduino Libraries

This project requires two libraries to be installed prior to flashing the controller:

- **[autowp CAN BUS MCP2515 Library](https://github.com/autowp/arduino-mcp2515)** - Driver for the MPC2515 CAN Bus transceiver.
- **[SBWire](https://github.com/freespace/SBWire)** - Replacement for the default Arduino Wire (i<sup>2</sup>c) library with basic timeouts to fix the board from occasionally not correctly responding to messages.

---

## Rejected I<sup>2</sup>C Requests

In order to give the module enough time to process I2C requests, there should be some delay introduced between I2C commands.

If consecutive write commands are sent without sufficient delay, the module will respond to the rejected write command with four bytes representing the `RECEIVE_REJECTED_RESPONSE` message.

If a read command is sent too early after a write command, the controller will respond with four bytes representing the `RESPONSE_NOT_READY_RESPONSE` message. The response can be retrieved if another read request is sent after sufficient delay.

---

## I<sup>2</sup>C Registers

The device default I<sup>2</sup>C slave address is 0x25 but can be changed by writing to address 1. Note that the new address is stored inside the internal EEPROM of the chip, and therefore changes in the bus address will remain after power loss.

The following table contains the various registers that are available on the i2c interface of the CAN Module.

| Address | Name  | Length   | Type | Default Value | Description                    |
| ------- | ----- | -------- | ---- | ------------- | ------------------------------ |
| 0x01    | Addr  | 1 Byte   | W    | 0x25          | I2C slave Address              |
| 0x02    | DNUM  | 1 Byte   | R    | 0             | No. of CAN frames in RX buffer |
| 0x03    | BAUD  | 1 Byte   | W/R  | 16(500kb/s)   | Set the CAN baudrate           |
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

## 0x40 - Read CAN Frame

If data is requested without any additional bytes passed, the code defaults back to FIFO behavior and serves the oldest available frame. Conversely, if four big-endian bytes representing the 32-bit frame id are passed to the register, the latest frame with that id is returned if the frame is available in the dictionary.

### No Frames Available

If the oldest frame, or the frame for the given id, is not available, the module will respond with four bytes representing the `NO_FRAMES_AVAILABLE_RESPONSE` message.
