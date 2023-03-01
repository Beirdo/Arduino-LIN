/*  Copyright (c) 2016 Macchina
 *
 *  Permission is hereby granted, free of charge, to any person obtaining
 *  a copy of this software and associated documentation files (the
 *  "Software"), to deal in the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  LIN STACK for TJA1021
 *  v2.0
 *
 *  Short description:
 *  Comunication stack for LIN and TJA1021 LIN transceiver.
 *  Can be modified for any Arduino board with UART available and any LIN slave.
 *
 *  Author: Blaž Pongrac B.S., RoboSap, Institute of Technology, Ptuj (www.robosap-institut.eu)
 *  Author: Laszlo Hegedüs
 *
 *  Arduino IDE 1.6.9
 *  RoboSap, Institute of Technology, September 2016
 *
 * Changes in 3.0 are Copyright (c) 2023 Gavin Hurlbut
 *  - reworked for better slave use
 *  - reworked sleep/wake modes
 *  - refactored/renamed class
 *  - added support for AVR (particularly in break detection)
 *  - made available via the Arduino Library Manager
 */

#ifndef __LINBus_stack_h_
#define __LINBus_stack_h_

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>

typedef enum {
    STATE_NORMAL,
    STATE_STANDBY,
    STATE_SLEEP,
} linbus_state_t;

#ifndef OWERRIDE_GPIO_FUNCS
#define pinModeFunc pinMode
#define digitalWriteFunc digitalWrite
#endif

class LINBus_stack {
public:
    // Constructors
    LINBus_stack(HardwareSerial &_channel = Serial, uint16_t _baud = 19200);

#ifdef OVERRIDE_GPIO_FUNCS
    void setPinMode(void (*pinModeFunc_)(uint8_t, uint8_t))
    {
      pinModeFunc = pinModeFunc_;
    };

    void setDigitalWrite(void (*digitalWriteFunc_)(uint8_t, uint8_t))
    {
      digitalWriteFunc = digitalWriteFunc_;
    };
#endif

    void begin(int8_t _wakeup_pin = -1, int8_t _sleep_pin = -1,
               uint8_t _ident = 0); // Constructor for Master and Slave Node

    // Methods

    // write whole package
    void write(const uint8_t ident, const void *data, size_t len);

    // Write header only (used to request data from a slave)
    void writeRequest(const uint8_t ident);

    // Write response only (used by a slave to respond to a request)
    void writeResponse(const void *data, size_t len);

    // Writing user data to LIN bus as is.
    void writeStream(const void *data, size_t len);

    // read data from LIN bus, checksum and ident validation (no checksum check if only header received)
    bool read(uint8_t *data, const size_t len, size_t *read);

    // send wakeup frame for waking up all bus participants
    void busWakeUp(void);

    // method for controlling transceiver modes (0 - sleep, 1 - standby, 2 - normal)
    void sleep(linbus_state_t sleep_state);

    // set up Serial communication for receiving data.
    void setupSerial(void);

    bool waitBreak(uint32_t maxTimeout);

    // read data from LIN bus as is.
    int readStream(uint8_t *data, size_t len);

    uint8_t generateIdent(const uint8_t addr) const;

    uint8_t calcIdentParity(const uint8_t ident) const;

    // for validating Checksum Byte
    bool validateChecksum(const void *data, size_t len);

    static constexpr uint32_t MAX_DELAY = UINT32_MAX;

private:
    // 10417 is best for LIN Interface, most device should work
    const uint16_t baud;

    // which channel should be used
    HardwareSerial &channel;

    // user defined Identification Byte
    uint8_t ident;

    int8_t wake_pin;
    int8_t sleep_pin;
    linbus_state_t current_sleep_state;

#ifdef OVERRIDE_GPIO_FUNCS
    void (*pinModeFunc)(uint8_t, uint8_t);
    void (*digitalWriteFunc)(uint8_t, uint8_t);
#endif

    // configuration of sleep pins
    void sleep_config(void);

    // is break detected?
    bool breakDetected(void);

    // for generating Synch Break
    void lin_break(void);

    // for validating Identification Byte, can be modified for validating parity
    bool validateParity(uint8_t ident);

    uint8_t calcChecksum(const void *data, size_t len);
};

#endif
