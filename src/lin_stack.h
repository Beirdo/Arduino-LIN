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
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>

/*
        Please, read Getting Started Guide first.
*/

enum {
    STATE_NORMAL,
    STATE_STANDBY,
    STATE_SLEEP,
};

class lin_stack {
public:
    // Constructors
    lin_stack(HardwareSerial &_channel = Serial, uint16_t _baud = 19200, 
              int8_t _wakeup_pin = -1, int8_t _sleep_pin = -1,
              uint8_t _ident = 0); // Constructor for Master and Slave Node

    // Methods

    // Writing data to bus
    void write(const uint8_t ident, const void *data, size_t len); // write whole package
    void writeRequest(const uint8_t ident);                        // Write header only
    void writeResponse(const void *data, size_t len);              // Write response only
    void writeStream(const void *data, size_t len);                // Writing user data to LIN bus
    bool read(uint8_t *data, const size_t len,
              size_t *read);                      // read data from LIN bus, checksum and ident validation
    void busWakeUp();                             // send wakeup frame for waking up all bus participants
    void sleep(int8_t sleep_state); // method for controlling transceiver modes (0 - sleep, 1 - standby, 2 - normal)

    void setupSerial();  // set up Serial communication for receiving data.
    bool waitBreak(uint32_t maxTimeout);
    int readStream(uint8_t *data, size_t len); // read data from LIN bus

    uint8_t generateIdent(const uint8_t addr) const;
    uint8_t calcIdentParity(const uint8_t ident) const;

    static constexpr uint32_t MAX_DELAY = UINT32_MAX;

    // Private methods and variables
private:
    const uint16_t baud;     // 10417 is best for LIN Interface, most device should work
    HardwareSerial &channel; // which channel should be used
    uint8_t ident;           // user defined Identification Byte
    int8_t wake_pin;
    int8_t sleep_pin;
    int8_t current_sleep_state;

    void sleep_config(); // configuration of sleep pins
    void lin_break();    // for generating Synch Break
    bool validateParity(uint8_t ident); // for validating Identification Byte, can be modified for validating parity
    bool validateChecksum(const void *data, size_t len); // for validating Checksum Byte
    uint8_t calcChecksum(const void *data, size_t len);
};
