/*
 * AT28C256 EEPROM Reader and Programmer
 * 
 * This code implements the serial wire protocol as described in protocol.txt
 * 
 * Pin Layout
 * 
 * Pin | Circuit
 * ----+--------------
 *  49 | EEPROM IO0 (PL0)
 *  48 | EEPROM IO1
 *  47 | EEPROM IO2
 *  46 | EEPROM IO3
 *  45 | EEPROM IO4
 *  44 | EEPROM IO5
 *  43 | EEPROM IO6
 *  42 | EEPROM IO7 (PL7)
 * ----+--------------
 *  22 | A0 (PA0)
 *  23 | A1
 *  24 | A2
 *  25 | A3
 *  26 | A4
 *  27 | A5
 *  28 | A6
 *  29 | A7 (PA7)
 *  37 | A8 (PC0)
 *  36 | A9
 *  35 | A10
 *  34 | A11
 *  33 | A12
 *  32 | A13
 *  31 | A14
 *  30 | A15 (PC7)
 * ----+--------------
 *  52 | EEPROM WE
 *  50 | EEPROM OE
 *  51 | EEPROM CE
 * ----+--------------
 *  13 | Activity LED
 * ----+--------------
 *  
 * Copyright 2019, Erik van Zijst <erik.van.zijst@gmail.com>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <Arduino.h>

enum MODE {STANDBY, READ, WRITE};
typedef enum {
  OK,
  E_RESET,      // reset command received
  E_CORRUPT,    // inbound packet corrupt
  E_UNEXPECTED, // unexpected packet received
  E_UNKNOWN     // unknown error
} error;

const unsigned int MAX_PAYLOAD = 63;
const unsigned int DELAY_US = 15;

// AT28C256 contol lines
const unsigned int EEPROM_WE = 52;
const unsigned int EEPROM_OE = 50;
const unsigned int EEPROM_CE = 51;

// 74HC595 control lines
// const unsigned int SHIFT_OE = A3;
// const unsigned int SHIFT_SER = A4;
// const unsigned int SHIFT_RCLK = 12;
// const unsigned int SHIFT_SCLK = 11;
// const unsigned int SHIFT_CLR = 13;

// Activity indicator LED
const unsigned int ACT_LED = 13;

// Data pins (LSB to MSB)
// const unsigned int dataPins[] = {2, 3, 4, 5, 6, 7, 8, 9};

#define DATA_IN  (PINL)
#define DATA_OUT (PORTL)
#define ADDR_H   (PORTC)
#define ADDR_L   (PORTA)


#define DIR_IN  0x00
#define DIR_OUT 0xFF
#define DATA_DIR   DDRL
#define ADDR_H_DIR DDRC
#define ADDR_L_DIR DDRA

MODE mode = STANDBY;
error errno = OK;

int receive(byte *buf, size_t len, bool sendAck);
int send(byte *buf, size_t len, bool waitForAck);
void pulse(int pin);
void loadShiftAddr(unsigned int addr);
byte readAddr(unsigned int addr);
void writeAddr(unsigned int addr, byte val);
int dump();
int load(unsigned int len);
int writeMode();
int readMode();
int standbyMode();
void processError();


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(120000L);

  pinMode(EEPROM_CE, OUTPUT);
  pinMode(EEPROM_OE, OUTPUT);
  pinMode(EEPROM_WE, OUTPUT);

  DATA_DIR = DIR_IN;
  ADDR_H_DIR = DIR_OUT;
  ADDR_L_DIR = DIR_OUT;

  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, LOW);

  standbyMode();
}

/*
 * Reads the next message from the serial port and copies its payload into
 * the specified `buf` byte array.
 * 
 * This function can participate in explicit flow control by sending an
 * explicit acknowledgement message if `sendAck` is `true`.
 *
 * Returns the number of bytes that were copied into `buf` (0 for acks), or -1
 * if there was an error (check global `errno`).
 */
int receive(byte *buf, size_t len, bool sendAck) {
  int l;
  do {
    l = Serial.read();
  } while (l == -1);

  if (l > 0) {
    if (Serial.readBytes(buf, min(l, len)) != l) {
      errno = E_CORRUPT;
      return -1;
    }
  }
  if (sendAck && send(NULL, 0, false) == -1) {
    return -1;
  }
  return l;
}

/*
 * Writes the supplied bytes to the serial port, preceeding it with a length
 * octet.
 *
 * This function enforces flow control, blocking until the client has
 * acknowledged receipt with a 0-byte ack.
 *
 * Returns 0 on success, or -1 if an error occurred, in which case the global
 * `errno` variable gets set.
 */
int send(byte *buf, size_t len, bool waitForAck) {
  Serial.write(len);
  if (len > 0) {
    Serial.write(buf, len);
  }
  if (waitForAck) {
    byte buf[1 + MAX_PAYLOAD];
    int len = receive(buf, MAX_PAYLOAD, false);
    if (len != 0) {
        if (len == 1 && buf[0] == 'r') {
            // reset
            errno = E_RESET;
        } else if (len != -1) {
            errno = E_UNEXPECTED;
        }
      return -1;
    }
  }
  return 0;
}

void pulse(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(DELAY_US);
  digitalWrite(pin, LOW);
  delayMicroseconds(DELAY_US);
}

/*
 * Loads the specified 16 bit address into the 595 shift register.
 */
void loadShiftAddr(unsigned int addr) {
  // ADDR_L = addr && 0xff;
  // ADDR_H = (addr >> 8) && 0xff;
  ADDR_L = addr;
  ADDR_H = addr >> 8;
//  delayMicroseconds(DELAY_US);
}

/*
 * Returns the byte at the specified address.
 */
byte readAddr(unsigned int addr) {
  standbyMode();
  delayMicroseconds(DELAY_US);
  loadShiftAddr(addr);
  delayMicroseconds(DELAY_US);
  readMode();
  delayMicroseconds(DELAY_US);
  byte val = DATA_IN;
  delayMicroseconds(DELAY_US);
  return val;
}

/*
 * Writes a single byte to the specified addess.
 * Requires IO0-IO7 to be set to OUTPUT mode, EEPROM_CE to be LOW and EEPROM_OE
 * to be HIGH prior to invocation.
 */
void writeAddr(unsigned int addr, byte val) {
  loadShiftAddr(addr);

  writeMode();

  // load data byte
  DATA_OUT = val;
  delayMicroseconds(DELAY_US);

  digitalWrite(EEPROM_WE, LOW);
  delayMicroseconds(DELAY_US);
  digitalWrite(EEPROM_WE, HIGH);

  delayMicroseconds(DELAY_US);
  standbyMode();
}

/*
 * Writes the full contents of the EEPROM to the serial port in sequential
 * messages of up to 63 bytes, waiting for explicit acknowledgement of each.
 *
 * Returns 0 on success, -1 on error, in which case the global `errno` variable
 * will be set.
 */
int dump() {
  byte payload[MAX_PAYLOAD];
  unsigned int i = 0;

  for (unsigned int addr = 0; addr < 8192; addr++) {
    i = addr % MAX_PAYLOAD;

    if (addr > 0 && i == 0) {
      // payload at capacity, send out:
      if (send(payload, sizeof(payload), true) == -1) {
        return -1;  // abort immediately
      }
    }
    payload[i++] = readAddr(addr);
  }

  if (i) {
    // send remainder
    return send(payload, i, true);
  }
  return 0;
}

/*
 * Reads the specified number of bytes from the serial port and writes them
 * to the EEPROM.
 *
 * This requires the Arduino to be in WRITE mode.
 *
 * Returns 0 on success, -1 on error, in which case the global `errno` variable
 * gets set.
 */
int load(unsigned int len) {
  unsigned int addr = 0;
  byte buf[1 + MAX_PAYLOAD];
  
  while (addr < len) {
    int cnt = receive(buf, sizeof(buf), true);
    if (cnt == -1) {
      // unexpected error; abort immediately
      return -1;
    }

    for (int i = 0; i < cnt; i++) {
      writeAddr(addr++, buf[i]);
      delay(10);
    }
  }
  return 0;
}

/*
 * Switches the pin mode for the I/O pins to OUTPUT, pulls EEPROM_CE LOW and
 * EEPROM_OE HIGH.
 *
 * Returns 0 on success, or -1 on error.
 */
int writeMode() {
  digitalWrite(EEPROM_CE, LOW);
  digitalWrite(EEPROM_OE, HIGH);
  digitalWrite(EEPROM_WE, HIGH);

  DATA_DIR = DIR_OUT;

  delayMicroseconds(DELAY_US);
  mode = WRITE;
  return 0;
}

/**
 * Switches the pin mode for the I/O pins to INPUT, pulls EEPROM_CE LOW,
 * EEPROM_OE LOW and EEPROM_WE HIGH.
 *
 * Returns 0 on success, or -1 on error.
 */
int readMode() {
  if (mode != READ) {
    DATA_DIR = DIR_IN;

    digitalWrite(EEPROM_CE, LOW);
    digitalWrite(EEPROM_OE, LOW);
    digitalWrite(EEPROM_WE, HIGH);

    delayMicroseconds(DELAY_US);
    mode = READ;
  }
  return 0;
}

int standbyMode() {

  digitalWrite(EEPROM_OE, HIGH);
  digitalWrite(EEPROM_CE, HIGH);

  mode = STANDBY;
  return 0;
}

/**
 * Flashes out the errno value and resets the errno variable.
 */
void processError() {
  // TODO: different patterns for different errors
  if (errno != OK) {
      for (int i = 0; i < 5; i++) {
        digitalWrite(ACT_LED, HIGH);
        delay(100);
        digitalWrite(ACT_LED, LOW);
        delay(100);
      }
  }
  // clear global error state
  errno = OK;
}

void loop() {
  if (Serial.available() > 0) {
    byte buf[1 + MAX_PAYLOAD];
    digitalWrite(ACT_LED, HIGH);

    const int len = receive(buf, sizeof(buf), false);
    if (len > 0) {
        if (buf[0] == 0x72 && len == 3) {
          byte val = readAddr((buf[1] << 8) + buf[2]);
          send(&val, 1, false);

        } else if (buf[0] == 0x77 && len == 4) {
            writeAddr((buf[1] << 8) + buf[2], buf[3]);
            // signal operation completion
            send(NULL, 0, false);

        } else if (buf[0] == 0x64 && len == 1) {
            dump();

        } else if (buf[0] == 0x6c && len == 3) {
            send(NULL, 0, false); // acknowledge cmd message
            load((buf[1] << 8) + buf[2]);

        } else if (buf[0] == 0x72 && len == 1) {
            // ignore reset command

        } else {
          errno = E_UNKNOWN;
        }
    }
    digitalWrite(ACT_LED, LOW);
  }
  processError();
}
