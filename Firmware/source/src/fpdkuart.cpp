/*
Copyright (C) 2019-2020  freepdk  https://free-pdk.github.io

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include "fpdkuart.h"
#include "fpdkusb.h"
#include "esp32_pins.h"
#include <string.h>

#define FPDKUART_QUEUE_SIZE 32      // 32 byte internal RX queue

static uint8_t  _uartRXQueue[FPDKUART_QUEUE_SIZE];
static uint32_t _uartRXQueueWPos;
static uint32_t _uartRXQueueRPos;

void FPDKUART_Init(void)
{
    _uartRXQueueWPos = 0;
    _uartRXQueueRPos = 0;

    // Initialize UART to target IC at fixed 19200 baud
    // Use the pin mapping from esp32_pins.h
    Serial1.end();
    Serial1.begin(TARGET_UART_BAUD, SERIAL_8N1, PIN_IC_UART_RX, PIN_IC_UART_TX);

    // Flush any stale data
    while (Serial1.available())
        Serial1.read();
}

void FPDKUART_DeInit(void)
{
    Serial1.end();
}

void FPDKUART_SendData(const uint8_t* dat, const uint16_t len)
{
    Serial1.write(dat, len);
    Serial1.flush();
}

void FPDKUART_HandleQueue(void)
{
    // Read any available bytes from the target IC UART into our queue
    while (Serial1.available())
    {
        if (_uartRXQueueWPos - _uartRXQueueRPos < FPDKUART_QUEUE_SIZE)
        {
            _uartRXQueue[(_uartRXQueueWPos++) % FPDKUART_QUEUE_SIZE] = (uint8_t)Serial1.read();
        }
        else
        {
            // Queue full, discard oldest byte
            Serial1.read();
        }
    }

    // Forward queued data to host via USB debug
    if (_uartRXQueueWPos > _uartRXQueueRPos)
    {
        uint32_t sendlen = _uartRXQueueWPos - _uartRXQueueRPos;
        uint32_t rpos = _uartRXQueueRPos % FPDKUART_QUEUE_SIZE;
        if ((rpos + sendlen) > FPDKUART_QUEUE_SIZE)
            sendlen = FPDKUART_QUEUE_SIZE - rpos;

        FPDKUSB_SendDebug(&_uartRXQueue[rpos], sendlen);
        _uartRXQueueRPos += sendlen;
    }
}
