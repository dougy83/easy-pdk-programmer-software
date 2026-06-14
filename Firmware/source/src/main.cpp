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
#include "esp32_pins.h"
#include "fpdk.h"
#include "fpdkusb.h"
#include "fpdkuart.h"

void setup(void)
{
    // --- CRITICAL: Wait for system to stabilize before USB init ---
    delay(3000);
    Serial.begin(115200);
    delay(500);

    // --- LEDC PWM for VDD and VPP (9-bit, ~156kHz) ---
    ledcSetup(PWM_VDD_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_VDD_PWM, PWM_VDD_CHANNEL);
    ledcSetup(PWM_VPP_CHANNEL, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_VPP_PWM, PWM_VPP_CHANNEL);

    // --- Programming I/O pins (initial state: input, no pull) ---
    pinMode(PIN_IC_CLK, INPUT);
    pinMode(PIN_IC_PA4, INPUT);
    pinMode(PIN_IC_DAT, INPUT);
    pinMode(PIN_IC_CLK2, INPUT);
    pinMode(PIN_IC_CMT, INPUT);

    // --- Board control ---
    pinMode(PIN_DCDC_ENABLE, OUTPUT);
    digitalWrite(PIN_DCDC_ENABLE, LOW);          // booster off initially
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);           // LED off (active low)
    pinMode(PIN_USER_BUTTON, INPUT_PULLUP);

    // --- ADC resolution ---
    analogReadResolution(12);                     // 12-bit, analogReadMilliVolts handles conversion

    // --- Initialize PDK module ---
    FPDK_Init();
}

void loop(void)
{
    // Handle USB commands from host (Serial = USB CDC with ARDUINO_USB_CDC_ON_BOOT)
    if (Serial)
    {
        while (Serial.available())
        {
            uint8_t b = Serial.read();
            FPDKUSB_USBHandleReceive(&b, 1);
        }
        FPDKUSB_HandleCommands();
        FPDKUART_HandleQueue();
    }
}
