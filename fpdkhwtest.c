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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/select.h>

#include "fpdkcom.h"
#include "fpdkicdata.h"
#include "fpdkutil.h"
#include "fpdkhwtest.h"

// portable millisecond sleep using select()
static void _msleep(unsigned int ms)
{
    struct timeval tv;
    tv.tv_sec = ms / 1000;
    tv.tv_usec = (long)(ms % 1000) * 1000L;
    select(0, NULL, NULL, NULL, &tv);
}

static bool _prompt_yesno(const char* question)
{
    printf("%s (y/n): ", question);
    fflush(stdout);
    int c = getchar();
    // consume rest of line
    while (c != '\n' && c != EOF)
    {
        int next = getchar();
        if (next == '\n' || next == EOF)
            break;
        c = next;
    }
    return (c == 'y' || c == 'Y');
}

static const char* _passfail(bool pass)
{
    return pass ? "PASS" : "FAIL";
}

int fpdk_hwtest(const int comfd)
{
    unsigned int tests_passed = 0;
    unsigned int tests_total  = 0;

    printf("\n========================================\n");
    printf("  Easy PDK Programmer Hardware Test\n");
    printf("========================================\n\n");

    // ========================================================================
    // Step 1: Version / Connection
    // ========================================================================
    printf("--- Step 1: Connection & Version ---\n");

    unsigned int hw_major, hw_minor, sw_major, sw_minor, proto_major, proto_minor;
    if (!FPDKCOM_GetVersion(comfd, &hw_major, &hw_minor, &sw_major, &sw_minor, &proto_major, &proto_minor))
    {
        printf("  FAIL: Could not get version info from programmer.\n");
        return -1;
    }

    printf("  Hardware: %u.%u  Firmware: %u.%u  Protocol: %u.%u\n",
           hw_major, hw_minor, sw_major, sw_minor, proto_major, proto_minor);

    // ========================================================================
    // Step 2: Power Supply — VDD
    // ========================================================================
    printf("\n--- Step 2: VDD Power Supply ---\n");
    {
        printf("  Setting VDD to 1.65V...\n");
        if (!FPDKCOM_SetOutputVoltages(comfd, 1.65f, 0.0f))
        {
            printf("  FAIL: Could not set VDD.\n");
            return -1;
        }
        _msleep(200);  // 200ms settle time

        float vdd_read, vpp_read, vref_read;
        if (!FPDKCOM_MeasureOutputVoltages(comfd, &vdd_read, &vpp_read, &vref_read))
        {
            printf("  FAIL: Could not read voltages.\n");
            return -1;
        }
        printf("  VDD reported: %.3f V\n", vdd_read);

        bool ok = (vdd_read > 1.40f && vdd_read < 1.90f);
        printf("  Result: %s (expected ~1.65V, tolerance ±0.25V)\n", _passfail(ok));
        tests_total++;
        if (ok) tests_passed++;

        if (!_prompt_yesno("  Measure VDD pin (GPIO0 PWM out) with multimeter — is it ~1.65V?"))
        {
            printf("  User-reported: FAIL\n");
        }
        else
        {
            printf("  User-reported: PASS\n");
        }
    }

    printf("\n  Setting VDD to 3.30V...\n");
    {
        if (!FPDKCOM_SetOutputVoltages(comfd, 3.30f, 0.0f))
        {
            printf("  FAIL: Could not set VDD.\n");
            return -1;
        }
        _msleep(200);

        float vdd_read, vpp_read, vref_read;
        if (!FPDKCOM_MeasureOutputVoltages(comfd, &vdd_read, &vpp_read, &vref_read))
        {
            printf("  FAIL: Could not read voltages.\n");
            return -1;
        }
        printf("  VDD reported: %.3f V\n", vdd_read);

        bool ok = (vdd_read > 2.80f && vdd_read < 3.80f);
        printf("  Result: %s (expected ~3.30V, tolerance ±0.50V)\n", _passfail(ok));
        tests_total++;
        if (ok) tests_passed++;

        _prompt_yesno("  Measure VDD pin — is it ~3.30V? (y/n)");
    }

    // ========================================================================
    // Step 3: Power Supply — VPP / DCDC Booster
    // ========================================================================
    printf("\n--- Step 3: VPP Power Supply & DCDC Booster ---\n");
    {
        printf("  Setting VPP to 5.00V...\n");
        if (!FPDKCOM_SetOutputVoltages(comfd, 0.0f, 5.00f))
        {
            printf("  FAIL: Could not set VPP.\n");
            return -1;
        }
        _msleep(200);

        float vdd_read, vpp_read, vref_read;
        if (!FPDKCOM_MeasureOutputVoltages(comfd, &vdd_read, &vpp_read, &vref_read))
        {
            printf("  FAIL: Could not read voltages.\n");
            return -1;
        }
        printf("  VPP reported: %.3f V\n", vpp_read);

        bool ok = (vpp_read > 4.00f && vpp_read < 6.00f);
        printf("  Result: %s (expected ~5.00V, tolerance ±1.0V)\n", _passfail(ok));
        tests_total++;
        if (ok) tests_passed++;

        _prompt_yesno("  Measure VPP pin (GPIO1 PWM out) — is it ~5.00V?");
    }

    printf("\n  Setting VPP to 9.00V (DCDC booster engaged)...\n");
    {
        if (!FPDKCOM_SetOutputVoltages(comfd, 0.0f, 9.00f))
        {
            printf("  FAIL: Could not set VPP.\n");
            return -1;
        }
        _msleep(500);

        float vdd_read, vpp_read, vref_read;
        if (!FPDKCOM_MeasureOutputVoltages(comfd, &vdd_read, &vpp_read, &vref_read))
        {
            printf("  FAIL: Could not read voltages.\n");
            return -1;
        }
        printf("  VPP reported: %.3f V\n", vpp_read);

        bool ok = (vpp_read > 7.50f && vpp_read < 10.50f);
        printf("  Result: %s (expected ~9.00V, tolerance ±1.5V)\n", _passfail(ok));
        tests_total++;
        if (ok) tests_passed++;

        _prompt_yesno("  Measure VPP pin — is it ~9.00V?");
    }

    // ========================================================================
    // Step 4: LED
    // ========================================================================
    printf("\n--- Step 4: Status LED ---\n");
    {
        printf("  Blinking LED 5 times...\n");
        for (int i = 0; i < 5; i++)
        {
            FPDKCOM_SetLed(comfd, 0x01);
            _msleep(200);
            FPDKCOM_SetLed(comfd, 0x00);
            _msleep(200);
        }

        bool ok = _prompt_yesno("  Did the status LED blink 5 times?");
        printf("  Result: %s\n", _passfail(ok));
        tests_total++;
        if (ok) tests_passed++;
    }

    // ========================================================================
    // Step 5: Button
    // ========================================================================
    printf("\n--- Step 5: User Button ---\n");
    {
        printf("  Please press the button on the programmer...\n");
        bool pressed = false;
        for (int i = 0; i < 200; i++)
        {
            bool state;
            if (FPDKCOM_GetButtonState(comfd, &state) && state)
            {
                pressed = true;
                break;
            }
            _msleep(100);  // 100ms intervals, 20s total timeout
        }

        printf("  Result: %s\n", _passfail(pressed));
        tests_total++;
        if (pressed) tests_passed++;
    }

    // ========================================================================
    // Step 6: GPIO Probe Waveform
    // ========================================================================
    printf("\n--- Step 6: GPIO Probe Waveform ---\n");
    {
        printf("  Probing for IC (bit-bangs CLK and DAT on GPIO5/GPIO7)...\n");
        printf("  Check these pins with an oscilloscope:\n");
        printf("    - CLK  (GPIO5): ~1MHz burst of clock pulses\n");
        printf("    - DAT  (GPIO7): data toggling synchronous with CLK\n");
        printf("    - PA4  (GPIO6): secondary data line (may toggle)\n");
        printf("    - CLK2 (GPIO20): alternate clock (may be idle)\n");
        printf("    - CMT  (GPIO21): commit signal (may be idle)\n\n");

        float vpp, vdd;
        FPDKICTYPE type;
        int icid = FPDKCOM_IC_Probe(comfd, &vpp, &vdd, &type);

        if (icid <= 0)
            printf("  (No IC found — that's expected if no target is connected)\n");
        else
            printf("  IC found: ID=0x%X type=%s\n", icid,
                   FPDK_IS_FLASH_TYPE(type) ? "FLASH" : "OTP");

        bool ok = _prompt_yesno("  Did you see the probe waveform on CLK (GPIO5)?");
        printf("  Result: %s\n", _passfail(ok));
        tests_total++;
        if (ok) tests_passed++;
    }

    // ========================================================================
    // Step 7: UART Loopback
    // ========================================================================
    printf("\n--- Step 7: UART Loopback ---\n");
    {
        bool ok = _prompt_yesno("  Jumper UART TX (GPIO20) to UART RX (GPIO21), then press y");

        if (ok)
        {
            printf("  Starting IC execution (powers target VDD + inits UART)...\n");
            if (!FPDKCOM_IC_StartExecution(comfd, 3.3f))
            {
                printf("  FAIL: Could not start execution.\n");
                return -1;
            }
            _msleep(100);

            const char testmsg[] = "Hello PDK!";
            printf("  Sending: \"%s\"\n", testmsg);
            if (!FPDKCOM_IC_SendDebugData(comfd, (const uint8_t*)testmsg, strlen(testmsg)))
            {
                printf("  FAIL: Could not send debug data.\n");
                FPDKCOM_IC_StopExecution(comfd);
                return -1;
            }

            // Wait for UART loopback at 19200 baud
            _msleep(100);

            uint8_t rxbuf[64];
            memset(rxbuf, 0, sizeof(rxbuf));
            int rxlen = FPDKCOM_IC_ReceiveDebugData(comfd, rxbuf, sizeof(rxbuf) - 1);

            bool loopback_ok = false;
            if (rxlen > 0)
            {
                rxbuf[rxlen] = '\0';
                printf("  Received: \"%s\" (%d bytes)\n", (char*)rxbuf, rxlen);
                loopback_ok = (rxlen == (int)strlen(testmsg) &&
                               memcmp(rxbuf, testmsg, rxlen) == 0);
            }
            else
            {
                printf("  Received: nothing\n");
            }

            FPDKCOM_IC_StopExecution(comfd);

            printf("  Result: %s\n", _passfail(loopback_ok));
            tests_total++;
            if (loopback_ok) tests_passed++;
        }
        else
        {
            printf("  Skipped.\n");
        }
    }

    // ========================================================================
    // Power Down
    // ========================================================================
    printf("\n  Powering down...\n");
    FPDKCOM_SetOutputVoltages(comfd, 0.0f, 0.0f);
    _msleep(100);

    // ========================================================================
    // Summary
    // ========================================================================
    printf("\n========================================\n");
    printf("  Test Summary: %u / %u passed\n", tests_passed, tests_total);
    printf("========================================\n");

    return (tests_passed == tests_total) ? 0 : -1;
}
