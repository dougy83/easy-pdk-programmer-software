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
#include "esp32_pins.h"
#include "fpdk.h"
#include "fpdkproto.h"
#include <string.h>
#include <stdlib.h>

// Frequency measurement: GPIO interrupt-based edge counting
// (PCNT driver not available in precompiled SDK libraries for this target)

//board specific defines for programing IO
#define _FPDK_CLK2_UP()      digitalWrite(PIN_IC_CLK2, HIGH)
#define _FPDK_CLK2_DOWN()    digitalWrite(PIN_IC_CLK2, LOW)
#define _FPDK_CLK_UP()       digitalWrite(PIN_IC_CLK, HIGH)
#define _FPDK_CLK_DOWN()     digitalWrite(PIN_IC_CLK, LOW)
#define _FPDK_SET_DAT_O(bit) digitalWrite(PIN_IC_PA4, (bit)?HIGH:LOW)
#define _FPDK_SET_DAT_F(bit) digitalWrite(PIN_IC_DAT, (bit)?HIGH:LOW)
#define _FPDK_GET_DAT()      digitalRead(PIN_IC_DAT)
#define _FPDK_SET_CMT(bit)   digitalWrite(PIN_IC_CMT, (bit)?HIGH:LOW)

//general macros for programing IO
#define _FPDK_DelayUS(us)    delayMicroseconds(us)
#define _FPDK_Clock()        { _FPDK_CLK_UP(); _FPDK_DelayUS(1); _FPDK_CLK_DOWN(); }
#define _FPDK_Clock2()       { _FPDK_CLK2_UP(); _FPDK_DelayUS(1); _FPDK_CLK2_DOWN(); }
#define _FPDK_Commit2()      { _FPDK_SET_CMT(1); _FPDK_DelayUS(1); _FPDK_SET_CMT(0); _FPDK_Clock2(); }
#define _FPDK_SendBitO(bit)  { _FPDK_SET_DAT_O(bit); _FPDK_Clock(); }
#define _FPDK_SendBitO2(bit) { _FPDK_SET_DAT_O(bit); _FPDK_Clock2(); }
#define _FPDK_SendBitF(bit)  { _FPDK_SET_DAT_F(bit); _FPDK_Clock(); }
#define _FPDK_RecvBit()      ({ _FPDK_CLK_UP(); _FPDK_DelayUS(1); uint32_t bit=_FPDK_GET_DAT(); _FPDK_CLK_DOWN(); bit; })
#define _FPDK_RecvBit2()     ({ _FPDK_CLK2_UP(); _FPDK_DelayUS(1); uint32_t bit=_FPDK_GET_DAT(); _FPDK_CLK2_DOWN(); bit; })

//volatile counter for GPIO interrupt-based frequency measurement
static volatile uint32_t _freq_pulse_count = 0;

static void IRAM_ATTR _FPDK_FreqPulseIsr(void)
{
    _freq_pulse_count++;
}

//board specific max values
// 9-bit PWM (0-511), 3.3V supply, ×4.9 opamp gain => max 16170 mV
#define FPDK_VDD_DAC_MAX_MV 16170
#define FPDK_VPP_DAC_MAX_MV 16170

//PDK command timings
#define FPDK_VPP_CMD_STABELIZE_DELAYUS  100
#define FPDK_VDD_CMD_STABELIZE_DELAYUS  500
#define FPDK_VPP_R_STABELIZE_DELAYUS    1000
#define FPDK_VDD_R_STABELIZE_DELAYUS    1000
#define FPDK_VPP_EW_STABELIZE_DELAYUS   10000
#define FPDK_VDD_EW_STABELIZE_DELAYUS   10000
#define FPDK_VDD_STOP_DELAYUS           0
#define FPDK_VPP_STOP_DELAYUS           0
#define FPDK_LEAVE_PROG_MODE_DELAYUS    10000
#define FPDK_VDD_CAL_STARTUP_DELAYUS    1000

//current pwm output values, we need to store them so we can set channels seperate
static uint32_t _pwm_vdd;
static uint32_t _pwm_vpp;

//averaged ADC conversions converted to mV
static volatile uint32_t _adc_vref;
static volatile uint32_t _adc_vdd;
static volatile uint32_t _adc_vpp;

//software ADC averaging
#define ADC_AVG_SAMPLES 8
static uint32_t _adc_sample_buf_vdd[ADC_AVG_SAMPLES];
static uint32_t _adc_sample_buf_vpp[ADC_AVG_SAMPLES];
static uint32_t _adc_sample_idx = 0;

static void _FPDK_UpdateAdcReadings(void)
{
    uint32_t vdd_mv = analogReadMilliVolts(PIN_ADC_VDD_SENSE);
    uint32_t vpp_mv = analogReadMilliVolts(PIN_ADC_VPP_SENSE);

    _adc_sample_buf_vdd[_adc_sample_idx] = vdd_mv;
    _adc_sample_buf_vpp[_adc_sample_idx] = vpp_mv;
    _adc_sample_idx = (_adc_sample_idx + 1) % ADC_AVG_SAMPLES;

    uint32_t sum_vdd = 0, sum_vpp = 0;
    for (uint32_t i = 0; i < ADC_AVG_SAMPLES; i++)
    {
        sum_vdd += _adc_sample_buf_vdd[i];
        sum_vpp += _adc_sample_buf_vpp[i];
    }

    _adc_vdd = (sum_vdd * 49 + 5) / 10;   // ×4.9, rounded
    _adc_vpp = (sum_vpp * 49 + 5) / 10;   // ×4.9, rounded
}

static void _FPDK_SetClkOutgoing(void)
{
    pinMode(PIN_IC_CLK, OUTPUT);
    digitalWrite(PIN_IC_CLK, LOW);
}

static void _FPDK_SetClkIncoming(void)
{
    pinMode(PIN_IC_CLK, INPUT);
}

static void _FPDK_SetDatOutgoing(void)
{
    pinMode(PIN_IC_DAT, OUTPUT);
    digitalWrite(PIN_IC_DAT, LOW);
}

static void _FPDK_SetDatIncoming(void)
{
    pinMode(PIN_IC_DAT, INPUT);
}

static void _FPDK_SetPA4Outgoing(void)
{
    pinMode(PIN_IC_PA4, OUTPUT);
    digitalWrite(PIN_IC_PA4, LOW);
}

static void _FPDK_SetPA4Incoming(void)
{
    pinMode(PIN_IC_PA4, INPUT);
}

static void _FPDK_SetPA0Outgoing(void)
{
    pinMode(PIN_IC_CLK2, OUTPUT);
    digitalWrite(PIN_IC_CLK2, LOW);
}

static void _FPDK_SetPA0Incoming(void)
{
    pinMode(PIN_IC_CLK2, INPUT);
}

static void _FPDK_SetPA7Outgoing(void)
{
    pinMode(PIN_IC_CMT, OUTPUT);
    digitalWrite(PIN_IC_CMT, LOW);
}

static void _FPDK_SetPA7Incoming(void)
{
    pinMode(PIN_IC_CMT, INPUT);
}

static void _FPDK_SendBits32O(const uint32_t data, const uint8_t bits)
{
    uint32_t bitdat = data << (32 - bits);
    for (uint32_t p = 0; p < bits; p++)
    {
        _FPDK_SendBitO(bitdat & 0x80000000);
        bitdat <<= 1;
    }
    _FPDK_SET_DAT_O(0);
}

static void _FPDK_SendBits32O2(const uint32_t data, const uint8_t bits)
{
    uint32_t bitdat = data << (32 - bits);
    for (uint32_t p = 0; p < bits; p++)
    {
        _FPDK_SendBitO2(bitdat & 0x80000000);
        bitdat <<= 1;
    }
    _FPDK_SET_DAT_O(0);
}

static void _FPDK_SendBits32F(const uint32_t data, const uint8_t bits)
{
    uint32_t bitdat = data << (32 - bits);
    for (uint32_t p = 0; p < bits; p++)
    {
        _FPDK_SendBitF(bitdat & 0x80000000);
        bitdat <<= 1;
    }
}

static uint32_t _FPDK_RecvBits32(const uint8_t bits)
{
    uint32_t bitdat = 0;
    for (uint32_t p = 0; p < bits; p++)
        bitdat = (bitdat << 1) | (_FPDK_RecvBit() ? 1 : 0);
    return bitdat;
}

static uint32_t _FPDK_RecvBits32O2(const uint8_t bits)
{
    uint32_t bitdat = 0;
    for (uint32_t p = 0; p < bits; p++)
        bitdat = (bitdat << 1) | (_FPDK_RecvBit2() ? 1 : 0);
    return bitdat;
}

static int _FPDK_EnterProgramingmMode(const FPDKICTYPE type, const uint32_t VPP_mV, const uint32_t VDD_mV)
{
    _FPDK_SetClkOutgoing();

    if (!FPDK_SetVPP(VPP_mV, FPDK_VPP_CMD_STABELIZE_DELAYUS))
    {
        FPDK_SetVPP(0, 0);
        return -1;
    }

    if (!FPDK_SetVDD(VDD_mV, FPDK_VDD_CMD_STABELIZE_DELAYUS))
    {
        FPDK_SetVPP(0, 0);
        FPDK_SetVDD(0, 0);
        return -2;
    }

    if (FPDK_IS_FLASH_TYPE(type))
        _FPDK_SetDatOutgoing();
    else
    {
        _FPDK_SetDatIncoming();
        _FPDK_SetPA4Outgoing();
        if (FPDK_IC_OTP3_1 == type)
        {
            _FPDK_SetPA0Outgoing();
            _FPDK_SetPA7Outgoing();
        }
    }
    return 0;
}

static void _FPDK_LeaveProgramingMode(const FPDKICTYPE type, const uint32_t extrawaitus)
{
    _FPDK_SetDatIncoming();
    _FPDK_SetPA4Incoming();
    _FPDK_SetPA0Incoming();
    _FPDK_SetPA7Incoming();
    FPDK_SetVDD(0, FPDK_VDD_STOP_DELAYUS);
    FPDK_SetVPP(0, FPDK_VPP_STOP_DELAYUS);
    _FPDK_DelayUS(FPDK_LEAVE_PROG_MODE_DELAYUS);
    _FPDK_DelayUS(extrawaitus + 1);
    _FPDK_SetClkIncoming();
}

static uint16_t _FPDK_SendCommand(const FPDKICTYPE type, const uint8_t command)
{
    uint32_t ack = 0;
    switch (type)
    {
    case FPDK_IC_FLASH:
    case FPDK_IC_FLASH_1:
        _FPDK_SendBits32F(0xA5A5A5A0 | command, 32);
        _FPDK_SetDatIncoming();
        ack = _FPDK_RecvBits32(16);
        _FPDK_SetDatOutgoing();
        _FPDK_Clock();
        break;

    case FPDK_IC_FLASH_2:
        _FPDK_SendBits32F(0xA5A5A5A0 | command, 32);
        _FPDK_SetDatIncoming();
        ack = _FPDK_RecvBits32(17);
        _FPDK_SetDatOutgoing();
        _FPDK_Clock();
        break;

    case FPDK_IC_OTP1_2:
        _FPDK_SendBits32O(0xA5A5A5A0 | command, 32);
        break;

    case FPDK_IC_OTP2_1:
    case FPDK_IC_OTP2_2:
        _FPDK_SendBits32O(0x5A5A5A50 | command, 32);
        break;

    case FPDK_IC_OTP3_1:
        _FPDK_SendBits32O(0x5A5A5A5A, 32);
        _FPDK_SendBits32O(0x00000000, 27);
        break;
    }
    return ack;
}

static uint32_t _FPDK_ReadAddr(const FPDKICTYPE type, const uint32_t addr, const uint8_t addr_bits, const uint8_t data_bits)
{
    uint32_t dat;
    switch (type)
    {
    case FPDK_IC_FLASH_1:
    case FPDK_IC_FLASH_2:
        _FPDK_SendBits32F(addr, addr_bits);
        _FPDK_SetDatIncoming();
        _FPDK_CLK_UP();
        _FPDK_DelayUS(5);
        dat = _FPDK_RecvBits32(data_bits);
        _FPDK_SetDatOutgoing();
        _FPDK_Clock();
        break;

    case FPDK_IC_OTP3_1:
        _FPDK_SendBits32O2(addr, addr_bits);
        _FPDK_Commit2();
        dat = _FPDK_RecvBits32O2(data_bits);
        break;

    default:
        _FPDK_SendBits32O(addr, addr_bits);
        dat = _FPDK_RecvBits32(data_bits);
    }

    return dat;
}

static void _FPDK_WriteAddr(const FPDKICTYPE type, const uint32_t addr, const uint8_t addr_bits, const uint16_t* data, const uint8_t data_bits,
    const uint32_t count, const uint8_t write_block_clock_groups, const uint8_t write_block_clocks_per_group)
{
    switch (type)
    {
    case FPDK_IC_FLASH_1:
    {
        for (uint32_t p = 0; p < count; p++)
            _FPDK_SendBits32F(data[p], data_bits);

        _FPDK_SendBits32F(addr, addr_bits);
        _FPDK_SetDatIncoming();
        _FPDK_DelayUS(4);

        for (uint32_t l = 0; l < write_block_clock_groups; l++)
        {
            for (uint32_t w = 0; w < write_block_clocks_per_group; w++)
            {
                _FPDK_CLK_UP();
                _FPDK_DelayUS(15);
                _FPDK_CLK_DOWN();
                _FPDK_DelayUS(15);
            }
            _FPDK_Clock();
            _FPDK_DelayUS(4);
        }

        _FPDK_SetDatOutgoing();
    }
    break;

    case FPDK_IC_FLASH_2:
    {
        _FPDK_SendBits32F(addr, addr_bits);

        for (uint32_t p = 0; p < count; p++)
            _FPDK_SendBits32F(data[p], data_bits);

        _FPDK_SetDatIncoming();

        for (uint32_t l = 0; l < write_block_clock_groups; l++)
        {
            _FPDK_Clock();
            _FPDK_DelayUS(2);

            for (uint32_t w = 0; w < write_block_clocks_per_group; w++)
            {
                _FPDK_CLK_UP();
                _FPDK_DelayUS(40);
                _FPDK_CLK_DOWN();
                _FPDK_DelayUS(40);
            }

            for (uint32_t e = 0; e < 4; e++)
            {
                _FPDK_CLK_UP();
                _FPDK_DelayUS(2);
                _FPDK_CLK_DOWN();
                _FPDK_DelayUS(2);
            }
        }

        _FPDK_SetDatOutgoing();
    }
    break;

    case FPDK_IC_OTP3_1:
    {
        //TODO: OTP3_1 write not yet implemented
    }
    break;

    default:
    {
        for (uint32_t p = 0; p < count; p++)
            _FPDK_SendBits32O(data[p], data_bits);

        _FPDK_SendBits32O(addr, addr_bits);
        _FPDK_Clock();
        _FPDK_DelayUS(4);

        for (uint32_t l = 0; l < write_block_clock_groups; l++)
        {
            _FPDK_CLK_UP();
            for (uint32_t w = 0; w < write_block_clocks_per_group; w++)
            {
                _FPDK_SET_DAT_O(1);
                _FPDK_DelayUS(30);
                _FPDK_SET_DAT_O(0);
                _FPDK_DelayUS(30);
            }
            _FPDK_CLK_DOWN();
            _FPDK_DelayUS(4);

            _FPDK_Clock();
            _FPDK_DelayUS(4);
        }
    }
    }
    _FPDK_DelayUS(25);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void FPDK_Init(void)
{
    // De-initialize UART (pins default to bit-bang programming mode)
    Serial1.end();
    pinMode(PIN_IC_CLK2, INPUT);
    pinMode(PIN_IC_CMT, INPUT);

    // Enable DCDC 15V booster
    digitalWrite(PIN_DCDC_ENABLE, HIGH);

    // Initialize PWM outputs to 0
    _pwm_vdd = 0;
    _pwm_vpp = 0;
    ledcWrite(PWM_VDD_CHANNEL, 0);
    ledcWrite(PWM_VPP_CHANNEL, 0);

    // Initialize ADC averaging buffers
    memset((void*)_adc_sample_buf_vdd, 0, sizeof(_adc_sample_buf_vdd));
    memset((void*)_adc_sample_buf_vpp, 0, sizeof(_adc_sample_buf_vpp));
    _adc_sample_idx = 0;
    _adc_vref = 3300;
    _adc_vdd = 0;
    _adc_vpp = 0;

    // Set all programming I/O to input
    _FPDK_SetClkIncoming();
    _FPDK_SetDatIncoming();
    _FPDK_SetPA4Incoming();
    _FPDK_SetPA0Incoming();
    _FPDK_SetPA7Incoming();
}

void FPDK_DeInit(void)
{
    ledcWrite(PWM_VDD_CHANNEL, 0);
    ledcWrite(PWM_VPP_CHANNEL, 0);
    digitalWrite(PIN_DCDC_ENABLE, LOW);
    _FPDK_SetClkIncoming();
    _FPDK_SetDatIncoming();
    _FPDK_SetPA4Incoming();
    _FPDK_SetPA0Incoming();
    _FPDK_SetPA7Incoming();
}

void FPDK_SetLeds(uint32_t val)
{
    // Only one LED on ESP32-C3 super mini (GPIO8, active low)
    digitalWrite(PIN_STATUS_LED, (val & 1) ? LOW : HIGH);
}

void FPDK_SetLed(uint32_t led, bool enable)
{
    if (led == 1)
        digitalWrite(PIN_STATUS_LED, enable ? LOW : HIGH);
}

bool FPDK_IsButtonPressed(void)
{
    static uint32_t press_count = 0;
    static uint32_t tick_press_checked = 0;

    if (millis() > tick_press_checked)
    {
        tick_press_checked = millis() + 1;
        if (digitalRead(PIN_USER_BUTTON) == LOW)     // active low
            press_count++;
        else
            press_count = 0;
    }
    return (press_count > 10);
}

uint32_t FPDK_GetAdcVref(void)
{
    return _adc_vref;
}

uint32_t FPDK_GetAdcVdd(void)
{
    return _adc_vdd;
}

uint32_t FPDK_GetAdcVpp(void)
{
    return _adc_vpp;
}

bool FPDK_SetVDD(uint32_t mV, uint32_t stabelizeDelayUS)
{
    _pwm_vdd = (mV * PWM_MAX_DUTY) / FPDK_VDD_DAC_MAX_MV;

    if (_pwm_vdd > PWM_MAX_DUTY)
        _pwm_vdd = PWM_MAX_DUTY;

    ledcWrite(PWM_VDD_CHANNEL, _pwm_vdd);
    _FPDK_UpdateAdcReadings();

    if (stabelizeDelayUS)
        _FPDK_DelayUS(stabelizeDelayUS);

    return true;
}

bool FPDK_SetVPP(uint32_t mV, uint32_t stabelizeDelayUS)
{
    _pwm_vpp = (mV * PWM_MAX_DUTY) / FPDK_VPP_DAC_MAX_MV;

    if (_pwm_vpp > PWM_MAX_DUTY)
        _pwm_vpp = PWM_MAX_DUTY;

    ledcWrite(PWM_VPP_CHANNEL, _pwm_vpp);
    _FPDK_UpdateAdcReadings();

    if (stabelizeDelayUS)
        _FPDK_DelayUS(stabelizeDelayUS);

    return true;
}

static uint32_t _FPDK_GetIDIC(const FPDKICTYPE type, const uint32_t vpp_cmd, const uint32_t vdd_cmd, const uint8_t databits)
{
    uint32_t ic_id = 0;

    if (_FPDK_EnterProgramingmMode(type, vpp_cmd, vdd_cmd) < 0)
        return FPDK_ERR_VPPVDD;

    switch (type)
    {
    case FPDK_IC_FLASH:
        ic_id = _FPDK_SendCommand(type, 0x6);
        break;

    case FPDK_IC_OTP1_2:
        _FPDK_SendCommand(type, 0x7);
        ic_id = _FPDK_RecvBits32(databits + databits + 12);
        break;

    case FPDK_IC_OTP2_1:
        _FPDK_SendCommand(type, 0x7);
        ic_id = _FPDK_RecvBits32(databits + 12);
        break;

    case FPDK_IC_OTP2_2:
        _FPDK_SendCommand(type, 0x7);
        ic_id = _FPDK_RecvBits32(databits + databits + 12);
        break;

    case FPDK_IC_OTP3_1:
        _FPDK_SendCommand(type, 0);
        ic_id = _FPDK_RecvBits32O2(databits + 1 + 12);
        break;

    default:
        break;
    }
    _FPDK_LeaveProgramingMode(type, 0);

    return ic_id;
}

uint32_t FPDK_ProbeIC(FPDKICTYPE* type, uint32_t* vpp_cmd, uint32_t* vdd_cmd)
{
    *vpp_cmd = 4500;
    *vdd_cmd = 2000;

    uint32_t ic_id = 0;

    if ((ic_id = _FPDK_GetIDIC(FPDK_IC_FLASH, *vpp_cmd, *vdd_cmd, 0)))
        *type = FPDK_IC_FLASH;
    else if ((ic_id = _FPDK_GetIDIC(FPDK_IC_OTP1_2, *vpp_cmd, *vdd_cmd, 16)) & 0xFFFF)
        *type = FPDK_IC_OTP1_2;
    else if ((ic_id = _FPDK_GetIDIC(FPDK_IC_OTP2_2, *vpp_cmd, *vdd_cmd, 16)) & 0xFFFF)
        *type = FPDK_IC_OTP2_2;
    else if ((ic_id = _FPDK_GetIDIC(FPDK_IC_OTP2_1, *vpp_cmd, *vdd_cmd, 16)) & 0xFFFF)
        *type = FPDK_IC_OTP2_1;
    else if ((ic_id = _FPDK_GetIDIC(FPDK_IC_OTP3_1, *vpp_cmd, *vdd_cmd, 16) & 0xFFFF))
        *type = FPDK_IC_OTP3_1;

    return ic_id;
}

uint16_t FPDK_ReadIC(const uint16_t ic_id, const FPDKICTYPE type,
    const uint32_t vpp_cmd, const uint32_t vdd_cmd,
    const uint32_t vpp_read, const uint32_t vdd_read,
    const uint32_t addr, const uint8_t addr_bits,
    uint16_t* data, const uint8_t data_bits,
    const uint32_t count)
{
    if (!FPDK_IS_FLASH_TYPE(type) && (ic_id != (_FPDK_GetIDIC(type, vpp_cmd, vdd_cmd, data_bits) & 0xFFF)))
        return FPDK_ERR_CMDRSP;

    if (_FPDK_EnterProgramingmMode(type, vpp_cmd, vdd_cmd) < 0)
        return FPDK_ERR_VPPVDD;

    uint16_t resp;
    switch (type)
    {
    case FPDK_IC_FLASH_2:
        resp = _FPDK_SendCommand(type, 0xC);
        break;
    default:
        resp = _FPDK_SendCommand(type, 0x6);
        break;
    }
    if (FPDK_IS_FLASH_TYPE(type) && (ic_id != (resp & 0xFFF)))
    {
        _FPDK_LeaveProgramingMode(type, 0);
        return FPDK_ERR_CMDRSP;
    }

    if ((vpp_cmd != vpp_read) || (vdd_cmd != vdd_read))
    {
        if (!FPDK_SetVDD(vdd_read, FPDK_VDD_R_STABELIZE_DELAYUS) ||
            !FPDK_SetVPP(vpp_read, FPDK_VPP_R_STABELIZE_DELAYUS))
        {
            _FPDK_LeaveProgramingMode(type, 0);
            return FPDK_ERR_HVPPHVDD;
        }
    }

    for (uint32_t p = 0; p < count; p++)
        data[p] = _FPDK_ReadAddr(type, addr + p, addr_bits, data_bits);

    _FPDK_LeaveProgramingMode(type, 0);
    return ic_id;
}

uint16_t FPDK_VerifyIC(const uint16_t ic_id, const FPDKICTYPE type,
    const uint32_t vpp_cmd, const uint32_t vdd_cmd,
    const uint32_t vpp_read, const uint32_t vdd_read,
    const uint32_t addr, const uint8_t addr_bits,
    const uint16_t* data, const uint8_t data_bits,
    const uint32_t count,
    const bool addr_exclude_first_instr, const uint32_t addr_exclude_start, const uint32_t addr_exclude_end)
{
    if (!FPDK_IS_FLASH_TYPE(type) && (ic_id != (_FPDK_GetIDIC(type, vpp_cmd, vdd_cmd, data_bits) & 0xFFF)))
        return FPDK_ERR_CMDRSP;

    if (_FPDK_EnterProgramingmMode(type, vpp_cmd, vdd_cmd) < 0)
        return FPDK_ERR_VPPVDD;

    uint16_t resp;
    switch (type)
    {
    case FPDK_IC_FLASH_2:
        resp = _FPDK_SendCommand(type, 0xC);
        break;
    default:
        resp = _FPDK_SendCommand(type, 0x6);
        break;
    }
    if (FPDK_IS_FLASH_TYPE(type) && (ic_id != (resp & 0xFFF)))
    {
        _FPDK_LeaveProgramingMode(type, 0);
        return FPDK_ERR_CMDRSP;
    }

    if ((vpp_cmd != vpp_read) || (vdd_cmd != vdd_read))
    {
        if (!FPDK_SetVDD(vdd_read, FPDK_VDD_R_STABELIZE_DELAYUS) ||
            !FPDK_SetVPP(vpp_read, FPDK_VPP_R_STABELIZE_DELAYUS))
        {
            _FPDK_LeaveProgramingMode(type, 0);
            return FPDK_ERR_HVPPHVDD;
        }
    }

    uint16_t ret = ic_id;
    uint32_t blank_value = (1 << data_bits) - 1;

    for (uint32_t p = 0; p < count; p++)
    {
        if (addr_exclude_first_instr && (0 == addr + p))
            continue;

        if ((p < addr_exclude_start) || (p > addr_exclude_end))
        {
            uint32_t dat = _FPDK_ReadAddr(type, addr + p, addr_bits, data_bits);
            if ((data[p] & blank_value) != (dat & blank_value))
            {
                if ((data[p] & blank_value) != blank_value)
                {
                    ret = FPDK_ERR_VERIFY;
                    break;
                }
            }
        }
    }

    _FPDK_LeaveProgramingMode(type, 0);
    return ret;
}

uint16_t FPDK_BlankCheckIC(const uint16_t ic_id, const FPDKICTYPE type,
    const uint32_t vpp_cmd, const uint32_t vdd_cmd,
    const uint32_t vpp_read, const uint32_t vdd_read,
    const uint8_t addr_bits, const uint8_t data_bits,
    const uint32_t count,
    const bool addr_exclude_first_instr, const uint32_t addr_exclude_start, const uint32_t addr_exclude_end)
{
    if (!FPDK_IS_FLASH_TYPE(type) && (ic_id != (_FPDK_GetIDIC(type, vpp_cmd, vdd_cmd, data_bits) & 0xFFF)))
        return FPDK_ERR_CMDRSP;

    if (_FPDK_EnterProgramingmMode(type, vpp_cmd, vdd_cmd) < 0)
        return FPDK_ERR_VPPVDD;

    uint16_t resp;
    switch (type)
    {
    case FPDK_IC_FLASH_2:
        resp = _FPDK_SendCommand(type, 0xC);
        break;
    default:
        resp = _FPDK_SendCommand(type, 0x6);
        break;
    }
    if (FPDK_IS_FLASH_TYPE(type) && (ic_id != (resp & 0xFFF)))
    {
        _FPDK_LeaveProgramingMode(type, 0);
        return FPDK_ERR_CMDRSP;
    }

    if ((vpp_cmd != vpp_read) || (vdd_cmd != vdd_read))
    {
        if (!FPDK_SetVDD(vdd_read, FPDK_VDD_R_STABELIZE_DELAYUS) ||
            !FPDK_SetVPP(vpp_read, FPDK_VPP_R_STABELIZE_DELAYUS))
        {
            _FPDK_LeaveProgramingMode(type, 0);
            return FPDK_ERR_HVPPHVDD;
        }
    }
    uint32_t blank_value = (1 << data_bits) - 1;

    uint16_t ret = ic_id;

    for (uint32_t p = 0; p < count; p++)
    {
        if (addr_exclude_first_instr && (0 == p))
            continue;

        if ((p < addr_exclude_start) || (p > addr_exclude_end))
        {
            uint32_t dat = _FPDK_ReadAddr(type, p, addr_bits, data_bits);
            if (blank_value != dat)
            {
                ret = FPDK_ERR_NOTBLANK;
                break;
            }
        }
    }

    _FPDK_LeaveProgramingMode(type, 0);
    return ret;
}

uint16_t FPDK_EraseIC(const uint16_t ic_id, const FPDKICTYPE type,
    const uint32_t vpp_cmd, const uint32_t vdd_cmd,
    const uint32_t vpp_erase, const uint32_t vdd_erase,
    const uint8_t erase_clocks)
{
    if (_FPDK_EnterProgramingmMode(type, vpp_cmd, vdd_cmd) < 0)
        return FPDK_ERR_VPPVDD;

    if (!FPDK_IS_FLASH_TYPE(type))
        return FPDK_ERR_UKNOWN;

    uint16_t resp;
    switch (type)
    {
    case FPDK_IC_FLASH_2:
        resp = _FPDK_SendCommand(type, 0x5);
        break;
    default:
        resp = _FPDK_SendCommand(type, 0x3);
        break;
    }
    if (ic_id != (resp & 0xFFF))
    {
        _FPDK_LeaveProgramingMode(type, 0);
        return FPDK_ERR_CMDRSP;
    }

    if (!FPDK_SetVDD(vdd_erase, FPDK_VDD_EW_STABELIZE_DELAYUS) ||
        !FPDK_SetVPP(vpp_erase, FPDK_VPP_EW_STABELIZE_DELAYUS))
    {
        _FPDK_LeaveProgramingMode(type, 0);
        return FPDK_ERR_HVPPHVDD;
    }

    for (uint32_t e = 0; e < erase_clocks; e++)
    {
        _FPDK_CLK_UP();
        _FPDK_DelayUS((FPDK_IC_FLASH_2 == type) ? 40000 : 5000);
        _FPDK_CLK_DOWN();
        _FPDK_DelayUS(1);
        _FPDK_CLK_UP();
        _FPDK_DelayUS(1);
        _FPDK_CLK_DOWN();
        _FPDK_DelayUS(4);
    }

    _FPDK_Clock();
    _FPDK_LeaveProgramingMode(type, 100000);

    return ic_id;
}

uint16_t FPDK_WriteIC(const uint16_t ic_id, const FPDKICTYPE type,
    const uint32_t vpp_cmd, const uint32_t vdd_cmd,
    const uint32_t vpp_write, const uint32_t vdd_write,
    const uint32_t addr, const uint8_t addr_bits,
    const uint16_t* data, const uint8_t data_bits,
    const uint32_t count,
    const uint8_t write_block_size, const uint8_t write_block_clock_groups, const uint8_t write_block_clocks_per_group)
{
    if (!write_block_size || (write_block_size > 8))
        return FPDK_ERR_UKNOWN;

    if (!FPDK_IS_FLASH_TYPE(type) && (ic_id != (_FPDK_GetIDIC(type, vpp_cmd, vdd_cmd, data_bits) & 0xFFF)))
        return FPDK_ERR_CMDRSP;

    if (_FPDK_EnterProgramingmMode(type, vpp_cmd, vdd_cmd) < 0)
        return FPDK_ERR_VPPVDD;

    uint16_t resp = _FPDK_SendCommand(type, 0x7);

    if (FPDK_IS_FLASH_TYPE(type) && (ic_id != (resp & 0xFFF)))
    {
        _FPDK_LeaveProgramingMode(type, 0);
        return FPDK_ERR_CMDRSP;
    }

    if (!FPDK_SetVDD(vdd_write, FPDK_VDD_EW_STABELIZE_DELAYUS) ||
        !FPDK_SetVPP(vpp_write, FPDK_VPP_EW_STABELIZE_DELAYUS))
    {
        _FPDK_LeaveProgramingMode(type, 0);
        return FPDK_ERR_HVPPHVDD;
    }

    uint32_t blank_value = (1 << data_bits) - 1;

    for (uint32_t p = 0; p < count; p += write_block_size)
    {
        uint16_t write_buf[8];
        memset(write_buf, 0xFF, sizeof(write_buf));

        uint32_t write_count = (count > (p + write_block_size - 1)) ? write_block_size : (count - p);
        memcpy(&write_buf[addr % write_block_size], &data[p], write_count * sizeof(uint16_t));

        bool block_is_empty = true;
        for (uint32_t c = 0; c < write_block_size; c++)
        {
            if (blank_value != (write_buf[c] & blank_value))
            {
                block_is_empty = false;
                break;
            }
        }

        if (!block_is_empty)
        {
            uint32_t write_addr_aligned = ((addr + p) / write_block_size) * write_block_size;
            _FPDK_WriteAddr(type,
                write_addr_aligned, addr_bits, write_buf, data_bits,
                write_block_size, write_block_clock_groups, write_block_clocks_per_group);
        }

        _FPDK_DelayUS(100);
    }

    _FPDK_LeaveProgramingMode(type, 100000);

    return ic_id;
}

////////////////////////////
// GPIO interrupt-based frequency measurement for calibration
// Replaces the original SPI slave + DMA + TIM2 and planned PCNT approach
////////////////////////////

// Measure frequency on CLK pin over a given measurement window (in microseconds)
// Uses GPIO interrupt to count rising edges
// Returns frequency in Hz, or 0 on timeout
static uint32_t _FPDK_MeasureFrequency(uint32_t measure_time_us)
{
    _freq_pulse_count = 0;
    delayMicroseconds(measure_time_us);
    uint32_t count = _freq_pulse_count;
    return count * 1000000ULL / measure_time_us;
}

// Calibration support: wait for the target IC to settle
static void _FPDK_CalibrateNext(uint32_t steps)
{
    if (steps > 0)
        delayMicroseconds(steps * 500);
}

static uint8_t _FPDK_CalibrateSingleFrequency(const uint32_t tune_frequency, const uint32_t multiplier,
    const uint8_t minval, const uint8_t maxval, const uint8_t step,
    const bool skipFirstStep, uint32_t* actual_frequency)
{
    *actual_frequency = 0;

    int32_t bestDistance = 100000000;
    uint8_t bestMatch = 0;

    // Choose measurement window based on expected frequency
    uint32_t measure_us = 100000;   // 100ms default
    if (tune_frequency >= 1000000) measure_us = 10000;    // 10ms for MHz range
    else if (tune_frequency >= 100000) measure_us = 50000; // 50ms for 100kHz range

    // Attach interrupt for rising edge detection
    attachInterrupt(digitalPinToInterrupt(PIN_IC_CLK), _FPDK_FreqPulseIsr, RISING);

    if (minval > 0)
        _FPDK_CalibrateNext(minval);

    for (uint16_t t = minval + (skipFirstStep ? 1 : 0); t <= maxval; t += step)
    {
        _FPDK_DelayUS(5000);   // settle after calibration register change

        uint32_t measured_frequency = _FPDK_MeasureFrequency(measure_us) * multiplier;

        if (0 == measured_frequency)
            break;

        int32_t distance = abs((int32_t)measured_frequency - (int32_t)tune_frequency);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestMatch = t;
            *actual_frequency = measured_frequency;
        }

        if (step > 1)
            _FPDK_CalibrateNext(step - 1);
    }

    detachInterrupt(digitalPinToInterrupt(PIN_IC_CLK));
    return bestMatch;
}

static int _FPDK_CalibrateBG(const uint8_t minval, const uint8_t maxval, const uint8_t step)
{
    _FPDK_SetPA0Incoming();

    _FPDK_CalibrateNext(1 + minval);

    for (uint16_t t = minval; t <= maxval; t += step)
    {
        _FPDK_DelayUS(500);

        if (digitalRead(PIN_IC_CLK2) == LOW)
            return t;

        _FPDK_CalibrateNext(step);
    }
    return -1;
}

bool FPDK_Calibrate(const uint32_t type, const uint32_t vdd,
    const uint32_t frequency, const uint32_t multiplier,
    uint8_t* fcalval, uint32_t* freq_tuned)
{
    bool ret = false;

    // Power the target IC
    if (!FPDK_SetVDD(vdd, FPDK_VDD_CAL_STARTUP_DELAYUS))
        return false;

    switch (type)
    {
    case 1: // IHRC
        // CLK pin is shared with PA3 on the target IC
        _FPDK_SetClkIncoming();   // listen to target IC's clock output
        *fcalval = _FPDK_CalibrateSingleFrequency(frequency, multiplier, 0, 0x9F, 1, true, freq_tuned);
        ret = true;
        break;

    case 2: // ILRC
        _FPDK_SetClkIncoming();
        *fcalval = _FPDK_CalibrateSingleFrequency(frequency, multiplier, 0, 0xF0, 0x10, false, freq_tuned);
        ret = true;
        break;

    case 3: // BG
    {
        int v = _FPDK_CalibrateBG(0, 0xF0, 0x10);
        if (v >= 0)
        {
            *fcalval = v;
            ret = true;
        }
    }
    break;
    }

    FPDK_SetVDD(0, 0);
    _FPDK_DelayUS(50000);

    return ret;
}
