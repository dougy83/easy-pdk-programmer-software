# ESP32-C3 Super Mini Port — TODO

## Objective
Port the STM32F072 firmware to ESP32-C3 super mini using **Arduino framework**. Remove all STM32-specific files (HAL drivers, USB middleware, CubeMX code, CMSIS, linker script, startup asm). Keep the application logic (`fpdk.c`, `fpdkusb.c`, `fpdkuart.c`, `fpdkproto.h`) structurally intact — adapt all hardware access to Arduino APIs.

## Constraints
- GPIO 2, 8, 9 are strapping pins. Do not use GPIO 2. GPIO 8 (onboard LED) and GPIO 9 (onboard button, active low) can be used for those purposes.
- ADC on GPIO0-4 only (GPIO2 excluded)
- No hardware DAC — use LEDC PWM (8-bit, high frequency) + external RC filter
- No auto-baud UART — fixed 19200 baud for target IC debug UART
- No -4.8V charge pump PWM needed (hardware never required it)
- No HW variant detection (single board type)

---

## Phase 1: Repository Structure

### Files to delete
```
Firmware/source/Drivers/                 (entire STM32 HAL, ~50 files)
Firmware/source/Middlewares/             (entire USB device library, ~10 files)
Firmware/source/Inc/                     (all STM32 headers)
Firmware/source/startup_stm32f072xb.s
Firmware/source/STM32F072C8Tx_FLASH.ld
Firmware/source/EASYPDKPROG.ioc
Firmware/source/system_stm32f0xx.c
Firmware/source/stm32f0xx_hal_msp.c
Firmware/source/stm32f0xx_it.c
Firmware/source/stm32f0xx_it.h
Firmware/source/stm32f0xx_hal_conf.h
Firmware/source/Src/stm32f0xx_it.c
Firmware/source/Src/system_stm32f0xx.c
Firmware/source/Src/stm32f0xx_hal_msp.c
Firmware/source/Src/usbd_conf.c
Firmware/source/Src/usbd_desc.c
Firmware/source/Src/usb_device.c
Firmware/source/Src/usbd_cdc_if.c
Firmware/source/Src/usbd_cdc_if.h
Firmware/source/Src/main.c
Firmware/source/Inc/main.h
Firmware/source/LICENCE-ADDITONAL
Firmware/source/LICENSE
Firmware/source/README
Firmware/source/Makefile
Firmware/EASYPDKPROG.dfu
Firmware/LICENCE-ADDITONAL
Firmware/README
```

### Files to keep and adapt
```
Firmware/source/Src/fpdk.c          — core PDK programming logic (adapt all HAL calls → Arduino)
Firmware/source/Src/fpdk.h          — enums, types, function prototypes (adapt pin defines)
Firmware/source/Src/fpdkusb.c       — USB command dispatch protocol (adapt USB I/O + HAL_GetTick)
Firmware/source/Src/fpdkusb.h       — keep, adapt includes
Firmware/source/Src/fpdkuart.c      — UART bridge to target IC (replace HAL_UART with Serial1)
Firmware/source/Src/fpdkuart.h      — keep, adapt includes
Firmware/source/Src/fpdkproto.h     — protocol definition (keep as-is, shared with host)
```

### New files to create
```
Firmware/source/src/                  — Arduino sketch directory
Firmware/source/src/main.cpp          — setup() + loop() entry point
Firmware/source/src/esp32_pins.h      — pin mapping header
Firmware/source/platformio.ini        — PlatformIO project config
                               — alternative: Arduino IDE folder structure
```

**Structure note:** PlatformIO expects `src/` and optionally `lib/`. We'll use:
```
Firmware/source/
├── platformio.ini
└── src/
    ├── main.cpp
    ├── esp32_pins.h
    ├── fpdk.c          (adapted)
    ├── fpdk.h          (adapted)
    ├── fpdkusb.c       (adapted)
    ├── fpdkusb.h
    ├── fpdkuart.c      (adapted)
    ├── fpdkuart.h
    └── fpdkproto.h
```

---

## Phase 2: Peripheral Mapping

### STM32 Peripheral → ESP32-C3 Arduino Replacement

| STM32 Peripheral | Function | ESP32-C3 Replacement | Arduino API |
|---|---|---|---|
| DAC1 ch1 (PA4) | VDD voltage (0-6.26V) | LEDC PWM (8-bit, ~50kHz), GPIO0 → RC filter → opamp | `ledcSetup()` + `ledcAttachPin()` + `ledcWrite()` |
| DAC1 ch2 (PA5) | VPP voltage (0-13.3V) | LEDC PWM (8-bit, ~50kHz), GPIO1 → RC filter → opamp | `ledcSetup()` + `ledcAttachPin()` + `ledcWrite()` |
| ADC1 ch8 (PB0) | VPP voltage sense | ADC1, GPIO4 (voltage divider from VPP rail) | `analogReadMilliVolts()` |
| ADC1 ch9 (PB1) | VDD voltage sense | ADC1, GPIO3 (voltage divider from VDD rail) | `analogReadMilliVolts()` |
| ADC1 VREFINT | Internal reference | ESP32-C3 factory-calibrated ADC | `analogReadMilliVolts()` handles this |
| TIM1 | ADC trigger timer | Not needed — poll ADC in main loop (no DMA) | `millis()` for timing |
| TIM15 ch2 (PA3) | PWM for -4.8V | **Not needed** — hardware never required it | — |
| SPI1 slave (PB3-5) | Frequency measurement (calibration) | **PCNT (pulse counter)** peripheral on CLK pin | ESP-IDF `pcnt_` functions via `esp32-hal-pcnt.h` or direct register access |
| USART1 (PB6/7) | Target IC debug UART | UART1 on GPIO20(TX)/21(RX), fixed 19200 baud | `Serial1.begin(19200, SERIAL_8N1, 21, 20)` |
| GPIO | Bit-banged CLK/DAT/CMT | Any available GPIO | `digitalWrite()` / `digitalRead()` / `pinMode()` |
| USB + CDC | Host communication | **USBSerial** — built-in USB Serial/JTAG on GPIO18/19 | `USBSerial` object (read/write/available) |
| SysTick | HAL_GetTick() millis | Built-in | `millis()` |

### Pin Assignment (Final)

| Function | GPIO | Notes |
|---|---|---|
| VDD_PWM | GPIO0 | LEDC channel 0, 8-bit, ~50kHz → RC filter → opamp → target VDD |
| VPP_PWM | GPIO1 | LEDC channel 1, 8-bit, ~50kHz → RC filter → opamp → target VPP |
| ADC_VDD_SENSE | GPIO3 | analogReadMilliVolts(), voltage divider ×5.7 from VDD rail |
| ADC_VPP_SENSE | GPIO4 | analogReadMilliVolts(), voltage divider ×5.7 from VPP rail |
| IC_IO_PA3_CLK | GPIO5 | bit-bang CLK / PCNT input during calibration |
| IC_IO_PA4 | GPIO6 | bit-bang secondary data line |
| IC_IO_PA6_DAT | GPIO7 | bit-bang main data line |
| STATUS_LED | GPIO8 | Onboard LED (active low — verify polarity) |
| USER_BUTTON | GPIO9 | Onboard button, active low, INPUT_PULLUP internally |
| DCDC_ENABLE | GPIO10 | DCDC 15V booster enable (active high) |
| IC_UART_TX | GPIO20 | Serial1 TX to target IC (CLK2 for OTP3_1) |
| IC_UART_RX | GPIO21 | Serial1 RX from target IC (CMT for OTP3_1) |
| USB D- | GPIO18 | Fixed by ESP32-C3 hardware (USB Serial/JTAG) |
| USB D+ | GPIO19 | Fixed by ESP32-C3 hardware (USB Serial/JTAG) |
| GPIO2 | — | Strapping pin — do not connect |

**Total application GPIOs:** 12 (GPIO0,1,3,4,5,6,7,8,9,10,20,21)

---

## Phase 3: Calibration — Replace SPI Slave with PCNT

The calibration trick (target IC outputs clock on CLK pin, firmware measures frequency) needs a different implementation on ESP32-C3.

**Original approach:** SPI slave + DMA + TIM2 — counts SPI transactions over timed window using double-buffered DMA callbacks.

**ESP32-C3 approach:** Use the **PCNT (Pulse Counter)** peripheral to count rising edges on the CLK pin (GPIO5) over a timed window measured with `micros()` or an ESP32-C3 timer.

```
PCNT setup:
  - unit 0, channel 0 on GPIO5
  - count rising edges, filter enabled
  - set up control loop to start PCNT, delay for measurement window, stop PCNT, read count

Frequency = pulse_count / measurement_time_seconds
```

**PCNT is simpler** — no SPI slave mode needed, no DMA, no pin mode switching on CLK/PA4/DAT. Just count pulses on one pin. The CLK pin stays in input mode during calibration (it already has an input mode function `_FPDK_SetClkIncoming()`).

**For the calibration algorithm:** The `_FPDK_CalibrateSingleFrequency()` function iterates through calibration values and measures frequency for each. This maps cleanly:

| Original | Replacement |
|---|---|
| `_FPDK_CalibrateNext(steps)` — sets up SPI DMA to pulse target IC | `delayMicroseconds()` — target IC is already powered and running; just wait |
| `_FPDK_CalibrateGetNextFreqeuncy()` — reads SPI DMA result | PCNT reset, delay, read count, compute frequency |
| `HAL_SPI_TxRxHalfCpltCallback/CpltCallback` | **Not needed** — PCNT handles counting |
| TIM2 for time measurement | `micros()` before and after measurement window |

**Exception — Bandgap calibration (`_FPDK_CalibrateBG`):** This doesn't measure frequency. Instead it monitors a GPIO pin level (PA0) while stepping through calibration values. The original used SPI DMA just as a delay mechanism to advance trough calibration steps. With PCNT, we can use `delayMicroseconds()` instead.

---

## Phase 4: HAL Call Replacements (by file)

### `fpdk.c` — ~60 HAL call sites

| STM32 Pattern | Arduino Replacement |
|---|---|
| `HAL_GPIO_WritePin(port, pin, state)` | `digitalWrite(pin, state)` |
| `HAL_GPIO_ReadPin(port, pin)` | `digitalRead(pin)` |
| `HAL_GPIO_Init(port, &init_struct)` | `pinMode(pin, mode)` — mode is `OUTPUT`, `INPUT`, `INPUT_PULLUP`, `INPUT_PULLDOWN` |
| `HAL_DAC_Start/DualSetValue(12bit)` | `ledcWrite(channel, 8bit_duty)` — map mV→duty using `V_max_mV/256` |
| `HAL_ADC_Init/ConfigChannel/Start_DMA` | `analogReadMilliVolts(pin)` — polled, no DMA, no double-buffering |
| `HAL_ADCEx_Calibration_Start` | **Not needed** — `analogReadMilliVolts()` uses factory calibration |
| `HAL_TIM_Base_Start(&htim1)` | **Not needed** — ADC is polled, no timer trigger |
| `HAL_TIM_Base_Start(&htim2)` | **Not needed** — PCNT uses `micros()` for timing |
| `HAL_TIM_PWM_Start(&htim15, ch2)` | **Not needed** — no -4.8V charge pump |
| `HAL_SPI_Init + TransmitReceive_DMA` + callbacks | **PCNT** — see Phase 3 |
| `HAL_SPI_DeInit` | `pcnt_unit_disable()` |
| `HAL_Delay(ms)` | `delay(ms)` |
| `_FPDK_DelayUS(us)` inline asm | `delayMicroseconds(us)` |

**`FPDK_Init()` changes:**
- Remove `HAL_SPI_DeInit`, `HAL_UART_DeInit` — no need to undo CubeMX defaults
- Replace `HAL_TIM_PWM_Start` for -4.8V → **remove entirely**
- Replace `HAL_DAC_Start` → `ledcSetup()` + `ledcAttachPin()` for both VDD and VPP channels
- Replace `HAL_DACEx_DualSetValue` → `ledcWrite()` for each channel
- Replace `HAL_GPIO_WritePin(DCDC_ENABLE)` → `digitalWrite(DCDC_ENABLE, HIGH)`
- **Remove HW variant detection** (PB8/PB9 + ADC channel reconfig) — single board type
- Replace `HAL_ADCEx_Calibration_Start + HAL_ADC_Start_DMA + HAL_TIM_Base_Start` → **nothing** (ADC polled later)
- Keep `HAL_TIM_Base_Start(&htim2)` → **replace with nothing** (PCNT + micros replaces TIM2)

**`FPDK_DeInit()` changes:**
- Replace `HAL_DACEx_DualSetValue(0,0)` → `ledcWrite(vdd_ch, 0)`, `ledcWrite(vpp_ch, 0)`
- Replace `HAL_GPIO_WritePin(DCDC_ENABLE, RESET)` → `digitalWrite(DCDC_ENABLE, LOW)`
- Replace `HAL_ADC_Stop` → **nothing**

**`FPDK_SetVDD(mV, delayUS)` / `FPDK_SetVPP(mV, delayUS)`:**
- Replace `HAL_DACEx_DualSetValue(12bit)` → `ledcWrite(ch, mV * 255 / V_max_mV)`
- Replace `_FPDK_DelayUS(stabelizeDelayUS)` → `delayMicroseconds(stabelizeDelayUS)`

**ADC reading (`FPDK_GetAdcVref/Vdd/Vpp`):**
- Replace DMA double-buffered ADC with polled `analogReadMilliVolts()`
- The original averaged 8 samples over DMA double-buffer. Replace with simple rolling average in software: keep last N readings, return average.
- `FPDK_GetAdcVref()` — ESP32-C3 has no VREFINT monitor. Just return 3300 (3.3V supply nominal). The host only uses this for informational display.
- `FPDK_GetAdcVdd()` — `analogReadMilliVolts(GPIO3) * 5.7`
- `FPDK_GetAdcVpp()` — `analogReadMilliVolts(GPIO4) * 5.7`

**`FPDK_Calibrate()` function:**
- Replace entire SPI+DMA+callback block with PCNT approach
- Keep the frequency sweep algorithm structure, just change the measurement method

### `fpdkusb.c` — USB command dispatch

| STM32 Pattern | Arduino Replacement |
|---|---|
| `HAL_GetTick()` | `millis()` |
| `CDC_Transmit_FS(buf, len)` | `USBSerial.write(buf, len)` |
| `CDC_IsHostPortOpen()` | `(bool)USBSerial` or track `USBSerial` availability |
| `FPDKUSB_USBHandleReceive(buf, len)` | Called from `loop()` via `USBSerial.readBytes()` |
| `USBD_BUSY` check | `USBSerial` handles buffering; add local TX buffer if needed |

**`usbd_cdc_if.c` replacement:** This entire file is replaced by direct `USBSerial` usage in `fpdkusb.c`:
- `FPDKUSB_Init()` → **nothing** (USBSerial is auto-initialized by Arduino core)
- `FPDKUSB_DeInit()` → **nothing**
- `FPDKUSB_IsConnected()` → `USBSerial` (evaluates to true if connected)
- `FPDKUSB_USBSignalPortOpenClose()` → track via USBSerial state in main loop
- `FPDKUSB_USBHandleReceive()` → called with data from `USBSerial.read()` in main loop
- `CDC_Transmit_FS()` → `USBSerial.write()`

### `fpdkuart.c` — UART bridge

| STM32 Pattern | Arduino Replacement |
|---|---|
| `HAL_UART_Init/DeInit` | `Serial1.begin(19200, SERIAL_8N1, rxPin, txPin)` / `Serial1.end()` |
| `HAL_UART_Receive_DMA(buf, size)` + callbacks | `Serial1.read()` in main loop + software ring buffer |
| `HAL_UART_Transmit_IT(buf, len)` | `Serial1.write(buf, len)` |
| `__HAL_UART_GET_FLAG(ABRF/ABRE)` | **Remove entirely** — fixed baud, no auto-baud logic |
| `HAL_RCC_GetPCLK1Freq() / USART1->BRR` | Remove the auto-baud connect message or just report "Connected @19200 baud" |

**Simplified flow:**
1. `FPDKUART_Init()` → `Serial1.begin(19200, SERIAL_8N1, rxPin=21, txPin=20)`
2. `FPDKUART_HandleQueue()` → read `Serial1.available()` and `Serial1.read()` into ring buffer, then forward via `FPDKUSB_SendDebug()`
3. `FPDKUART_SendData()` → `Serial1.write()`
4. `FPDKUART_DeInit()` → `Serial1.end()`
5. **Remove** auto-baud logic entirely (no more `_uartRXAutoBaudFinished` flag, no connect string)

The double-buffered DMA UART RX pattern becomes a simple software ring buffer filled by `Serial1.read()` calls.

### `main.c` → `main.cpp` — Complete replacement

```cpp
// Firmware/source/src/main.cpp
#include <Arduino.h>
#include "esp32_pins.h"
#include "fpdk.h"
#include "fpdkusb.h"
#include "fpdkuart.h"

void setup() {
    // USB serial for host protocol (auto-init by Arduino core on ESP32-C3)
    
    // LEDC PWM for VDD and VPP (8-bit, ~50kHz)
    ledcSetup(PWM_VDD_CHANNEL, 50000, 8);
    ledcAttachPin(PIN_VDD_PWM, PWM_VDD_CHANNEL);
    ledcSetup(PWM_VPP_CHANNEL, 50000, 8);
    ledcAttachPin(PIN_VPP_PWM, PWM_VPP_CHANNEL);
    
    // Programming I/O pins
    pinMode(PIN_IC_CLK, OUTPUT);
    pinMode(PIN_IC_PA4, OUTPUT);
    pinMode(PIN_IC_DAT, OUTPUT);
    // (CLK2 and CMT are configured by Serial1 or by _FPDK_Set* functions)
    
    // Board I/O
    pinMode(PIN_DCDC_ENABLE, OUTPUT);
    digitalWrite(PIN_DCDC_ENABLE, LOW);
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_USER_BUTTON, INPUT_PULLUP);
    
    // ADC resolution
    analogReadResolution(12);  // 12-bit, but analogReadMilliVolts handles conversion
    
    // Initialize PDK module
    FPDK_Init();
}

void loop() {
    // Handle USB commands from host
    if (USBSerial) {  // host port open
        // Read available USB data into packet buffer
        while (USBSerial.available()) {
            uint8_t b = USBSerial.read();
            FPDKUSB_USBHandleReceive(&b, 1);
        }
        FPDKUSB_HandleCommands();
        FPDKUART_HandleQueue();
    }
}
```

---

## Phase 5: Build System — PlatformIO

### `platformio.ini`

```ini
[env:esp32-c3-devkitm-1]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
board_build.flash_mode = dio
board_build.f_cpu = 160000000L

[env:esp32-c3-super-mini]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
board_build.flash_mode = dio
board_build.f_cpu = 160000000L
```

**Build commands:**
```bash
cd Firmware/source/
pio run                     # build
pio run -t upload           # build + flash
pio device monitor          # serial monitor
```

---

## Phase 6: Key Voltage/Timing Constants

The original firmware has hardware-specific constants that must be recalculated for the ESP32-C3 hardware:

| Constant | Original (STM32) | ESP32-C3 Replacement |
|---|---|---|
| External circuit gain | — | ×5.7 (opamp feedback resistor ratio) |
| Max output at 100% PWM (3.3V × 5.7) | — | 18810 mV |
| `FPDK_VDD_DAC_MAX_MV` | 6260 mV | Keep as `18810` for duty calculation. Actual max VDD limited by target IC needs (~5.5V). |
| `FPDK_VPP_DAC_MAX_MV` | 13270 mV | Keep as `18810` for duty calculation. Actual max VPP limited to ~13.3V by original design. |
| PWM duty for VDD at 6.26V | — | `6.26 / 18.81 * 255 ≈ 85` |
| PWM duty for VPP at 13.3V | — | `13.3 / 18.81 * 255 ≈ 180` |
| DAC value formula | `mV × 4095 / VDD_DAC_MAX_MV` | `mV × 255 / 18810` |
| VDD step size | ~1.5 mV (12-bit) | 18810 / 256 ≈ 73.5 mV (8-bit) |
| VPP step size | ~3.2 mV (12-bit) | 18810 / 256 ≈ 73.5 mV (8-bit) |
| ADC measurement | DMA + 8-sample avg, VREFINT-calibrated | Polled + software rolling average |
| ADC conversion formula | `vref × raw × 6 >> 15` | `analogReadMilliVolts(pin) × 5.7` |
| Voltage divider ratio | ×6 | ×5.7 (same resistors on opamp feedback) |
| VREF monitor | Internal VREFINT_CAL (factory trim) | Return 3300 (3.3V supply nominal)

---

## Phase 7: Functional Verification Checklist

- [ ] Host tool detects programmer via USB CDC (FPDKCOM_OpenAuto)
- [ ] Host gets version info (GETVERINFO)
- [ ] Host probes an IC successfully (PROBEIC)
- [ ] Voltage control works: set VDD to 3.0V, 4.5V, 5.0V — verify with multimeter + ADC readback
- [ ] Voltage control works: set VPP to 5.0V, 8.0V, 11.0V — verify
- [ ] Bit-banged CLK/DAT timing matches original (logic analyzer — verify `delayMicroseconds` accuracy)
- [ ] PCNT-based frequency calibration for IHRC matches original results
- [ ] PCNT-based frequency calibration for ILRC
- [ ] Bandgap calibration via GPIO read works
- [ ] Blank check, read, write, verify, erase operations on a target IC
- [ ] UART debug bridge works during IC execution (19200 baud)
- [ ] Fuse writing works
- [ ] Protocol response times under 50ms for simple commands, under 10s for calibration
- [ ] Onboard button (GPIO9) for user input
- [ ] Onboard LED (GPIO8) status indication
- [ ] Reset/reboot behavior — programmer comes up in safe state (VDD/VPP = 0)

---

## Phase 8: Key Risks

| Risk | Impact | Mitigation |
|---|---|---|
| `delayMicroseconds()` accuracy at 1us | PDK CLK pulses are 1us high/low — wrong pulse width causes bit errors | Profile with `digitalWriteFast` macros or direct GPIO register access; add fine-tuning loop if needed |
| PWM ripple not fully filtered by RC network | Voltage noise on VDD/VPP line can corrupt programming | Verify with oscilloscope; increase PWM frequency or add 2nd order filter |
| `analogReadMilliVolts()` latency (~100us) | ADC not sampled synchronously with VDD/VPP changes | Acceptable — ADC is for monitoring, not control loop |
| 9-bit resolution (~32mV steps) | Acceptable for PDK programming voltage levels | 512 steps across 0-16.17V range should be sufficient for all target ICs |
| PCNT peripheral not available in Arduino | Calibration won't work | Use ESP-IDF `pcnt_` functions directly (included in Arduino core's `esp32-hal-pcnt.h`), or fall back to GPIO interrupt counting |
| USBSerial buffers overflow | Lost USB command data | Increase USB buffer size if needed; or add flow control |
| `pinMode()` + `digitalWrite()` too slow for bit-banging | CLK pulses too wide, programming too slow | Use direct GPIO register writes (`GPIO.out_w1ts` / `GPIO.out_w1tc`) for critical timing paths |

---

## Resolved Decisions

| Question | Decision |
|---|---|
| PCNT availability | Use PCNT first. ESP32-C3 has PCNT peripheral, header `<driver/pcnt.h>` available in Arduino via ESP-IDF. Fallback to GPIO interrupt ISR if PCNT has issues. |
| Voltage divider ratio | ×5.7 for both VDD and VPP. 100% PWM (3.3V × 5.7) = 18.81V max. Same divider used for ADC sense. |
