#ifndef ESP32_PINS_H
#define ESP32_PINS_H

// ============================================================
// ESP32-C3 Super Mini — Easy PDK Programmer Pin Assignment
// ============================================================
// GPIO 2, 8, 9 are strapping pins.
// GPIO 2: DO NOT USE
// GPIO 8: onboard LED (active low)
// GPIO 9: onboard button (active low, external pull-up)
// ============================================================

// --- PWM (Replaces STM32 DAC) ---
// 9-bit, ~156kHz, external RC filter + opamp (×4.9 gain)
#define PIN_VDD_PWM             GPIO_NUM_0
#define PIN_VPP_PWM             GPIO_NUM_1
#define PWM_VDD_CHANNEL         0
#define PWM_VPP_CHANNEL         1
#define PWM_RESOLUTION_BITS     9
#define PWM_MAX_DUTY            511          // 2^9 - 1
#define PWM_FREQUENCY_HZ        78000        // ~40MHz / 512 (limited by XTAL clock on ESP32-C3)

// --- ADC Voltage Sense (×4.9 divider from opamp output) ---
#define PIN_ADC_VDD_SENSE       GPIO_NUM_3
#define PIN_ADC_VPP_SENSE       GPIO_NUM_4
#define ADC_VOLTAGE_DIVIDER     4.9f

// --- PDK Programming Interface (bit-banged) ---
#define PIN_IC_CLK              GPIO_NUM_5
#define PIN_IC_PA4              GPIO_NUM_6   // secondary data line
#define PIN_IC_DAT              GPIO_NUM_7   // main data line
#define PIN_IC_CLK2             GPIO_NUM_20  // alternate clock (UART TX to target)
#define PIN_IC_CMT              GPIO_NUM_21  // commit signal (UART RX from target)

// --- Board Control ---
#define PIN_DCDC_ENABLE         GPIO_NUM_10  // DCDC 15V booster enable
#define PIN_STATUS_LED          GPIO_NUM_8   // onboard LED (active low)
#define PIN_USER_BUTTON         GPIO_NUM_9   // onboard button (active low)

// --- UART to Target IC (debug bridge) ---
#define PIN_IC_UART_TX          GPIO_NUM_20  // Serial1 TX → target IC RX
#define PIN_IC_UART_RX          GPIO_NUM_21  // Serial1 RX ← target IC TX
#define TARGET_UART_BAUD        19200

#endif // ESP32_PINS_H
