/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include "drivers/flash/qspi_flash_definitions.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/i2c/definitions.h"
#include "drivers/i2c/nrf5.h"
#include "drivers/mic.h"
#include "drivers/mic/nrf5/pdm_definitions.h"
#include "drivers/pmic/npm1300.h"
#include "drivers/qspi_definitions.h"
#include "drivers/rtc.h"
#include "drivers/uart/nrf5.h"
#include "kernel/util/delay.h"
#include "system/logging.h"

#include <hal/nrf_clock.h>
#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <nrfx_pdm.h>
#include <nrfx_spim.h>
#include <nrfx_twim.h>

// LFXO start timeout before falling back to the internal RC oscillator
#define LFXO_START_TIMEOUT_US (1000U * 1000U)

/* QSPI */

static QSPIPortState s_qspi_port_state;
static QSPIPort QSPI_PORT = {
    .state = &s_qspi_port_state,
    .clk_freq_hz = 24000000UL,
    .cs_gpio = NRF_GPIO_PIN_MAP(0, 18),
    .clk_gpio = NRF_GPIO_PIN_MAP(0, 17),
    .data_gpio =
        {
            NRF_GPIO_PIN_MAP(0, 13),
            NRF_GPIO_PIN_MAP(0, 14),
            NRF_GPIO_PIN_MAP(0, 15),
            NRF_GPIO_PIN_MAP(0, 16),
        },
};
QSPIPort *const QSPI = &QSPI_PORT;

static QSPIFlashState s_qspi_flash_state;
static QSPIFlash QSPI_FLASH_DEVICE = {
    .state = &s_qspi_flash_state,
    .qspi = &QSPI_PORT,
    .read_mode = QSPI_FLASH_READ_READ4IO,
    .write_mode = QSPI_FLASH_WRITE_PP4IO,
};
QSPIFlash *const QSPI_FLASH = &QSPI_FLASH_DEVICE;

/* Debug UART */

static UARTDeviceState s_dbg_uart_state;
static UARTDevice DBG_UART_DEVICE = {
    .state = &s_dbg_uart_state,
    .tx_gpio = NRF_GPIO_PIN_MAP(0, 20),
    .rx_gpio = NRF_GPIO_PIN_MAP(0, 22),
    .rts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .cts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
    .periph = NRFX_UARTE_INSTANCE(0),
    .counter = NRFX_TIMER_INSTANCE(2),
};
UARTDevice *const DBG_UART = &DBG_UART_DEVICE;
IRQ_MAP_NRFX(SERIAL0, nrfx_uarte_0_irq_handler);
IRQ_MAP_NRFX(TIMER2, nrfx_timer_2_irq_handler);

/* EXTI */

IRQ_MAP_NRFX(GPIOTE0, nrfx_gpiote_0_irq_handler);

/* PMIC / ALS I2C bus */

static I2CBusState s_i2c_pmic_bus_state;

static const I2CBusHal s_i2c_pmic_bus_hal = {
    .twim = NRFX_TWIM_INSTANCE(1),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_PMIC_BUS = {
    .state = &s_i2c_pmic_bus_state,
    .hal = &s_i2c_pmic_bus_hal,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 4),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(0, 5),
        },
    .name = "I2C_PMIC",
};
IRQ_MAP_NRFX(SERIAL1, nrfx_twim_1_irq_handler);

static const I2CSlavePort I2C_SLAVE_NPM1300 = {
    .bus = &I2C_PMIC_BUS,
    .address = 0x6B << 1,
};
I2CSlavePort *const I2C_NPM1300 = &I2C_SLAVE_NPM1300;

// OPT3007 is register compatible with the OPT3001
static const I2CSlavePort I2C_SLAVE_OPT3001 = {
    .bus = &I2C_PMIC_BUS,
    .address = 0x45 << 1,
};
I2CSlavePort *const I2C_OPT3001 = &I2C_SLAVE_OPT3001;

/* IMU / touch I2C bus */

static I2CBusState s_i2c_sensor_bus_state;

static const I2CBusHal s_i2c_sensor_bus_hal = {
    .twim = NRFX_TWIM_INSTANCE(2),
    .frequency = NRF_TWIM_FREQ_400K,
};

static const I2CBus I2C_SENSOR_BUS = {
    .state = &s_i2c_sensor_bus_state,
    .hal = &s_i2c_sensor_bus_hal,
    .scl_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(1, 3),
        },
    .sda_gpio =
        {
            .gpio = NRF5_GPIO_RESOURCE_EXISTS,
            .gpio_pin = NRF_GPIO_PIN_MAP(1, 2),
        },
    .name = "I2C_SENSOR",
};
IRQ_MAP_NRFX(SERIAL2, nrfx_twim_2_irq_handler);

static const I2CSlavePort I2C_SLAVE_LSM6DSO = {
    .bus = &I2C_SENSOR_BUS,
    .address = 0x6B << 1,
};
I2CSlavePort *const I2C_LSM6DSO = &I2C_SLAVE_LSM6DSO;

static const I2CSlavePort I2C_SLAVE_AW93203 = {
    .bus = &I2C_SENSOR_BUS,
    .address = 0x12 << 1,
};
I2CSlavePort *const I2C_AW93203 = &I2C_SLAVE_AW93203;

/* Display */

static void prv_display_power_up(void) {
  // Order matters: 1.8V logic, then 1.2V core, then the -2V LED supply
  NPM1300_OPS.ldsw1_set_enabled(true);
  delay_us(1000);
  NPM1300_OPS.buck2_set_enabled(true);
  delay_us(1000);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.neg_supply_en, true);
  delay_us(15000);
}

static void prv_display_power_down(void) {
  delay_us(2000);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.neg_supply_en, false);
  delay_us(16000);
  NPM1300_OPS.buck2_set_enabled(false);
  delay_us(5000);
  NPM1300_OPS.ldsw1_set_enabled(false);
}

const BoardConfigJbdDisplay BOARD_CONFIG_DISPLAY = {
    .spi = NRFX_SPIM_INSTANCE(4),
    .frequency_hz = NRFX_MHZ_TO_HZ(32),

    .clk = {NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 8), true},
    .mosi = {NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 9), true},
    .miso = {NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 10), true},
    .cs = {NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 11), false},

    .neg_supply_en = {NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 24), true},

    .power_up = prv_display_power_up,
    .power_down = prv_display_power_down,

    // Visible area is roughly 640x200 at the top of the 640x480 panel
    .window_x = (640 - PBL_DISPLAY_WIDTH) / 2,
    .window_y = (200 - PBL_DISPLAY_HEIGHT) / 2,

    .offset_row = 12,
    .offset_col = 10,

    .mirror_rl = true,
    .invert = true,
};
IRQ_MAP_NRFX(SPIM4, nrfx_spim_4_irq_handler);

/* Microphone */

static MicDeviceState s_mic_state;
static MicDevice s_mic_device = {
    .state = &s_mic_state,
    .pdm_instance = NRFX_PDM_INSTANCE(0),
    .clk_pin = NRF_GPIO_PIN_MAP(1, 13),
    .data_pin = NRF_GPIO_PIN_MAP(1, 14),
    .channels = 1,
};
MicDevice *const MIC = &s_mic_device;
IRQ_MAP_NRFX(PDM0, NRFX_PDM_INST_HANDLER_GET(0));

/* RTC */

IRQ_MAP_NRFX(RTC1, rtc_irq_handler);

/* PMIC */

const Npm1300Config NPM1300_CONFIG = {
    .chg_current_ma = 40,
    .dischg_limit_ma = 1000,
    .term_current_pct = 10,
    .thermistor_beta = 4250,
    .ntc_hot_celsius = 45,
    .vbus_current_startup = 500,
};

static void prv_lfclk_start(void) {
  // LFXO uses XL1/XL2 (P0.00/P0.01), which must be handed to the oscillator
  nrf_gpio_pin_control_select(NRF_GPIO_PIN_MAP(0, 0), NRF_GPIO_PIN_SEL_PERIPHERAL);
  nrf_gpio_pin_control_select(NRF_GPIO_PIN_MAP(0, 1), NRF_GPIO_PIN_SEL_PERIPHERAL);

  nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL);
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);

  for (uint32_t waited_us = 0U; waited_us < LFXO_START_TIMEOUT_US; waited_us += 100U) {
    if (nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
      nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
      return;
    }
    delay_us(100U);
  }

  // No crystal: fall back to the internal RC oscillator
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
  nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_RC);
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
  while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
  }
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
}

void board_early_init(void) {
  prv_lfclk_start();
}

static void prv_gpio_output_init_level(uint32_t pin, bool level) {
  nrf_gpio_pin_write(pin, level);
  nrf_gpio_cfg_output(pin);
}

void board_init(void) {
  // Idle levels used by the stock firmware: panel -2V off, TMR low,
  // NFC low-power-detect low, PMIC reset released, touch controller powered
  prv_gpio_output_init_level(NRF_GPIO_PIN_MAP(0, 24), false);
  prv_gpio_output_init_level(NRF_GPIO_PIN_MAP(0, 19), false);
  prv_gpio_output_init_level(NRF_GPIO_PIN_MAP(0, 21), false);
  prv_gpio_output_init_level(NRF_GPIO_PIN_MAP(0, 23), false);
  prv_gpio_output_init_level(NRF_GPIO_PIN_MAP(0, 30), true);

  i2c_init(&I2C_PMIC_BUS);
  i2c_init(&I2C_SENSOR_BUS);
}
