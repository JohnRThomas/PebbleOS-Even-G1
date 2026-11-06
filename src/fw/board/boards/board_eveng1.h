/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "drivers/pmic/npm1300.h"
#include "util/size.h"

// Even Realities G1 (one temple, nRF5340 application core).
//
// Pin map, from the stock G1 application core firmware:
//   UARTE0 (debug)      TX P0.20, RX P0.22
//   TWIM1               SCL P0.04, SDA P0.05: nPM1300 (0x6B), OPT3007 (0x45)
//   TWIM2               SCL P1.03, SDA P1.02: LSM6DSO (0x6B), AW93203 (0x12)
//   QSPI                SCK P0.17, CSN P0.18, IO0-3 P0.13-P0.16: MX25U25645G
//   SPIM4 (JBD panel)   SCK P0.08, MOSI P0.09, MISO P0.10, CS P0.11
//   PDM0 (mic)          CLK P1.13, DIN P1.14
//   Panel -2V enable    P0.24
//   Touch IRQ           P1.09 (active low), touch power P0.30
//   NFC GPO             P1.10
//   HW ID straps        P0.25-P0.28
//   nPM1300 rails       BUCK2 1.2V + LDSW1 1.8V: panel, LDSW2: IMU and mic

#define BT_VENDOR_ID 0x0EEA
#define BT_VENDOR_NAME "Core Devices LLC"

#define BOARD_RTC_INST NRF_RTC1
#define BOARD_RTC_IRQN RTC1_IRQn

static const BoardConfig BOARD_CONFIG = {
    .ambient_light_dark_threshold = 100,
    .ambient_k_delta_threshold = 30,
    .als_always_on = true,

    .backlight_on_percent = 25,

    .mic_config =
        {
            .gain = 40,
        },
};

// No physical buttons: input comes from the AW93203 touch controller
static const BoardConfigButton BOARD_CONFIG_BUTTON = {};

static const BoardConfigPower BOARD_CONFIG_POWER = {
    // nPM1300 interrupt is not routed to the host (CONFIG_PMIC_NPM1300_IRQ=n)
    .low_power_threshold = 2,
    .battery_capacity_hours = 24,
};

static const BoardConfigAccel BOARD_CONFIG_ACCEL = {
    .default_motion_sensitivity = 55U,  // Medium
};

typedef struct {
  nrfx_spim_t spi;
  uint32_t frequency_hz;

  const OutputConfig clk;
  const OutputConfig mosi;
  const OutputConfig miso;
  const OutputConfig cs;

  //! Panel -2V supply enable
  const OutputConfig neg_supply_en;

  //! Panel rail control
  void (*power_up)(void);
  void (*power_down)(void);

  //! Window of the Pebble framebuffer inside the panel's visible area
  uint16_t window_x;
  uint16_t window_y;

  //! Panel row/column offset (0-31), centers the image in the optics
  uint8_t offset_row;
  uint8_t offset_col;

  //! Mirror the image left/right for the waveguide
  bool mirror_rl;

  //! Light up black pixels instead of white ones
  bool invert;
} BoardConfigJbdDisplay;

extern const BoardConfigJbdDisplay BOARD_CONFIG_DISPLAY;

typedef struct {
  //! AW93203 interrupt (active low)
  const ExtiConfig irq;
  //! AW93203 supply enable
  const OutputConfig power_en;
} BoardConfigTouch;

static const BoardConfigTouch BOARD_CONFIG_TOUCH = {
    .irq = {NRFX_GPIOTE_INSTANCE(0), 0, NRF_GPIO_PIN_MAP(1, 9)},
    .power_en = {NRF5_GPIO_RESOURCE_EXISTS, NRF_GPIO_PIN_MAP(0, 30), true},
};

extern UARTDevice *const DBG_UART;

extern QSPIPort *const QSPI;
extern QSPIFlash *const QSPI_FLASH;

extern MicDevice *const MIC;

extern I2CSlavePort *const I2C_NPM1300;
extern I2CSlavePort *const I2C_OPT3001;
extern I2CSlavePort *const I2C_AW93203;
extern I2CSlavePort *const I2C_LSM6DSO;

extern const Npm1300Config NPM1300_CONFIG;
