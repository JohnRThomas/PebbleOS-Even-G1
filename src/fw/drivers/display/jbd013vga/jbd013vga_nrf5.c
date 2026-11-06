/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// Jade Bird Display JBD013VGA 640x480 4bpp micro-LED panel.
//
// The panel keeps a frame cache that is written over SPI and latched with a
// SYNC command. The 1bpp Pebble framebuffer is expanded to 4bpp and written
// into a window of the cache.

#include "drivers/display/display.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board/board.h"
#include "drivers/gpio.h"
#include "kernel/events.h"
#include "kernel/util/delay.h"
#include "system/logging.h"
#include "system/passert.h"
#include "util/math.h"

#include <nrfx_spim.h>

#include "FreeRTOS.h"
#include "semphr.h"

#define JBD_CMD_WR_STATUS_REG1 0x01U
#define JBD_CMD_WR_CACHE 0x02U
#define JBD_CMD_WR_ENABLE 0x06U
#define JBD_CMD_WR_STATUS_REG2 0x31U
#define JBD_CMD_WR_LUM_REG 0x36U
#define JBD_CMD_WR_CURRENT_REG 0x46U
#define JBD_CMD_RST_EN 0x66U
#define JBD_CMD_DISPLAY_DEFAULT_MODE 0x71U
#define JBD_CMD_DISPLAY_UD 0x72U
#define JBD_CMD_DISPLAY_RL 0x73U
#define JBD_CMD_SYNC 0x97U
#define JBD_CMD_RST 0x99U
#define JBD_CMD_RD_ID 0x9FU
#define JBD_CMD_DISPLAY_ENABLE 0xA3U
#define JBD_CMD_DISPLAY_DISABLE 0xA9U
#define JBD_CMD_WR_OFFSET_REG 0xC0U

#define JBD_PANEL_ID 0x4010U

#define JBD_PANEL_WIDTH 640U
#define JBD_PANEL_HEIGHT 480U

// Status register 1: demura off
#define JBD_STATUS_REG1_VAL 0x10U
// Status register 2 value used by the stock firmware
#define JBD_STATUS_REG2_VAL 0x04U

// Default drive: luminance (PWM) and current registers
#define JBD_LUM_DEFAULT 1000U
#define JBD_CUR_DEFAULT 15U

// Cache write: command, 3 address bytes, dummy byte
#define JBD_WR_CACHE_HDR_LEN 5U
// 2 pixels per byte; one trailing dummy byte like the reference code
#define JBD_ROW_DATA_LEN (PBL_DISPLAY_WIDTH / 2U)
#define JBD_ROW_LEN (JBD_WR_CACHE_HDR_LEN + JBD_ROW_DATA_LEN + 1U)

// Max bytes per SPIM transfer
#define JBD_SPIM_MAX_XFER (SPIM_TXD_MAXCNT_MAXCNT_Msk >> SPIM_TXD_MAXCNT_MAXCNT_Pos)

_Static_assert(PBL_DISPLAY_WIDTH % 8 == 0, "Width must be a multiple of 8");

typedef struct {
  uint16_t num_rows;
  uint16_t next_row;
  bool sync_sent;
} UpdateState;

static uint8_t s_rows[PBL_DISPLAY_HEIGHT][JBD_ROW_LEN];
static uint32_t s_expand_lut[256];
static UpdateState s_update;
static UpdateCompleteCallback s_uccb;
static bool s_updating;
static bool s_rotated;
static SemaphoreHandle_t s_sem;

static void prv_build_lut(bool invert) {
  // 8 1bpp pixels (LSB = leftmost) -> 4 bytes of 4bpp pixels (high nibble = leftmost)
  for (uint32_t i = 0U; i < 256U; i++) {
    uint8_t px = invert ? (uint8_t)~i : (uint8_t)i;
    uint8_t out[4];

    for (uint32_t j = 0U; j < 4U; j++) {
      out[j] = ((px & (1U << (2U * j))) ? 0xF0U : 0U) | ((px & (1U << (2U * j + 1U))) ? 0x0FU : 0U);
    }

    memcpy(&s_expand_lut[i], out, sizeof(out));
  }
}

static uint32_t prv_cache_addr(uint16_t x, uint16_t y) {
  return (((uint32_t)y << 10U) & 0x7FC00U) | (x & 0x3FFU);
}

static void prv_fill_row_header(uint8_t *row, uint16_t y) {
  uint32_t addr = prv_cache_addr(BOARD_CONFIG_DISPLAY.window_x, BOARD_CONFIG_DISPLAY.window_y + y);

  row[0] = JBD_CMD_WR_CACHE;
  row[1] = (uint8_t)(addr >> 16U);
  row[2] = (uint8_t)(addr >> 8U);
  row[3] = (uint8_t)addr;
  row[4] = 0xFFU;
  row[JBD_ROW_LEN - 1U] = 0x00U;
}

// Blocking transfers (init / control path)
///////////////////////////////////////////////////////////////////////////////

static void prv_xfer_blocking(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len) {
  nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TRX(tx, tx_len, rx, rx_len);

  PBL_ASSERTN(!s_updating);

  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);
  xSemaphoreTake(s_sem, portMAX_DELAY);
}

static void prv_send_cmd(uint8_t cmd) {
  prv_xfer_blocking(&cmd, 1U, NULL, 0U);
}

static void prv_send_frame(uint8_t cmd, const uint8_t *param, size_t len) {
  uint8_t buf[4];

  PBL_ASSERTN(len < sizeof(buf));

  buf[0] = cmd;
  memcpy(&buf[1], param, len);
  prv_xfer_blocking(buf, len + 1U, NULL, 0U);
}

static void prv_sync(void) {
  prv_send_cmd(JBD_CMD_SYNC);
  delay_us(1000U);
}

static uint16_t prv_read_id(void) {
  uint8_t tx[4] = {JBD_CMD_RD_ID};
  uint8_t rx[4] = {0};

  prv_xfer_blocking(tx, sizeof(tx), rx, sizeof(rx));

  return ((uint16_t)rx[2] << 8U) | rx[3];
}

static void prv_write_offset(uint8_t row, uint8_t col) {
  uint8_t param[2] = {row, col};

  prv_send_frame(JBD_CMD_WR_OFFSET_REG, param, sizeof(param));
  prv_sync();
}

static void prv_write_lum_cur(uint16_t lum, uint8_t cur) {
  uint8_t lum_be[2] = {(uint8_t)(lum >> 8U), (uint8_t)lum};

  prv_send_cmd(JBD_CMD_WR_ENABLE);
  prv_send_cmd(JBD_CMD_DISPLAY_DISABLE);
  prv_send_frame(JBD_CMD_WR_LUM_REG, lum_be, sizeof(lum_be));
  prv_send_frame(JBD_CMD_WR_CURRENT_REG, &cur, 1U);
  prv_send_frame(JBD_CMD_WR_STATUS_REG2, &(uint8_t){JBD_STATUS_REG2_VAL}, 1U);
  prv_send_cmd(JBD_CMD_DISPLAY_ENABLE);
  prv_sync();
}

static void prv_set_mirror(bool rotated) {
  // The waveguide needs a left/right mirror; 180 degree rotation flips both axes
  bool rl = BOARD_CONFIG_DISPLAY.mirror_rl ^ rotated;

  prv_send_cmd(JBD_CMD_DISPLAY_DEFAULT_MODE);
  if (rl) {
    prv_send_cmd(JBD_CMD_DISPLAY_RL);
  }
  if (rotated) {
    prv_send_cmd(JBD_CMD_DISPLAY_UD);
  }
  prv_sync();
}

static void prv_clear_cache(void) {
  // Reuse the row buffer: write zeros over the whole panel cache, as many
  // full panel rows per transfer as fit (the cache address wraps to the next
  // row).
  uint8_t *buf = &s_rows[0][0];
  const size_t row_bytes = JBD_PANEL_WIDTH / 2U;
  const size_t max_len = MIN(sizeof(s_rows), JBD_SPIM_MAX_XFER) - JBD_WR_CACHE_HDR_LEN;
  const uint16_t rows_per_xfer = max_len / row_bytes;

  memset(buf, 0, JBD_WR_CACHE_HDR_LEN + rows_per_xfer * row_bytes);
  for (uint16_t y = 0U; y < JBD_PANEL_HEIGHT; y += rows_per_xfer) {
    uint32_t addr = prv_cache_addr(0U, y);
    size_t len = MIN(rows_per_xfer, JBD_PANEL_HEIGHT - y) * row_bytes;

    buf[0] = JBD_CMD_WR_CACHE;
    buf[1] = (uint8_t)(addr >> 16U);
    buf[2] = (uint8_t)(addr >> 8U);
    buf[3] = (uint8_t)addr;
    buf[4] = 0xFFU;
    prv_xfer_blocking(buf, JBD_WR_CACHE_HDR_LEN + len, NULL, 0U);
  }

  prv_sync();
}

static void prv_panel_init(void) {
  delay_us(1000U);
  prv_send_cmd(JBD_CMD_RST_EN);
  delay_us(1000U);
  prv_send_cmd(JBD_CMD_RST);
  delay_us(10000U);

  prv_send_cmd(JBD_CMD_WR_ENABLE);
  prv_send_frame(JBD_CMD_WR_STATUS_REG1, &(uint8_t){JBD_STATUS_REG1_VAL}, 1U);

  prv_clear_cache();

  prv_write_offset(BOARD_CONFIG_DISPLAY.offset_row, BOARD_CONFIG_DISPLAY.offset_col);
  prv_set_mirror(s_rotated);
  prv_write_lum_cur(JBD_LUM_DEFAULT, JBD_CUR_DEFAULT);
}

// Asynchronous frame update
///////////////////////////////////////////////////////////////////////////////

static void prv_update_complete(void *data) {
  s_updating = false;
  s_uccb();
}

static bool prv_start_next_xfer(void) {
  nrfx_spim_xfer_desc_t desc;
  // Not const: EasyDMA buffers must be in RAM
  static uint8_t s_sync = JBD_CMD_SYNC;

  if (s_update.next_row < s_update.num_rows) {
    desc = (nrfx_spim_xfer_desc_t)NRFX_SPIM_XFER_TX(s_rows[s_update.next_row], JBD_ROW_LEN);
    s_update.next_row++;
  } else if (!s_update.sync_sent) {
    desc = (nrfx_spim_xfer_desc_t)NRFX_SPIM_XFER_TX(&s_sync, 1U);
    s_update.sync_sent = true;
  } else {
    return false;
  }

  nrfx_err_t err = nrfx_spim_xfer(&BOARD_CONFIG_DISPLAY.spi, &desc, 0);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  return true;
}

static void prv_spim_evt_handler(nrfx_spim_evt_t const *evt, void *ctx) {
  portBASE_TYPE woken = pdFALSE;

  if (!s_updating) {
    xSemaphoreGiveFromISR(s_sem, &woken);
  } else if (!prv_start_next_xfer()) {
    PebbleEvent e = {
        .type = PEBBLE_CALLBACK_EVENT,
        .callback =
            {
                .callback = prv_update_complete,
            },
    };

    woken = event_put_isr(&e) ? pdTRUE : pdFALSE;
  }

  portEND_SWITCHING_ISR(woken);
}

// Display API
///////////////////////////////////////////////////////////////////////////////

void display_init(void) {
  nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(
      BOARD_CONFIG_DISPLAY.clk.gpio_pin, BOARD_CONFIG_DISPLAY.mosi.gpio_pin,
      BOARD_CONFIG_DISPLAY.miso.gpio_pin, BOARD_CONFIG_DISPLAY.cs.gpio_pin);
  config.frequency = BOARD_CONFIG_DISPLAY.frequency_hz;
  config.ss_active_high = BOARD_CONFIG_DISPLAY.cs.active_high;

  s_sem = xSemaphoreCreateBinary();
  PBL_ASSERTN(s_sem != NULL);

  prv_build_lut(BOARD_CONFIG_DISPLAY.invert);

  gpio_output_init(&BOARD_CONFIG_DISPLAY.neg_supply_en, GPIO_OType_PP);
  gpio_output_set(&BOARD_CONFIG_DISPLAY.neg_supply_en, false);
  BOARD_CONFIG_DISPLAY.power_up();

  nrfx_err_t err = nrfx_spim_init(&BOARD_CONFIG_DISPLAY.spi, &config, prv_spim_evt_handler, NULL);
  PBL_ASSERTN(err == NRFX_SUCCESS);

  prv_panel_init();

  uint16_t id = prv_read_id();
  if (id != JBD_PANEL_ID) {
    PBL_LOG_ERR("Unexpected JBD panel ID: 0x%04x", id);
  }
}

void display_clear(void) {
  uint8_t *row = s_rows[0];

  for (uint16_t y = 0U; y < PBL_DISPLAY_HEIGHT; y++) {
    prv_fill_row_header(row, y);
    memset(&row[JBD_WR_CACHE_HDR_LEN], 0, JBD_ROW_DATA_LEN);
    prv_xfer_blocking(row, JBD_ROW_LEN, NULL, 0U);
  }
  prv_fill_row_header(row, 0U);

  prv_sync();
}

void display_set_enabled(bool enabled) {
  prv_send_cmd(enabled ? JBD_CMD_DISPLAY_ENABLE : JBD_CMD_DISPLAY_DISABLE);
  prv_sync();
}

void display_set_rotated(bool rotated) {
  s_rotated = rotated;
  prv_set_mirror(rotated);
}

void display_update(NextRowCallback nrcb, UpdateCompleteCallback uccb) {
  DisplayRow row;

  PBL_ASSERTN(!s_updating);

  // Rows come in increasing order; pack the dirty ones at the start of the
  // buffer, each with its own cache address.
  s_update = (UpdateState){0};
  while (nrcb(&row)) {
    uint8_t *dst = s_rows[s_update.num_rows];
    uint8_t *px = &dst[JBD_WR_CACHE_HDR_LEN];

    prv_fill_row_header(dst, row.address);
    for (uint16_t i = 0U; i < PBL_DISPLAY_WIDTH / 8U; i++) {
      memcpy(&px[i * 4U], &s_expand_lut[row.data[i]], sizeof(uint32_t));
    }

    s_update.num_rows++;
  }

  s_uccb = uccb;
  s_updating = true;

  prv_start_next_xfer();
}

bool display_update_in_progress(void) {
  return s_updating;
}

void display_update_boot_frame(uint8_t *framebuffer) {
  // 8bpp ARGB2222 frame: map to 4bpp grey levels
  for (uint16_t y = 0U; y < PBL_DISPLAY_HEIGHT; y++) {
    const uint8_t *src = &framebuffer[y * PBL_DISPLAY_WIDTH];
    uint8_t *dst = s_rows[y];

    prv_fill_row_header(dst, y);
    for (uint16_t x = 0U; x < PBL_DISPLAY_WIDTH; x += 2U) {
      uint8_t lvl[2];

      for (uint8_t i = 0U; i < 2U; i++) {
        uint8_t p = src[x + i];
        uint8_t grey = (uint8_t)(((((p >> 4U) & 3U) + ((p >> 2U) & 3U) + (p & 3U)) * 15U) / 9U);
        lvl[i] = BOARD_CONFIG_DISPLAY.invert ? (15U - grey) : grey;
      }

      dst[JBD_WR_CACHE_HDR_LEN + x / 2U] = (uint8_t)((lvl[0] << 4U) | lvl[1]);
    }

    prv_xfer_blocking(dst, JBD_ROW_LEN, NULL, 0U);
  }

  prv_sync();
}
