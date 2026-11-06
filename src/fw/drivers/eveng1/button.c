/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

// The G1 has no physical buttons. Until the AW93203 touch pad is mapped to
// button events, buttons can only be driven from the serial console.

#include <stdlib.h>

#include "console/prompt.h"
#include "drivers/button.h"
#include "drivers/debounced_button.h"
#include "kernel/events.h"

bool button_is_pressed(ButtonId id) {
  return false;
}

uint8_t button_get_state_bits(void) {
  return 0;
}

void button_init(void) {}

void button_set_rotated(bool rotated) {}

void debounced_button_init(void) {
  button_init();
}

void command_button_read(const char *button_id_str) {
  int button = atoi(button_id_str);

  if (button < 0 || button >= NUM_BUTTONS) {
    prompt_send_response("Invalid button");
    return;
  }

  prompt_send_response(button_is_pressed(button) ? "down" : "up");
}

void command_put_raw_button_event(const char *button_index, const char *is_button_down_event) {
  PebbleEvent e;
  int is_down = atoi(is_button_down_event);
  int button = atoi(button_index);

  if ((button < 0 || button >= NUM_BUTTONS) || (is_down != 1 && is_down != 0)) {
    return;
  }

  e.type = (is_down) ? PEBBLE_BUTTON_DOWN_EVENT : PEBBLE_BUTTON_UP_EVENT;
  e.button.button_id = button;
  event_put(&e);
}
