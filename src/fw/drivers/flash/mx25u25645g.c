/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "board/board.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/qspi_flash.h"
#include "drivers/flash/qspi_flash_part_definitions.h"
#include "flash_region/flash_region.h"
#include "system/passert.h"
#include "system/status_codes.h"
#include "util/math.h"
#include "util/size.h"

static bool s_protected;
static FlashAddress s_protected_start;
static FlashAddress s_protected_end;

// Macronix MX25U25645G (1.8V, 32MB). The status register only has SR1; the
// erase suspend flag (ESB) lives in the security register, so RDSCUR is used
// as the "SR2" read. The secured OTP area uses ENSO/EXSO mode instead of
// security registers, so none are exposed.
static QSPIFlashPart QSPI_FLASH_PART = {
    .instructions =
        {
            .fast_read = 0x0B,
            .read2o = 0x3B,
            .read2io = 0xBB,
            .read4o = 0x6B,
            .read4io = 0xEB,
            .pp = 0x02,
            .pp4io = 0x38,
            .erase_sector_4k = 0x20,
            .erase_block_64k = 0xD8,
            .write_enable = 0x06,
            .write_disable = 0x04,
            .rdsr1 = 0x05,
            .rdsr2 = 0x2B, /* RDSCUR */
            .wrsr = 0x01,
            .erase_suspend = 0xB0,
            .erase_resume = 0x30,
            .enter_low_power = 0xB9,
            .exit_low_power = 0xAB,
            .reset_enable = 0x66,
            .reset = 0x99,
            .qspi_id = 0x9F,
            .en4b = 0xB7,
        },
    .status_bit_masks =
        {
            .busy = 1 << 0,
            .write_enable = 1 << 1,
        },
    .flag_status_bit_masks =
        {
            .erase_suspend = 1 << 3, /* security register ESB */
        },
    .dummy_cycles =
        {
            .fast_read = 8,
        },
    .supports_block_lock = false,
    .reset_latency_ms = 12,
    .suspend_to_read_latency_us = 20,
    .standby_to_low_power_latency_us = 10,
    .low_power_to_standby_latency_us = 35,
    .supports_fast_read_ddr = false,
    .qer_type = JESD216_DW15_QER_S1B6,
    .qspi_id_value = 0x3925c2,
    .size = 0x2000000, /* 32MB */
    .name = "MX25U25645G",
};

static status_t prv_flash_check_protected(FlashAddress addr) {
  if (!s_protected) {
    return S_SUCCESS;
  }

  if (addr < s_protected_start || addr > s_protected_end) {
    return S_SUCCESS;
  }

  return E_INVALID_OPERATION;
}

FlashAddress flash_impl_get_sector_base_address(FlashAddress addr) {
  return (addr & SECTOR_ADDR_MASK);
}

FlashAddress flash_impl_get_subsector_base_address(FlashAddress addr) {
  return (addr & SUBSECTOR_ADDR_MASK);
}

void flash_impl_enable_write_protection(void) {}

status_t flash_impl_write_protect(FlashAddress start_sector, FlashAddress end_sector) {
  if (s_protected) {
    return E_ERROR;
  }

  s_protected_start = start_sector;
  s_protected_end = end_sector;
  s_protected = true;

  return S_SUCCESS;
}

status_t flash_impl_unprotect(void) {
  if (!s_protected) {
    return E_ERROR;
  }

  s_protected = false;

  return S_SUCCESS;
}

status_t flash_impl_init(bool coredump_mode) {
  qspi_flash_init(QSPI_FLASH, &QSPI_FLASH_PART, coredump_mode);
  return S_SUCCESS;
}

status_t flash_impl_get_erase_status(void) { return qspi_flash_is_erase_complete(QSPI_FLASH); }

status_t flash_impl_erase_subsector_begin(FlashAddress subsector_addr) {
  status_t status;

  status = prv_flash_check_protected(subsector_addr);
  if (FAILED(status)) {
    return status;
  }

  return qspi_flash_erase_begin(QSPI_FLASH, subsector_addr, true /* is_subsector */);
}
status_t flash_impl_erase_sector_begin(FlashAddress sector_addr) {
  status_t status;

  status = prv_flash_check_protected(sector_addr);
  if (FAILED(status)) {
    return status;
  }

  return qspi_flash_erase_begin(QSPI_FLASH, sector_addr, false /* !is_subsector */);
}

status_t flash_impl_erase_suspend(FlashAddress sector_addr) {
  return qspi_flash_erase_suspend(QSPI_FLASH, sector_addr);
}

status_t flash_impl_erase_resume(FlashAddress sector_addr) {
  qspi_flash_erase_resume(QSPI_FLASH, sector_addr);
  return S_SUCCESS;
}

status_t flash_impl_read_sync(void *buffer_ptr, FlashAddress start_addr, size_t buffer_size) {
  PBL_ASSERT(buffer_size > 0, "flash_impl_read_sync() called with 0 bytes to read");
  qspi_flash_read_blocking(QSPI_FLASH, start_addr, buffer_ptr, buffer_size);
  return S_SUCCESS;
}

int flash_impl_write_page_begin(const void *buffer, const FlashAddress start_addr, size_t len) {
  status_t status;

  status = prv_flash_check_protected(start_addr);
  if (FAILED(status)) {
    return status;
  }

  return qspi_flash_write_page_begin(QSPI_FLASH, buffer, start_addr, len);
}

status_t flash_impl_get_write_status(void) { return qspi_flash_get_write_status(QSPI_FLASH); }

status_t flash_impl_enter_low_power_mode(void) {
  qspi_flash_set_lower_power_mode(QSPI_FLASH, true);
  return S_SUCCESS;
}
status_t flash_impl_exit_low_power_mode(void) {
  qspi_flash_set_lower_power_mode(QSPI_FLASH, false);
  return S_SUCCESS;
}

status_t flash_impl_set_burst_mode(bool burst_mode) {
  // NYI
  return S_SUCCESS;
}

status_t flash_impl_blank_check_sector(FlashAddress addr) {
  return qspi_flash_blank_check(QSPI_FLASH, addr, false /* !is_subsector */);
}
status_t flash_impl_blank_check_subsector(FlashAddress addr) {
  return qspi_flash_blank_check(QSPI_FLASH, addr, true /* is_subsector */);
}

uint32_t flash_impl_get_typical_sector_erase_duration_ms(void) { return 150; }

uint32_t flash_impl_get_typical_subsector_erase_duration_ms(void) { return 50; }

status_t flash_impl_read_security_register(uint32_t addr, uint8_t *val) {
  return qspi_flash_read_security_register(QSPI_FLASH, addr, val);
}

status_t flash_impl_security_register_is_locked(uint32_t address, bool *locked) {
  return qspi_flash_security_register_is_locked(QSPI_FLASH, address, locked);
}

status_t flash_impl_erase_security_register(uint32_t addr) {
  return qspi_flash_erase_security_register(QSPI_FLASH, addr);
}

status_t flash_impl_write_security_register(uint32_t addr, uint8_t val) {
  return qspi_flash_write_security_register(QSPI_FLASH, addr, val);
}

const FlashSecurityRegisters *flash_impl_security_registers_info(void) {
  return qspi_flash_security_registers_info(QSPI_FLASH);
}

#ifdef CONFIG_RECOVERY_FW
status_t flash_impl_lock_security_register(uint32_t address) {
  return qspi_flash_lock_security_register(QSPI_FLASH, address);
}
#endif