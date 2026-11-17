/*
 * flash.c
 *
 *  Created on: 29 May 2020
 *      Author: Ralim
 */

#include "BSP.h"
#include "BSP_Flash.h"
#include "stm32f1xx_hal.h"
#include "string.h"

void flash_save_buffer(const uint8_t *buffer, const uint16_t length) {
  FLASH_EraseInitTypeDef pEraseInit;
  pEraseInit.TypeErase    = FLASH_TYPEERASE_PAGES;
  pEraseInit.Banks        = FLASH_BANK_1;
  pEraseInit.NbPages      = 1;
  pEraseInit.PageAddress  = (uint32_t)SETTINGS_START_PAGE;
  uint32_t failingAddress = 0;
  resetWatchdog();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR | FLASH_FLAG_BSY);
  HAL_FLASH_Unlock();
  HAL_Delay(1);
  resetWatchdog();
  HAL_FLASHEx_Erase(&pEraseInit, &failingAddress);
  //^ Erase the page of flash (1024 bytes on this stm32)
  // erased the chunk
  // now we program it
  uint16_t *data = (uint16_t *)buffer;
  HAL_FLASH_Unlock();
  for (uint16_t i = 0; i < (length / 2); i++) {
    resetWatchdog();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, SETTINGS_START_PAGE + (i * sizeof(uint16_t)), data[i]);
  }
  HAL_FLASH_Lock();
}

void flash_read_buffer(uint8_t *buffer, const uint16_t length) { memcpy(buffer, (uint8_t *)SETTINGS_START_PAGE, length); }

#ifdef MODEL_TS101
// ---------------------------------------------------------------------------
// Temporary diagnostics: a tiny event log kept in a spare flash page so that it
// survives the iron losing power. Each entry is one byte (stored in a halfword).
//   0x80|flags : boot, flags = RCC reset cause (1=pin 2=power-on 4=software 8=watchdog)
//   0x01..0x0F : stages (see POWThread.cpp / BSP.cpp)
//   0x10..0x3F : uptime in 250ms steps
// ---------------------------------------------------------------------------
#define DIAG_LOG_PAGE    (0x08000000 + (120 * 1024))
#define DIAG_LOG_ENTRIES 512
static int16_t diagLogNext    = -1; // next free slot
static int16_t diagLogAtBoot  = 0;  // slot count when this boot started

static void diagLogErase(void) {
  FLASH_EraseInitTypeDef pEraseInit;
  pEraseInit.TypeErase    = FLASH_TYPEERASE_PAGES;
  pEraseInit.Banks        = FLASH_BANK_1;
  pEraseInit.NbPages      = 1;
  pEraseInit.PageAddress  = (uint32_t)DIAG_LOG_PAGE;
  uint32_t failingAddress = 0;
  resetWatchdog();
  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&pEraseInit, &failingAddress);
  HAL_FLASH_Lock();
  resetWatchdog();
}

void diagLog(uint8_t value) {
  const uint16_t *page = (const uint16_t *)DIAG_LOG_PAGE;
  uint32_t        pm   = __get_PRIMASK();
  __disable_irq();
  if (diagLogNext < 0) {
    // First call this boot: find the end of the log
    int16_t i = 0;
    while (i < DIAG_LOG_ENTRIES && page[i] != 0xFFFF) {
      i++;
    }
    if (i > (DIAG_LOG_ENTRIES - 64)) {
      // Nearly full: start again
      diagLogErase();
      i = 0;
    }
    diagLogNext   = i;
    diagLogAtBoot = i;
  }
  if (diagLogNext < DIAG_LOG_ENTRIES) {
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DIAG_LOG_PAGE + (diagLogNext * sizeof(uint16_t)), (uint16_t)value);
    HAL_FLASH_Lock();
    diagLogNext++;
  }
  if (!pm) {
    __enable_irq();
  }
}

// Read an entry from before this boot: back = 1 is the last entry of the previous boot.
// Returns 0xFF when there is nothing there.
uint8_t diagLogReadBack(int16_t back) {
  const uint16_t *page = (const uint16_t *)DIAG_LOG_PAGE;
  int16_t         idx  = diagLogAtBoot - back;
  if (diagLogNext < 0 || idx < 0 || idx >= DIAG_LOG_ENTRIES) {
    return 0xFF;
  }
  return (uint8_t)(page[idx] & 0xFF);
}

// As diagLogReadBack, but skipping the uptime ticks (0x10..0x3F) so the viewer only shows
// events: n = 1 is the most recent event before this boot.
uint8_t diagLogReadBackEvents(int16_t n) {
  const uint16_t *page = (const uint16_t *)DIAG_LOG_PAGE;
  if (diagLogNext < 0 || n < 1) {
    return 0xFF;
  }
  for (int16_t idx = diagLogAtBoot - 1; idx >= 0; idx--) {
    uint8_t v = (uint8_t)(page[idx] & 0xFF);
    if (v >= 0x10 && v <= 0x3F) {
      continue;
    }
    n--;
    if (n == 0) {
      return v;
    }
  }
  return 0xFF;
}

void diagLogBoot(void) {
  // Reset cause flags live in the top bits of RCC->CSR
  uint8_t flags = (uint8_t)((RCC->CSR >> 26) & 0x0F);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  diagLog(0x80 | flags);
  diagLog(0xBA); // marker: this boot was recorded by diagnostic build 10
}
#endif
