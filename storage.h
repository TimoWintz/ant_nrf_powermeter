#ifndef STORAGE_H
#define STORAGE_H
#include "nrf_log.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"

#define HEADER_IDENTIFIER 0xB000BA
#define N_PARAMETERS 3
#define FLASH_ADDR 0x70000

typedef int32_t PARAM_T;

typedef union
{
    struct {
      PARAM_T scale_offset;
      PARAM_T scale_scale;
      PARAM_T gyro_offset;
    } params;
    PARAM_T vec[N_PARAMETERS];
} stored_data_t;

static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

static PARAM_T data[N_PARAMETERS+1];

static bool flash_busy = false;
static bool flash_write_success = true;

static void storage_read(stored_data_t* p_data)
{
  memcpy	((void *) data, (void *) FLASH_ADDR,
  (N_PARAMETERS+1) * sizeof(PARAM_T));

  if (data[0] != HEADER_IDENTIFIER) {
  NRF_LOG_INFO("No data in memory");
    return;
  }

  for (int i = 0; i < N_PARAMETERS; i++) {
    p_data->vec[i] = data[i+1];
      NRF_LOG_INFO("Read parameter : %i", data[i+1]);
  }
}


static bool storage_write(stored_data_t* p_data)
{
  flash_busy = true;
  sd_flash_page_erase(FLASH_ADDR / NRF_FICR->CODEPAGESIZE);
  while (flash_busy) { }
  
  data[0] = HEADER_IDENTIFIER;
  for (int i = 0; i < N_PARAMETERS; i++) {
    data[i+1] = p_data->vec[i];
  }
  flash_busy = true;
  uint32_t err_code = sd_flash_write	(	(uint32_t *) FLASH_ADDR,
  (uint32_t *) data,
  (N_PARAMETERS+1)
  );
  (int) 0;

  if (err_code > 0) {
    return false;
  }

  uint32_t * 	p_evt_id;
  while (flash_busy) { }
  if (!flash_write_success) {
    return false;
  }

  memcpy	((void *) data, (void *) FLASH_ADDR,
  (N_PARAMETERS+1) * sizeof(PARAM_T));

  // Verify
  for (int i = 0; i < N_PARAMETERS; i++) {
    if (data[i+1] != p_data->vec[i]) { 
      return false;
    }
  }
}

#endif