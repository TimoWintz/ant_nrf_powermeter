#ifndef STORAGE_H
#define STORAGE_H
#include "nrf_log.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"

#define HEADER_IDENTIFIER 0xB000BA
#define N_PARAMETERS 2
#define FLASH_ADDR 0x70000

typedef int32_t PARAM_T;

typedef union
{
    struct {
      PARAM_T scale_offset;
      PARAM_T scale_scale;
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


static void storage_write(stored_data_t* p_data)
{
  
  data[0] = HEADER_IDENTIFIER;
  for (int i = 0; i < N_PARAMETERS; i++) {
    data[i+1] = p_data->vec[i];
  }
  uint32_t err_code = sd_flash_write	(	(uint32_t *) FLASH_ADDR,
  (uint32_t *) data,
  (N_PARAMETERS+1)
  );
  APP_ERROR_CHECK(err_code);
  uint32_t * 	p_evt_id;
  do {
    sd_app_evt_wait();
  } while (sd_evt_get	(p_evt_id) == NRF_ERROR_NOT_FOUND);

  if (*p_evt_id == NRF_EVT_FLASH_OPERATION_SUCCESS) {
    NRF_LOG_INFO("Successfully wrote data to flash.");
  }
  else {
    NRF_LOG_WARNING("Could not write to flash.");
  }

  memcpy	((void *) data, (void *) FLASH_ADDR,
  (N_PARAMETERS+1) * sizeof(PARAM_T));

}


#endif