#ifndef STORAGE_H
#define STORAGE_H
#include "nrf_log.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

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

static void callback(nrf_fstorage_evt_t * p_evt) {
}

static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

NRF_FSTORAGE_DEF(nrf_fstorage_t storage_instance) =
{
    .evt_handler    = callback,
    .start_addr     = 0x70000,
    .end_addr       = 0x80000,
};


static void storage_init()
{
    ret_code_t rc = nrf_fstorage_init(
        &storage_instance,       /* You fstorage instance, previously defined. */
        &nrf_fstorage_sd,   /* Name of the backend. */
        NULL                /* Optional parameter, backend-dependant. */
    );
    APP_ERROR_CHECK(rc);
    while (nrf_fstorage_is_busy(&storage_instance))
    {

    }

}

static void storage_read(stored_data_t* p_data)
{
  static uint32_t number;
  ret_code_t rc = nrf_fstorage_read(
      &storage_instance,   /* The instance to use. */
      FLASH_ADDR,     /* The address in flash where to read data from. */
      &number,        /* A buffer to copy the data into. */
      sizeof(number)  /* Lenght of the data, in bytes. */
  );
  if (rc == NRF_SUCCESS)
  {
      if (number != HEADER_IDENTIFIER) {
      NRF_LOG_INFO("No data in memory");
        return;
      }
  }
  else
  {
      APP_ERROR_CHECK(rc);
  }
  size_t offset = sizeof(number);
  for (int i = 0; i < N_PARAMETERS; i++) {
    rc = nrf_fstorage_read(
        &storage_instance,   /* The instance to use. */
        FLASH_ADDR + offset,     /* The address in flash where to read data from. */
        &(p_data->vec[i]),        /* A buffer to copy the data into. */
        sizeof(number)  /* Lenght of the data, in bytes. */
    );
    if (rc == NRF_SUCCESS)
    {
      PARAM_T x = p_data->vec[i];
      NRF_LOG_INFO("Read parameter : %i", x);
    }
    else
    {
      APP_ERROR_CHECK(rc);
    }
    offset += sizeof(PARAM_T);
  }
}

#endif