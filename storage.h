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

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
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
    .evt_handler    = fstorage_evt_handler,
    .start_addr     = 0x70000,
    .end_addr       = 0x80000,
};

void wait_for_flash_ready()
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(&storage_instance))
    {

    }
}


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

static PARAM_T data[N_PARAMETERS+1];

static void storage_read(stored_data_t* p_data)
{
  ret_code_t rc = nrf_fstorage_read(
      &storage_instance,   /* The instance to use. */
      FLASH_ADDR,     /* The address in flash where to read data from. */
      data,        /* A buffer to copy the data into. */
      (N_PARAMETERS+1)*sizeof(PARAM_T)  /* Lenght of the data, in bytes. */
  );
  if (rc == NRF_SUCCESS)
  {
      if (data[0] != HEADER_IDENTIFIER) {
      NRF_LOG_INFO("No data in memory");
        return;
      }
  }
  APP_ERROR_CHECK(rc);
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
  ret_code_t rc = nrf_fstorage_write(
      &storage_instance,   /* The instance to use. */
      FLASH_ADDR,     /* The address in flash where to read data from. */
      data,        /* A buffer to copy the data into. */
      (N_PARAMETERS+1)*sizeof(PARAM_T),  /* Lenght of the data, in bytes. */
      NULL
  );
    wait_for_flash_ready();
  APP_ERROR_CHECK(rc);
}


#endif