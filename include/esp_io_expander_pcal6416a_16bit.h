#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_io_expander.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a new PCAL6416A_16bit IO expander driver
 *
 * @note The I2C communication should be initialized before use this function
 *
 * @param bus_handle: I2C bus handle
 * @param i2c_address: I2C address of chip (\see esp_io_expander_pcal_6416a_16bit_address)
 * @param handle: IO expander handle
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_new_i2c_pcal6416a_16bit(
  i2c_master_bus_handle_t   bus_handle,
  uint32_t                  i2c_address,
  esp_io_expander_handle_t* handle);

/**
 * @brief I2C address of the PCAL6416A
 *
 * The 8-bit address format for the PCAL6416A is as follows:
 *
 *                (Slave Address)
 *     ┌─────────────────┷─────────────────┐
 *  ┌─────┐─────┐─────┐─────┐─────┐─────┐─────┐─────┐
 *  |  0  |  1  |  0  |  0  |  0  |  0  |  A  | R/W |
 *  └─────┘─────┘─────┘─────┘─────┘─────┘─────┘─────┘
 *     └──────────────┯──────────────┘     ┯
 *                 (Fixed)        (Hareware Selectable)
 *
 * And the 7-bit slave address is the most important data for users.
 * For example, if a PCAL6416A chip's A are connected to GND, it's 7-bit slave address is 0b0100000.
 * Then users can use `ESP_IO_EXPANDER_I2C_PCAL6416A_ADDRESS_0` to init it.
 */
enum esp_io_expander_pcal_6416a_16bit_address {
    ESP_IO_EXPANDER_I2C_PCAL6416A_ADDRESS_0 = 0b0100000,
    ESP_IO_EXPANDER_I2C_PCAL6416A_ADDRESS_1 = 0b0100001
};

#ifdef __cplusplus
}
#endif
