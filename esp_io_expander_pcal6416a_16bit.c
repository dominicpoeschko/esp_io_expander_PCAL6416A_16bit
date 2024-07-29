#include "esp_io_expander.h"

#include "driver/i2c_master.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_io_expander_pcal6416a_16bit.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

/* Timeout of each I2C communication */
#define I2C_TIMEOUT_MS (1000)

#define IO_COUNT (16)

/* Register address */
#define INPUT_REG_ADDR     (0x00)
#define OUTPUT_REG_ADDR    (0x02)
#define DIRECTION_REG_ADDR (0x06)

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL (0xffff)
#define OUT_REG_DEFAULT_VAL (0xffff)

/**
 * @brief Device Structure Type
 *
 */
typedef struct {
    esp_io_expander_t       base;
    i2c_master_dev_handle_t dev_handle;
    struct {
        uint16_t direction;
        uint16_t output;
    } regs;
} esp_io_expander_pcal6416a_16bit_t;

static char* TAG = "pcal6416a_16";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t* value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t* value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t* value);
static esp_err_t reset(esp_io_expander_t* handle);
static esp_err_t del(esp_io_expander_t* handle);

esp_err_t esp_io_expander_new_i2c_pcal6416a_16bit(
  i2c_master_bus_handle_t   bus_handle,
  uint32_t                  i2c_address,
  esp_io_expander_handle_t* handle) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_io_expander_pcal6416a_16bit_t* pcal
      = (esp_io_expander_pcal6416a_16bit_t*)calloc(1, sizeof(esp_io_expander_pcal6416a_16bit_t));
    ESP_RETURN_ON_FALSE(pcal, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    pcal->base.config.io_count               = IO_COUNT;
    pcal->base.config.flags.dir_out_bit_zero = 1;
    pcal->base.read_input_reg                = read_input_reg;
    pcal->base.write_output_reg              = write_output_reg;
    pcal->base.read_output_reg               = read_output_reg;
    pcal->base.write_direction_reg           = write_direction_reg;
    pcal->base.read_direction_reg            = read_direction_reg;
    pcal->base.del                           = del;
    pcal->base.reset                         = reset;

    i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address  = i2c_address,
      .scl_speed_hz    = 100000,
    };

    esp_err_t ret = ESP_OK;

    ESP_GOTO_ON_ERROR(
      i2c_master_bus_add_device(bus_handle, &dev_cfg, &pcal->dev_handle),
      err,
      TAG,
      "Bus add device failed");

    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&pcal->base), err2, TAG, "Reset failed");

    *handle = &pcal->base;
    return ESP_OK;
err2:
    i2c_master_bus_rm_device(pcal->dev_handle);
err:
    free(pcal);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t* value) {
    esp_io_expander_pcal6416a_16bit_t* pcal = (esp_io_expander_pcal6416a_16bit_t*)
      __containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    uint8_t temp[2] = {0, 0};
    // *INDENT-OFF*
    ESP_RETURN_ON_ERROR(
      i2c_master_transmit_receive(
        pcal->dev_handle,
        (uint8_t[]){INPUT_REG_ADDR},
        1,
        (uint8_t*)&temp,
        2,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
      TAG,
      "Read input reg failed");
    // *INDENT-ON*
    *value = (((uint32_t)temp[1]) << 8) | (temp[0]);
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value) {
    esp_io_expander_pcal6416a_16bit_t* pcal = (esp_io_expander_pcal6416a_16bit_t*)
      __containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {OUTPUT_REG_ADDR, value & 0xff, value >> 8};
    ESP_RETURN_ON_ERROR(
      i2c_master_transmit(pcal->dev_handle, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
      TAG,
      "Write output reg failed");
    pcal->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t* value) {
    esp_io_expander_pcal6416a_16bit_t* pcal = (esp_io_expander_pcal6416a_16bit_t*)
      __containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    *value = pcal->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value) {
    esp_io_expander_pcal6416a_16bit_t* pcal = (esp_io_expander_pcal6416a_16bit_t*)
      __containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {DIRECTION_REG_ADDR, value & 0xff, value >> 8};
    ESP_RETURN_ON_ERROR(
      i2c_master_transmit(pcal->dev_handle, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
      TAG,
      "Write direction reg failed");
    pcal->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t* value) {
    esp_io_expander_pcal6416a_16bit_t* pcal = (esp_io_expander_pcal6416a_16bit_t*)
      __containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    *value = pcal->regs.direction;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t* handle) {
    ESP_RETURN_ON_ERROR(
      write_direction_reg(handle, DIR_REG_DEFAULT_VAL),
      TAG,
      "Write dir reg failed");
    ESP_RETURN_ON_ERROR(
      write_output_reg(handle, OUT_REG_DEFAULT_VAL),
      TAG,
      "Write output reg failed");
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t* handle) {
    esp_io_expander_pcal6416a_16bit_t* pcal = (esp_io_expander_pcal6416a_16bit_t*)
      __containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    i2c_master_bus_rm_device(pcal->dev_handle);
    free(pcal);
    return ESP_OK;
}
