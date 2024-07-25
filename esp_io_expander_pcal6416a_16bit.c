#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "driver/i2c.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_io_expander.h"
#include "esp_io_expander_pcal6416a_16bit.h"

/* Timeout of each I2C communication */
#define I2C_TIMEOUT_MS          (10)

#define IO_COUNT                (16)

/* Register address */
#define INPUT_REG_ADDR          (0x00)
#define OUTPUT_REG_ADDR         (0x02)
#define DIRECTION_REG_ADDR      (0x06)

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL     (0xffff)
#define OUT_REG_DEFAULT_VAL     (0xffff)

/**
 * @brief Device Structure Type
 *
 */
typedef struct {
    esp_io_expander_t base;
    i2c_port_t i2c_num;
    uint32_t i2c_address;
    struct {
        uint16_t direction;
        uint16_t output;
    } regs;
} esp_io_expander_pcal6416a_16bit_t;

static char *TAG = "pcal6416a_16";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

esp_err_t esp_io_expander_new_i2c_pcal6416a_16bit(i2c_port_t i2c_num, uint32_t i2c_address, esp_io_expander_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(i2c_num < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "Invalid i2c num");
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_io_expander_pcal6416a_16bit_t *pcal = (esp_io_expander_pcal6416a_16bit_t *)calloc(1, sizeof(esp_io_expander_pcal6416a_16bit_t));
    ESP_RETURN_ON_FALSE(pcal, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    pcal->base.config.io_count = IO_COUNT;
    pcal->base.config.flags.dir_out_bit_zero = 1;
    pcal->i2c_num = i2c_num;
    pcal->i2c_address = i2c_address;
    pcal->base.read_input_reg = read_input_reg;
    pcal->base.write_output_reg = write_output_reg;
    pcal->base.read_output_reg = read_output_reg;
    pcal->base.write_direction_reg = write_direction_reg;
    pcal->base.read_direction_reg = read_direction_reg;
    pcal->base.del = del;
    pcal->base.reset = reset;

    esp_err_t ret = ESP_OK;
    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&pcal->base), err, TAG, "Reset failed");

    *handle = &pcal->base;
    return ESP_OK;
err:
    free(pcal);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pcal6416a_16bit_t *pcal = (esp_io_expander_pcal6416a_16bit_t *)__containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    uint8_t temp[2] = {0, 0};
    // *INDENT-OFF*
    ESP_RETURN_ON_ERROR(
        i2c_master_write_read_device(pcal->i2c_num, pcal->i2c_address, (uint8_t[]){INPUT_REG_ADDR}, 1, (uint8_t*)&temp, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Read input reg failed");
    // *INDENT-ON*
    *value = (((uint32_t)temp[1]) << 8) | (temp[0]);
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pcal6416a_16bit_t *pcal = (esp_io_expander_pcal6416a_16bit_t *)__containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {OUTPUT_REG_ADDR, value & 0xff, value >> 8};
    ESP_RETURN_ON_ERROR(
        i2c_master_write_to_device(pcal->i2c_num, pcal->i2c_address, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Write output reg failed");
    pcal->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pcal6416a_16bit_t *pcal = (esp_io_expander_pcal6416a_16bit_t *)__containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    *value = pcal->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pcal6416a_16bit_t *pcal = (esp_io_expander_pcal6416a_16bit_t *)__containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {DIRECTION_REG_ADDR, value & 0xff, value >> 8};
    ESP_RETURN_ON_ERROR(
        i2c_master_write_to_device(pcal->i2c_num, pcal->i2c_address, data, sizeof(data), pdMS_TO_TICKS(I2C_TIMEOUT_MS)),
        TAG, "Write direction reg failed");
    pcal->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pcal6416a_16bit_t *pcal = (esp_io_expander_pcal6416a_16bit_t *)__containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    *value = pcal->regs.direction;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_pcal6416a_16bit_t *pcal = (esp_io_expander_pcal6416a_16bit_t *)__containerof(handle, esp_io_expander_pcal6416a_16bit_t, base);

    free(pcal);
    return ESP_OK;
}
