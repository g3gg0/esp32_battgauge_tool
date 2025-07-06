#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_console.h"

#include "i2c.h"
#include "gpio_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX_I2C_WRITE_BYTES 256

static const char *TAG = "i2c_cmd"; /* Added for ESP_LOG */

static struct
{
    struct arg_int *start;
    struct arg_int *end;
    struct arg_end *end_arg;
} i2cscan_args;

static struct
{
    struct arg_int *address;
    struct arg_int *nbytes;
    struct arg_end *end_arg;
} i2c_r_args;

static struct
{
    struct arg_int *address;
    struct arg_int *data; /* Array of data bytes */
    struct arg_end *end_arg;
} i2c_w_args;

static struct
{
    struct arg_int *address;
    struct arg_str *wdata;        /* Array of data bytes to write */
    struct arg_int *rbytes;       /* Number of bytes to read */
    struct arg_int *cyclic_ms;    /* Optional: cyclic delay in ms */
    struct arg_int *cyclic_count; /* Optional: number of repetitions */
    struct arg_end *end_arg;
} i2c_rw_args;

static int do_i2cscan(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&i2cscan_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, i2cscan_args.end_arg, argv[0]);
        return 1;
    }

    int start_addr = i2cscan_args.start->count ? i2cscan_args.start->ival[0] : 0x01;
    int end_addr = i2cscan_args.end->count ? i2cscan_args.end->ival[0] : 0x77;

    ESP_LOGI(TAG, "Scanning I2C bus from 0x%02X to 0x%02X:", start_addr, end_addr);

    for (int addr = start_addr; addr <= end_addr; ++addr)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
        }
    }
    return 0;
}

static int do_i2c_read_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&i2c_r_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, i2c_r_args.end_arg, argv[0]);
        return 1;
    }

    /* Check if mandatory arguments are provided */
    if (i2c_r_args.address->count == 0 || i2c_r_args.nbytes->count == 0)
    {
        ESP_LOGE(TAG, "Address and number of bytes (-n) are required.");
        return 1;
    }

    uint8_t addr = (uint8_t)i2c_r_args.address->ival[0];
    int num_bytes_to_read = i2c_r_args.nbytes->ival[0];

    if (num_bytes_to_read <= 0 || num_bytes_to_read > 256 /* Arbitrary limit, adjust as needed */)
    {
        ESP_LOGE(TAG, "Invalid number of bytes to read (must be between 1 and 256).");
        return 1;
    }

    uint8_t *data_buf = malloc(num_bytes_to_read);
    if (!data_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for read buffer.");
        return 1;
    }

    ESP_LOGI(TAG, "Reading %d byte(s) from I2C address 0x%02X...", num_bytes_to_read, addr);
    int ret = i2c_read(addr, data_buf, num_bytes_to_read);

    if (ret == 0)
    {
        char *hex_buf = malloc(num_bytes_to_read * 5 + 1);
        for (int i = 0; i < num_bytes_to_read; ++i)
        {
            sprintf(&hex_buf[5 * i], "0x%02X ", data_buf[i]);
        }
        ESP_LOGI(TAG, "Read data: %s", hex_buf);
        free(hex_buf);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read from I2C address 0x%02X.", addr);
    }

    free(data_buf);
    return ret == 0 ? 0 : 1;
}

static int do_i2c_write_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&i2c_w_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, i2c_w_args.end_arg, argv[0]);
        return 1;
    }

    /* Check if mandatory arguments are provided */
    if (i2c_w_args.address->count == 0 || i2c_w_args.data->count == 0)
    {
        ESP_LOGE(TAG, "Address and at least one data byte are required.");
        return 1;
    }

    uint8_t addr = (uint8_t)i2c_w_args.address->ival[0];
    int num_bytes_to_write = i2c_w_args.data->count;

    /* arg_intn already constrains count, but an explicit check is fine */
    if (num_bytes_to_write <= 0)
    {
        ESP_LOGE(TAG, "Number of data bytes must be bigger than one.");
        return 1;
    }

    uint8_t *data_buf = malloc(num_bytes_to_write);
    if (!data_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for write buffer.");
        return 1;
    }

    /* Prepare buffer for hex output of data to be written */
    char *hex_buf = malloc(num_bytes_to_write * 5 + 1);
    for (int i = 0; i < num_bytes_to_write; ++i)
    {
        if (i2c_w_args.data->ival[i] < 0 || i2c_w_args.data->ival[i] > 0xFF)
        {
            ESP_LOGE(TAG, "Data byte 0x%X (at index %d) is out of range (0x00-0xFF).", i2c_w_args.data->ival[i], i);
            free(data_buf);
            free(hex_buf);
            return 1;
        }
        data_buf[i] = (uint8_t)i2c_w_args.data->ival[i];
        sprintf(&hex_buf[5 * i], "0x%02X ", data_buf[i]);
    }
    ESP_LOGI(TAG, "Writing %d byte(s) to I2C address 0x%02X: %s", num_bytes_to_write, addr, hex_buf);
    free(hex_buf);

    int ret = i2c_write(addr, data_buf, num_bytes_to_write);

    if (ret == 0)
    {
        ESP_LOGI(TAG, "Successfully wrote to I2C address 0x%02X.", addr);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to write to I2C address 0x%02X.", addr);
    }

    free(data_buf);
    return ret == 0 ? 0 : 1;
}

static int do_i2c_rw_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&i2c_rw_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, i2c_rw_args.end_arg, argv[0]);
        return 1;
    }

    /* Check if mandatory arguments are provided */
    if (i2c_rw_args.address->count == 0 || i2c_rw_args.wdata->count == 0 || i2c_rw_args.rbytes->count == 0)
    {
        ESP_LOGE(TAG, "Address, at least one data byte to write (-w), and number of bytes to read (-r) are required.");
        return 1;
    }

    uint8_t addr = (uint8_t)i2c_rw_args.address->ival[0];

    int num_bytes_to_write = i2c_rw_args.wdata->count;

    uint8_t *w_data_buf = malloc(num_bytes_to_write);
    for (int i = 0; i < num_bytes_to_write; ++i)
    {
        char *end;
        long v = strtol(i2c_rw_args.wdata->sval[i], &end, 0); // auto 0x / dec
        if (*end || v < 0 || v > 0xFF)
        {
            ESP_LOGE(TAG, "Bad hex byte: %s", i2c_rw_args.wdata->sval[i]);
            return 1;
        }
        w_data_buf[i] = (uint8_t)v;
    }

    int num_bytes_to_read = i2c_rw_args.rbytes->ival[0];

    int cyclic_ms = 0;
    int cyclic_count = 1; /* Default to 1 execution if --cyclic is not provided */

    if (i2c_rw_args.cyclic_ms->count > 0 && i2c_rw_args.cyclic_count->count > 0)
    {
        cyclic_ms = i2c_rw_args.cyclic_ms->ival[0];
        cyclic_count = i2c_rw_args.cyclic_count->ival[0];
        if (cyclic_ms < 0)
        {
            ESP_LOGE(TAG, "Cyclic delay (ms) must be non-negative.");
            return 1;
        }
        if (cyclic_count <= 0)
        {
            ESP_LOGE(TAG, "Cyclic count must be positive.");
            return 1;
        }
        ESP_LOGI(TAG, "Cyclic mode: %d times, %d ms delay", cyclic_count, cyclic_ms);
    }
    else if (i2c_rw_args.cyclic_ms->count > 0 || i2c_rw_args.cyclic_count->count > 0)
    {
        ESP_LOGE(TAG, "Both <ms> and <count> must be provided for --cyclic option.");
        return 1;
    }

    if (num_bytes_to_write <= 0)
    {
        ESP_LOGE(TAG, "Data bytes to write missing.");
        return 1;
    }

    if (num_bytes_to_read <= 0 || num_bytes_to_read > 256 /* Arbitrary limit, adjust as needed */)
    {
        ESP_LOGE(TAG, "Invalid number of bytes to read (must be between 1 and 256).");
        return 1;
    }

    uint8_t *r_data_buf = malloc(num_bytes_to_read);
    if (!r_data_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for read buffer.");
        free(w_data_buf);
        return 1;
    }

    /* Prepare write-data buffer and log it */
    char *hex_buf = malloc(num_bytes_to_write * 5 + 1);

    for (int i = 0; i < num_bytes_to_write; ++i)
    {
        /* Parse each byte string (accepts 0xNN, decimal, etc.) */
        char *end;
        long v = strtol(i2c_rw_args.wdata->sval[i], &end, 0);
        if (*end != '\0' || v < 0 || v > 0xFF)
        {
            ESP_LOGE(TAG, "Invalid byte \"%s\" at index %d (expect 0x00-0xFF).",
                     i2c_rw_args.wdata->sval[i], i);
            free(w_data_buf);
            free(r_data_buf);
            free(hex_buf);
            return 1;
        }

        w_data_buf[i] = (uint8_t)v;
        sprintf(&hex_buf[i * 5], "0x%02X ", w_data_buf[i]);
    }

    int overall_ret = 0;

    for (int cycle = 0; cycle < cyclic_count; ++cycle)
    {
        if (cyclic_count > 1)
        {
            ESP_LOGI(TAG, "Cycle %d/%d: Writing %d byte(s) [%s] to I2C addr 0x%02X, then reading %d byte(s)...",
                     cycle + 1, cyclic_count, num_bytes_to_write, hex_buf, addr, num_bytes_to_read);
        }
        else
        {
            ESP_LOGI(TAG, "Writing %d byte(s) [%s] to I2C addr 0x%02X, then reading %d byte(s)...", num_bytes_to_write, hex_buf, addr, num_bytes_to_read);
        }

        int ret = i2c_write_read(addr, w_data_buf, num_bytes_to_write, r_data_buf, num_bytes_to_read);

        if (ret == 0)
        {
            char *hex_buf = malloc(num_bytes_to_read * 5 + 1);
            for (int i = 0; i < num_bytes_to_read; ++i)
            {
                sprintf(&hex_buf[i * 5], "0x%02X ", r_data_buf[i]);
            }
            ESP_LOGI(TAG, "Read data: %s", hex_buf);
            free(hex_buf);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to write/read I2C address 0x%02X in cycle %d.", addr, cycle + 1);
            overall_ret = 1; /* Mark failure if any cycle fails */
            if (cyclic_count > 1)
            { /* Potentially break or continue based on desired error handling for cyclic */
                /* For now, continue all cycles but report overall failure */
            }
        }
        if (cycle < cyclic_count - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(cyclic_ms));
        }
    }

    free(w_data_buf);
    free(r_data_buf);
    free(hex_buf);
    return overall_ret;
}

void register_i2c_commands(void) /* Renamed from register_i2cscan_command */
{
    /* i2cscan command registration (existing) */
    i2cscan_args.start = arg_int0("s", "start", "<addr>", "Start address (default 0x03)");
    i2cscan_args.end = arg_int0("e", "end", "<addr>", "End address (default 0x77)");
    i2cscan_args.end_arg = arg_end(2);

    const esp_console_cmd_t i2cscan_cmd_config = {
        .command = "i2cscan",
        .help = "Scan for I2C devices on the bus",
        .hint = NULL,
        .func = &do_i2cscan,
        .argtable = &i2cscan_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&i2cscan_cmd_config));

    /* i2c_r command registration */
    i2c_r_args.address = arg_int1(NULL, NULL, "<addr>", "I2C device address (e.g., 0x50 or 80)");
    i2c_r_args.nbytes = arg_int1("n", "num", "<nbytes>", "Number of bytes to read (1-256)");
    i2c_r_args.end_arg = arg_end(2); /* Max 2 errors to store for address and nbytes */

    const esp_console_cmd_t i2c_r_cmd_config = {
        .command = "i2c_r",
        .help = "Read N bytes from an I2C device. Usage: i2c_r <addr> -n <nbytes>",
        .hint = " <addr> -n <nbytes>",
        .func = &do_i2c_read_cmd,
        .argtable = &i2c_r_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&i2c_r_cmd_config));

    /* i2c_w command registration */
    i2c_w_args.address = arg_int1(NULL, NULL, "<addr>", "I2C device address (e.g., 0x50 or 80)");
    i2c_w_args.data = arg_intn(NULL, NULL, "<byte>", 1, MAX_I2C_WRITE_BYTES, "Data byte(s) to write (e.g., 0x01 0xAA 255)");
    i2c_w_args.end_arg = arg_end(MAX_I2C_WRITE_BYTES + 1); /* Max errors: 1 for addr + N for data bytes */

    const esp_console_cmd_t i2c_w_cmd_config = {
        .command = "i2c_w",
        .help = "Write byte(s) to an I2C device. Usage: i2c_w <addr> <byte1> [byte2 ... byteN]",
        .hint = " <addr> <byte1> [byte2...]",
        .func = &do_i2c_write_cmd,
        .argtable = &i2c_w_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&i2c_w_cmd_config));

    /* i2c_rw command registration */
    i2c_rw_args.address = arg_int1(NULL, NULL, "<addr>", "I2C device address (e.g., 0x50 or 80)");
    i2c_rw_args.wdata = arg_strn("w", "wdata", "<byte>", 1, MAX_I2C_WRITE_BYTES, "Data byte(s) to write");
    i2c_rw_args.rbytes = arg_int1("r", "rbytes", "<nbytes>", "Number of bytes to read (1-256)");
    i2c_rw_args.cyclic_ms = arg_int0(NULL, "cyclic", "<ms> <count>", "Optional: Repeat <count> times with <ms> delay. Both ms and count required if used.");
    i2c_rw_args.cyclic_count = arg_int0(NULL, NULL, "<count>", "Number of repetitions for --cyclic");
    i2c_rw_args.end_arg = arg_end(MAX_I2C_WRITE_BYTES + 4); /* Max errors: 1 for addr, N for wdata, 1 for rbytes, 1 for ms, 1 for count */

    const esp_console_cmd_t i2c_rw_cmd_config = {
        .command = "i2c_rw",
        .help = "Write byte(s) to an I2C device then read N bytes. Usage: i2c_rw <addr> -w <byte1>... -r <nbytes> [--cyclic <ms> <count>]",
        .hint = " <addr> -w <byte1>... -r <nbytes> [--cyclic <ms> <count>]",
        .func = &do_i2c_rw_cmd,
        .argtable = &i2c_rw_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&i2c_rw_cmd_config));
}

int i2c_write(uint8_t addr, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write(cmd, (uint8_t *)data, len, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

int i2c_write_partial(uint8_t addr, const uint8_t *data, size_t len, bool stop)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write(cmd, (uint8_t *)data, len, 1);
    if (stop)
    {
        i2c_master_stop(cmd);
    }
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

int i2c_read(uint8_t addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, 1);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

int i2c_write_read(uint8_t addr, const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
    if (wlen > 0)
    {
        i2c_master_write(cmd, (uint8_t *)wdata, wlen, 1);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, 1);
    if (rlen > 1)
    {
        i2c_master_read(cmd, rdata, rlen - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, rdata + rlen - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

void i2c_init()
{
    /* Configure I2C parameters */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .sda_pullup_en = 1,
        .scl_pullup_en = 1,
        .master.clk_speed = 100000,
        .clk_flags = 0};
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    register_i2c_commands(); 
}
