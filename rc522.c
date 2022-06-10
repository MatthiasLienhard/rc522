#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "rc522.h"

static const char *TAG = "ESP-RC522";

struct rc522
{
    bool running;
    rc522_config_t *config;
    spi_device_handle_t spi;
    TaskHandle_t task_handle;
    bool scan_started;
    uint8_t uid[10];
    uint8_t uid_len;
};


static struct rc522* rfid_reader = NULL;

#define rc522_fw_version() rc522_read(0x37)

bool rc522_is_inited()
{
    return rfid_reader != NULL;
}

static esp_err_t rc522_spi_init()
{
    if (!rfid_reader || !rfid_reader->config)
    {
        ESP_LOGE(TAG, "Fail to init SPI. Invalid handle");
        return ESP_ERR_INVALID_STATE;
    }

    if (rfid_reader->spi)
    {
        ESP_LOGW(TAG, "SPI already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = rfid_reader->config->spi_speed_hz,
        .mode = 0,
        .spics_io_num = rfid_reader->config->cs_io,
        .queue_size = 7,
        .address_bits=8};

    ESP_LOGD(TAG, "initializing SPI device");

    return spi_bus_add_device(rfid_reader->config->spi_host_id, &devcfg, &rfid_reader->spi);

}

static esp_err_t rc522_write_n(uint8_t addr, uint8_t n, uint8_t *data)
{
    
    spi_transaction_t t = {};
    uint8_t *buffer=NULL;
    t.length = 8 * n;
    t.addr=((addr << 1) & 0x7E);
	if (n <= 4) {
        t.flags = SPI_TRANS_USE_TXDATA;
        for (uint8_t i=0;i<n;i++){
            t.tx_data[i]=data[i];
        }
	} else {
        buffer = (uint8_t *) heap_caps_malloc(n, MALLOC_CAP_DMA);
        memcpy(buffer, data, n);
		t.tx_buffer = buffer;
	}	

    esp_err_t ret = spi_device_transmit(rfid_reader->spi, &t);
    if (buffer){
        free(buffer);
    }

    return ret;
}

static esp_err_t rc522_write(uint8_t addr, uint8_t val)
{
    return rc522_write_n(addr, 1, &val);
}

static uint8_t rc522_read(uint8_t addr)
{
    spi_transaction_t t={};
    t.flags = SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.addr=((addr << 1) & 0x7E) | 0x80;
    spi_device_transmit(rfid_reader->spi, &t);
    return t.rx_data[0];
}

static esp_err_t rc522_set_bitmask(uint8_t addr, uint8_t mask)
{
    return rc522_write(addr, rc522_read(addr) | mask);
}

static esp_err_t rc522_clear_bitmask(uint8_t addr, uint8_t mask)
{
    return rc522_write(addr, rc522_read(addr) & ~mask);
}

static esp_err_t rc522_antenna_on()
{
    return rc522_set_bitmask(0x14, 0x03);
    //return rc522_write(0x26, 0x60); // 43dB gain
}

static esp_err_t rc522_antenna_off()
{
    return rc522_clear_bitmask(0x14, 0x03);
}

static void rc522_task(void *arg);

esp_err_t rc522_init(rc522_config_t *config)
{
    if (!config)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (rfid_reader)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!(rfid_reader = calloc(1, sizeof(struct rc522))))
    {
        return ESP_ERR_NO_MEM;
    }

    if (!(rfid_reader->config = calloc(1, sizeof(rc522_config_t))))
    {
        rc522_destroy();
        return ESP_ERR_NO_MEM;
    }
    rfid_reader->uid_len = 0;
    // copy config considering defaults
    rfid_reader->config->callback = config->callback;
    rfid_reader->config->spi_speed_hz = config->spi_speed_hz == 0 ? RC522_DEFAULT_CLK_SPEED : config->spi_speed_hz;
    rfid_reader->config->cs_io = config->cs_io == 0 ? RC522_DEFAULT_CS : config->cs_io;
    rfid_reader->config->spi_host_id = config->spi_host_id == 0 ? RC522_DEFAULT_SPI_HOST : config->spi_host_id;
    rfid_reader->config->scan_interval_ms = config->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : config->scan_interval_ms;
    rfid_reader->config->task_stack_size = config->task_stack_size == 0 ? RC522_DEFAULT_TACK_STACK_SIZE : config->task_stack_size;
    rfid_reader->config->task_priority = config->task_priority == 0 ? RC522_DEFAULT_TACK_STACK_PRIORITY : config->task_priority;

    esp_err_t err = rc522_spi_init();
    if (err != ESP_OK)
    {
        ESP_LOGD(TAG, "SPI init failed");
        rc522_destroy();
        return err;
    }
    

    // ---------- RW test ------------
    const uint8_t test_addr = 0x24, test_val = 0x25;
    for (uint8_t i = test_val; i < test_val + 2; i++)
    {
        ESP_LOGD(TAG, "RW test %#x",i);
        if ((err = rc522_write(test_addr, i)) != ESP_OK)
        {
            ESP_LOGE(TAG, "RW test fail: write");
            rc522_destroy();
            return err;
        }else if (rc522_read(test_addr) != i){
            ESP_LOGE(TAG, "RW test fail: read %#x", rc522_read(test_addr));
            rc522_destroy();
            return err;
        }
    }
    ESP_LOGD(TAG, "RW test OK");
    // ------- End of RW test --------

    rc522_write(0x01, 0x0F); // reset
    rc522_write(0x2A, 0x8D); // timer mode
    rc522_write(0x2B, 0x3E); // timer prescale
    rc522_write(0x2D, 0x30); // timer reload (lower)<-0x30
    rc522_write(0x2C, 0x00); // timer reload (higer)
    rc522_write(0x15, 0x40); // transmit modulation (forces a 100 % ASK modulation independent of the ModGsPReg register setting)
    rc522_write(0x11, 0x3D); // general transmitting and receiving modes: default but CRC preset to 0x6363


    rfid_reader->running = true;
    if (xTaskCreate(rc522_task, "rc522_task", rfid_reader->config->task_stack_size, NULL, rfid_reader->config->task_priority, &rfid_reader->task_handle) != pdTRUE)
    {
        ESP_LOGE(TAG, "Fail to create rc522 task");
        rc522_destroy();
        return err;
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create timer");
        rc522_destroy();
        return err;
    }

    ESP_LOGI(TAG, "Initialized (firmware: 0x%x)", rc522_fw_version());
    return ESP_OK;
}

esp_err_t rc522_buffer_to_str(uint8_t *buffer, uint8_t buffer_len, char *str_buffer, uint8_t str_len)
{

    char *endofstr = str_buffer + str_len;
    for (uint8_t i = 0; i < buffer_len; i++)
    {
        if (str_buffer + 3 < endofstr)
        {
            if (i > 0)
            {
                str_buffer += sprintf(str_buffer, ":");
            }
            str_buffer += sprintf(str_buffer, "%02X", buffer[i]);
        }
        else
        {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

static esp_err_t rc522_calculate_crc(uint8_t *data, uint8_t n, uint8_t *res)
{
    rc522_clear_bitmask(0x05, 0x04);
    rc522_set_bitmask(0x0A, 0x80);

    rc522_write_n(0x09, n, data);

    rc522_write(0x01, 0x03);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    if (rc522_read(0x05) & 0x04)
    {
        res[0] = rc522_read(0x22);
        res[1] = rc522_read(0x21);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t rc522_communicate(uint8_t cmd, uint8_t *send_data, uint8_t send_n, uint8_t *res_data, uint8_t *res_n)
{
    
    ESP_LOGD(TAG, "communication %#x, data:", cmd);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, send_data, send_n, ESP_LOG_DEBUG);
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;

    if (cmd == 0x0E)
    {
        irq = 0x12;
        irq_wait = 0x10;
    }
    else if (cmd == 0x0C)
    {
        irq = 0x77;
        irq_wait = 0x30;
    }

    rc522_write(0x02, irq | 0x80);
    rc522_clear_bitmask(0x04, 0x80);
    rc522_set_bitmask(0x0A, 0x80); // flush FIFO
    rc522_write(0x01, 0x00);       // wake up, idle

    rc522_write_n(0x09, send_n, send_data);

    rc522_write(0x01, cmd);

    if (cmd == 0x0C)
    {
        rc522_set_bitmask(0x0D, 0x80); // start transceive transmission
    }

    uint16_t i;

    for (i = 10; i > 0; i--)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        uint8_t irq = rc522_read(0x04);

        if (irq & irq_wait)
        {
            ESP_LOGD(TAG, "interrupt bits: %#x", irq); // 0x45
            break;
        }
        else if (irq & 1)
        {
            ESP_LOGD(TAG, "Timeout irq");
            return ESP_ERR_TIMEOUT;
        }
        
    }
    if (i == 0)
    {
        ESP_LOGE(TAG, "Communication no response");
        return ESP_ERR_TIMEOUT; // time out
    }
    ESP_LOGD(TAG, "Communication steps: %i", i);

    rc522_clear_bitmask(0x0D, 0x80); // start transceive receive
    if (rc522_read(0x06) & 0x1B)
    { // 0x06 is the ERROR register - BufferOvfl(5) CollErr(4) ParityErr(2) ProtocolErr(1)
        ESP_LOGW(TAG, "Error bits during communication: %#x", rc522_read(0x06));
        char *data_str = (char *)malloc(30);
        rc522_buffer_to_str(send_data, send_n, data_str, 30);
        ESP_LOGW(TAG, "CMD %#x data: %s ", cmd, data_str);
        free(data_str);

        return ESP_FAIL;
    }
    if (res_data && res_n)
    { // return data expected
        uint8_t nn = rc522_read(0x0A);
        // if (last_bits){
        //     *last_bits = rc522_read(0x0C) & 0x07; //should not happen, CollError
        // }
        if (nn > *res_n)
        {
            ESP_LOGE(TAG, "Buffer too small");
            return ESP_ERR_NO_MEM; // ERROR
        }
        for (i = 0; i < nn; i++)
        {
            res_data[i] = rc522_read(0x09);
        }
        *res_n = nn;
    }
    ESP_LOGD(TAG, "return %i bytes of data:",*res_n);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, res_data, *res_n, ESP_LOG_DEBUG);
    return ESP_OK;
}

static esp_err_t rc522_request()
{
    rc522_write(0x0D, 0x07); // 7 bit framing

    uint8_t req_mode = 0x26; // reqA
    uint8_t buffer[2];
    uint8_t buffer_len = 2;
    esp_err_t state = rc522_communicate(0x0C, &req_mode, 1, buffer, &buffer_len);
    ESP_LOGD(TAG, "reqA returned %#x:%#x",buffer[0], buffer[1]);
    return state;
}

static esp_err_t rc522_halt()
{
    rc522_write(0x0D, 0x00); // 8 bit framing
    uint8_t buffer[4];
    uint8_t buffer_len;
    buffer[0] = 0x50;
    buffer[1] = 0x00;
    esp_err_t state = rc522_calculate_crc(buffer, 2, buffer + 2);
    if (state != ESP_OK)
    {
        return state;
    }
    state = rc522_communicate(0x0C, buffer, 4, buffer, &buffer_len);
    // suppose to timeout
    // on fail, it may return something
    if (state == ESP_ERR_TIMEOUT)
    {
        return ESP_OK;
    }
    else if (state == ESP_OK)
    {
        return ESP_FAIL;
    }
    return state;
}

static esp_err_t rc522_get_tag()
{
    
    uint8_t buffer[9];
    uint8_t res_n;
    uint8_t uid_len = 0;
    esp_err_t state = rc522_request();
    if (state == ESP_OK)
    {
        uint8_t uid[10];

        rc522_write(0x0D, 0x00); // 8 bits
        for (uint8_t level = 0; level < 3; level++)
        {
            ESP_LOGD(TAG, "Level %i anticol", level);
            buffer[0] = 0x93 + (level * 2);
            buffer[1] = 0x20;
            res_n = 5; // expect 5 bytes
            state = rc522_communicate(0x0C, buffer, 2, buffer + 2, &res_n);
            if (state != ESP_OK)
            {
                break;
            }
            if (res_n != 5)
            {
                state = ESP_ERR_INVALID_SIZE;
                break;
            }
            ESP_LOGD(TAG, "Level %i crc", level);
            buffer[1] = 0x70;
            state = rc522_calculate_crc(buffer, 7, buffer + 7);
            if (state != ESP_OK)
            {
                break;
            }
            ESP_LOGD(TAG, "Level %i select: %#x %#x %#x %#x %#x %#x %#x %#x %#x", level, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8]);
            res_n = 3; // 1 byte SAK and 2 byte CRC - encodes card type
            state = rc522_communicate(0x0C, buffer, 9, buffer + 6, &res_n);
            if (state != ESP_OK)
            {
                break;
            }
            else if (res_n != 3)
            {
                state = ESP_ERR_INVALID_SIZE;
                break;
            }
            ESP_LOGD(TAG, "Level %i cp uid %#x %#x %#x %#x ", level, buffer[2], buffer[3], buffer[4], buffer[5]);
            if (buffer[2] == 0x88)
            { // can also check buffer[6] & 4
                for (uint8_t i = 0; i < 3; i++)
                {
                    uid[i + level * 3] = buffer[i + 3];
                }
                uid_len += 3;
            }
            else
            {
                for (uint8_t i = 0; i < 4; i++)
                {
                    uid[i + level * 3] = buffer[i + 2];
                }
                uid_len += 4;
                break;
            }
        }
        if (state == ESP_OK)
        {
            if (rfid_reader->uid_len == 0)
            { //&& new_card
                rfid_reader->uid_len = uid_len;
                memcpy(rfid_reader->uid, uid, uid_len);
                ESP_LOGD(TAG, "New Card");
                rc522_handler_t cb = rfid_reader->config->callback;
                if (cb)
                {
                    cb(rfid_reader->uid, rfid_reader->uid_len, false);
                }
            }
        } // else ? reqA, but no uid read... count errors?
    }else if (rfid_reader->uid_len > 0)
    {
        // add card gone callback?
        ESP_LOGD(TAG, "Card Gone");
        rc522_handler_t cb = rfid_reader->config->callback;
        if (cb){
            cb(rfid_reader->uid, rfid_reader->uid_len, true);
        }
        rfid_reader->uid_len = 0;
    }
    return state;
}


esp_err_t rc522_start()
{
    if (!rfid_reader)
    {
        return ESP_ERR_INVALID_STATE;
    }

    rfid_reader->scan_started = true;

    return ESP_OK;
}

esp_err_t rc522_pause()
{
    if (!rfid_reader)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!rfid_reader->scan_started)
    {
        return ESP_OK;
    }

    rfid_reader->scan_started = false;

    return ESP_OK;
}

void rc522_destroy()
{
    if (!rfid_reader)
    {
        return;
    }

    rc522_pause();         // stop timer
    rfid_reader->running = false; // task will delete itself

    if (rfid_reader->spi)
    {
        spi_bus_remove_device(rfid_reader->spi);
        rfid_reader->spi = NULL;
    }

    free(rfid_reader->config);
    rfid_reader->config = NULL;

    free(rfid_reader);
    rfid_reader = NULL;
}

static void rc522_task(void *arg)
{
    while (rfid_reader->running)
    {
        if (!rfid_reader->scan_started)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        rc522_antenna_on();
        esp_err_t state = rc522_get_tag();
        if (state == ESP_OK)
        {
            state = rc522_halt();
        }
        rc522_antenna_off();
        int delay_interval_ms = rfid_reader->config->scan_interval_ms;
        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}