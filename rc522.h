#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_master.h"


#define RC522_DEFAULT_CS                   (5)
#define RC522_DEFAULT_CLK_SPEED            (8000000)
#define RC522_DEFAULT_SPI_HOST             (VSPI_HOST)
#define RC522_DEFAULT_SCAN_INTERVAL_MS     (125)
#define RC522_DEFAULT_TACK_STACK_SIZE      (4 * 1024)
#define RC522_DEFAULT_TACK_STACK_PRIORITY  (4)

typedef void(*rc522_handler_t)(uint8_t* , uint8_t, bool);

typedef struct {
    spi_host_device_t spi_host_id;  /*<! (Default: VSPI_HOST) */
    int cs_io;                      /*<! MFRC522 CS gpio  (Default: 5) */
    int spi_speed_hz;               /*<! SPI clock speed */
    rc522_handler_t callback;       /*<! Scanned tags handler */
    uint16_t scan_interval_ms;      /*<! How fast will ESP32 scan for nearby tags, in miliseconds. Default: 125ms */
    size_t task_stack_size;         /*<! Stack size of rc522 task (Default: 4 * 1024) */
    uint8_t task_priority;          /*<! Priority of rc522 task (Default: 4) */
} rc522_config_t;

/**
 * @brief Initialize RC522 module.
 *        To start scanning tags - call rc522_start function.
 * @param config Configuration
 * @return ESP_OK on success
 */
esp_err_t rc522_init(rc522_config_t* config);

/**
 * @brief Check if RC522 is inited
 * @return true if RC522 is inited
 */
bool rc522_is_inited();

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned.
 *        NOTE: This function is implemented because in time of implementation rc522_start function is intented for
 *        initialization and scanning in once. In future, when rc522_start gonna be refactored to just start to scan tags
 *        without initialization, this function will be just alias of rc522_start.
 * @return ESP_OK on success
 */
esp_err_t rc522_start();

/**
 * @brief Start to scan tags. If already started, ESP_OK will just be returned.
 * @return ESP_OK on success
 */

/**
 * @brief Pause scan tags. If already paused, ESP_OK will just be returned.
 * @return ESP_OK on success
 */
esp_err_t rc522_pause();

/**
 * @brief Destroy RC522 and free all resources
 */
void rc522_destroy();

esp_err_t rc522_buffer_to_str(uint8_t *buffer, uint8_t buffer_len, char *str_buffer, uint8_t str_len);


#ifdef __cplusplus
}
#endif