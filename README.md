# esp-idf-rc522

C library for interfacing ESP32 with MFRC522 RFID card reader. 
Modified to work with 7 byte tags, which involves cascading. 

## Demo

[![How To Connect MFRC522 With ESP32 | ESP IDF Component](https://img.youtube.com/vi/IHaccsDMg9s/mqdefault.jpg)](https://www.youtube.com/watch?v=IHaccsDMg9s)

## How to use

This directory is an ESP-IDF component. Clone it (or add it as submodule) into `components` directory of the project.

## Example

```c
#include "esp_log.h"
#include "rc522.h"



static const char* TAG = "TEST_RFID";

void rfid_handler(uint8_t* uid, uint8_t uid_len, bool remove) { 
    char* uid_string=(char*) malloc(uid_len*3);
    rc522_buffer_to_str(uid,uid_len, uid_string, uid_len*3);    
    if (remove){
        ESP_LOGI(TAG, "RFID Tag %s removed",uid_string );

    }else{
        ESP_LOGI(TAG, "New RFID Tag: %s",uid_string );
    }
    free(uid_string);
}

void app_main(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = 19,
        .mosi_io_num = 23,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    esp_err_t err = spi_bus_initialize(VSPI_HOST, &buscfg, 1);

    const rc522_config_t rfid_config = {
        .callback = &rfid_handler,
        .cs_io=5,
        .scan_interval_ms=500,
        .task_priority=4
    };
    assert(rc522_init(&rfid_config) == ESP_OK);
    rc522_start();
}
```

## Author

GitHub: [abobija](https://github.com/abobija)<br>
Homepage: [abobija.com](https://abobija.com)

## License

[MIT](LICENSE)