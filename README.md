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

void tag_handler(uint8_t* uid, uint8_t uid_len) { 
    char* uid_string=(char*) malloc(uid_len*3);
    rc522_buffer_to_str(uid,uid_len, uid_string, uid_len*3);    
    ESP_LOGI(TAG, "RFID Tag: %s",uid_string );
    free(uid_string);
}

void app_main(void) {
    

    const rc522_config_t start_args = {
        .miso_io  = 19,
        .mosi_io  = 23,
        .sck_io   = 18,
        .sda_io   = 5,
        .callback = &tag_handler,
        .scan_interval_ms=500,
        .task_priority=4
        // Uncomment next line for attaching RC522 to SPI2 bus. Default is VSPI_HOST (SPI3
        //.spi_host_id = HSPI_HOST
    };

    rc522_start(start_args);
}
```

## Author

GitHub: [abobija](https://github.com/abobija)<br>
Homepage: [abobija.com](https://abobija.com)

## License

[MIT](LICENSE)