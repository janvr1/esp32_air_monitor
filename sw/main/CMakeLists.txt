idf_component_register(
    SRCS "main.c" "web_app/web_app.c" "utilities/jan_nvs.c" "utilities/jan_wifi.c"
    INCLUDE_DIRS "utilities"
)
spiffs_create_partition_image(storage ./static FLASH_IN_PROJECT)
