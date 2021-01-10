idf_component_register(
    SRCS "main.c" "./sensors/jan_bme280.c" "sensors/jan_scd30.c" "sensors/jan_veml7700.c" "web_app/web_app.c" "jan_nvs.c" "jan_wifi.c"
    INCLUDE_DIRS ""
)
spiffs_create_partition_image(storage ./static FLASH_IN_PROJECT)