idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES ld2410c vl53l0x zigbee_node nvs_flash
)
