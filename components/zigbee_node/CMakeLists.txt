idf_component_register(
    SRCS "zigbee_node.c"
    INCLUDE_DIRS "include"
    REQUIRES esp-zigbee-lib esp-zboss-lib ld2410c vl53l0x
)
