idf_component_register(
    SRCS "ble_command_service.cpp"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash log common
)
