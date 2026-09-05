idf_component_register(
    SRCS "power_monitor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board common log esp_adc
)
