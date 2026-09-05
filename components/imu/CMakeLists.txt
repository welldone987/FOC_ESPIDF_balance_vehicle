idf_component_register(
    SRCS "bmi160_attitude.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board common log esp_timer freertos
)
