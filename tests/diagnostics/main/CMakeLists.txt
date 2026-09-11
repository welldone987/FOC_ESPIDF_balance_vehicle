set(root "${CMAKE_CURRENT_LIST_DIR}/../../..")
idf_component_register(SRCS "test_main.cpp" "${root}/components/BSP/Motor/current_control.cpp" "${root}/components/BSP/Motor/svpwm.cpp" "${root}/components/Middlewares/Control/balance_controller.cpp"
    INCLUDE_DIRS "stubs" "${root}/components/BSP/Error" "${root}/components/BSP/Board" "${root}/components/BSP/Encoder" "${root}/components/BSP/CurrentSensor" "${root}/components/BSP/Motor" "${root}/components/BSP/IMU" "${root}/components/Middlewares/Control" "${root}/components/Middlewares/BLE" "${root}/components/Middlewares/Diagnostics"
    REQUIRES unity)
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-fast-math)
