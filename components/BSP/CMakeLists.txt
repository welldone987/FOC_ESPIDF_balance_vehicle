# BSP 作为一个 ESP-IDF component 管理所有板级模块。
# 新增一级模块目录后，C/C++/ASM 源文件与该目录的头文件路径会在 CMake 配置阶段自动加入构建。
# 注意：ESP-IDF 会以 script mode 预解析组件依赖，因此这里不能使用 CONFIGURE_DEPENDS。
file(GLOB bsp_module_entries
    RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/*"
)

set(bsp_srcs)
set(bsp_include_dirs)

foreach(module IN LISTS bsp_module_entries)
    if(NOT IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${module}")
        continue()
    endif()

    list(APPEND bsp_include_dirs "${module}")
    if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${module}/include")
        list(APPEND bsp_include_dirs "${module}/include")
    endif()

    file(GLOB_RECURSE module_srcs
        RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.cc"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.S"
    )
    list(APPEND bsp_srcs ${module_srcs})
endforeach()

# ESP-IDF组件依赖保持显式声明，第三方组件由idf_component.yml管理。
set(requires
    esp_adc
    esp_driver_gpio
    esp_timer
    freertos
    log
)

idf_component_register(
    SRCS ${bsp_srcs}
    INCLUDE_DIRS ${bsp_include_dirs}
    REQUIRES ${requires}
)

# ESP-IDF 6.x 推荐直接对当前 COMPONENT_LIB target 设置编译选项。
target_compile_options(${COMPONENT_LIB} PRIVATE
    -ffast-math
    -O3
    -Wno-error=format
    -Wno-format
)

# 电流目标的NaN/Inf拒绝检查不能被fast-math消除。
set_source_files_properties(Motor/motor_foc_service.cpp PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
