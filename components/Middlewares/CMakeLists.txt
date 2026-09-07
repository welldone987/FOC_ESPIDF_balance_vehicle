# Middlewares 作为一个 ESP-IDF component 管理中间件与控制模块。
# 新增一级模块目录后，C/C++/ASM 源文件与该目录的头文件路径会在 CMake 配置阶段自动加入构建。
# 注意：ESP-IDF 会以 script mode 预解析组件依赖，因此这里不能使用 CONFIGURE_DEPENDS。
file(GLOB middleware_module_entries
    RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/*"
)

set(middleware_srcs)
set(middleware_include_dirs)

foreach(module IN LISTS middleware_module_entries)
    if(NOT IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${module}")
        continue()
    endif()

    list(APPEND middleware_include_dirs "${module}")
    if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${module}/include")
        list(APPEND middleware_include_dirs "${module}/include")
    endif()

    file(GLOB_RECURSE module_srcs
        RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.cc"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/${module}/*.S"
    )
    list(APPEND middleware_srcs ${module_srcs})
endforeach()

# 组件依赖保持显式声明；BSP 是本项目内部依赖，ESP-IDF 自带组件
# 由构建系统直接解析。这样比从 include 文本猜依赖更可靠。
set(requires
    BSP
    bt
    esp_timer
    freertos
    heap
    log
    nvs_flash
)

idf_component_register(
    SRCS ${middleware_srcs}
    INCLUDE_DIRS ${middleware_include_dirs}
    REQUIRES ${requires}
)

# 与 BSP 使用同一套编译选项。
target_compile_options(${COMPONENT_LIB} PRIVATE
    -ffast-math
    -O3
    -Wno-error=format
    -Wno-format
)
