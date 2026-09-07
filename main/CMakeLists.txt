# main组件只注册app_main.cpp，并通过BSP与Middlewares取得应用边界接口。
idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES BSP Middlewares freertos log
)
