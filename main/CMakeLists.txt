idf_component_get_property(LVGL_DIR lvgl COMPONENT_DIR)

file(GLOB LVGL_DEMOS 
    ${LVGL_DIR}/demos/benchmark
    ${LVGL_DIR}/demos/benchmark/assets
)

idf_component_register(
    SRC_DIRS ${DIRS} ${LVGL_DEMOS} "."
    INCLUDE_DIRS "." ${LVGL_DEMOS} ${DIRS}
    REQUIRES esp_lcd driver lvgl esp_timer button mt8901
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

