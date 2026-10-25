idf_component_register(SRCS "weiruan.c" "jianhao.c" "main.c"
                    INCLUDE_DIRS ".")
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    LV_COLOR_DEPTH=16
    LV_HOR_RES_MAX=320
    LV_VER_RES_MAX=480
    LV_MEM_SIZE=32768U
    LV_USE_BTN=1
    LV_USE_LABEL=1
    LV_USE_LOG=0
    LV_USE_PERF_MONITOR=0
)