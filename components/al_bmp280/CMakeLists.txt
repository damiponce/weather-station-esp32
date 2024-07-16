idf_component_register(
    SRCS "al_bmp280.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES general pl_i2c log freertos
)
target_compile_options(${COMPONENT_LIB} PRIVATE  -Wno-format)
