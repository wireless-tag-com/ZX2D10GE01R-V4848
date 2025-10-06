idf_component_register(
    SRC_DIRS ${DIRS} "."
    INCLUDE_DIRS "." ${DIRS}
    REQUIRES zx2d10ge01r
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
