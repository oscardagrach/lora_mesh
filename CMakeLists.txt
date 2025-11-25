cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pico_lora)

# Specify the directory containing the source files
set(SOURCE_DIR "src")

set(SOURCES
    "${SOURCE_DIR}/main.c"
    "${SOURCE_DIR}/led.c"
    "${SOURCE_DIR}/lora.c"
    "${SOURCE_DIR}/mesh.c"
    "${SOURCE_DIR}/mesh_handlers.c"
    "${SOURCE_DIR}/mesh_utils.c"
    "${SOURCE_DIR}/packet.c"
    "${SOURCE_DIR}/ring_buffer.c"
    "${SOURCE_DIR}/sensor.c"
    "${SOURCE_DIR}/work_queue.c"
)

zephyr_include_directories(include)

target_sources(app PRIVATE ${SOURCES})
