cmake_minimum_required(VERSION 3.20)

project(simenc C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ST publishes the Cube package split into per-component repositories. Using
# those three directly instead of the whole STM32CubeF4 pulls 7 MB of git
# history rather than 97 MB, and this project only ever compiled these parts.
set(CMSIS_CORE ${CMAKE_CURRENT_SOURCE_DIR}/vendor/cmsis_core)
set(CMSIS_DEV  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/cmsis_device_f4)
set(HAL        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/stm32f4xx_hal_driver)

if(NOT EXISTS ${HAL}/Src/stm32f4xx_hal.c OR NOT EXISTS ${CMSIS_CORE}/CMSIS/Core/Include/core_cm4.h)
  message(FATAL_ERROR
    "Vendor submodules are missing. Run:\n"
    "  git submodule update --init")
endif()

# --- Clock source -------------------------------------------------------------
# AUTO (default): try HSE (8 MHz from the ST-LINK MCO, UM1724 7.9.1) and fall
# back to HSI if the board's solder bridges do not provide it.
#
# HSE matters: the SSI4 Time Stamp is specified accurate to "better than 1%
# (based on the system oscillator)", and measured against a logic analyser HSI
# is +1.40% (out of spec) while HSE is +0.02%.
#
# Force one with -DSIMENC_CLOCK_SOURCE=HSE (hang if absent) or =HSI.
set(SIMENC_CLOCK_SOURCE "AUTO" CACHE STRING "System clock source: AUTO, HSE or HSI")
set_property(CACHE SIMENC_CLOCK_SOURCE PROPERTY STRINGS AUTO HSE HSI)


set(HAL_MODULES
    hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_dma hal_dma_ex hal_pwr
    hal_pwr_ex hal_flash hal_flash_ex hal_flash_ramfunc hal_adc hal_adc_ex
    hal_tim hal_tim_ex hal_uart)

set(HAL_SOURCES "")
foreach(m ${HAL_MODULES})
  list(APPEND HAL_SOURCES ${HAL}/Src/stm32f4xx_${m}.c)
endforeach()

add_executable(simenc.elf
    src/main.c
    src/stm32f4xx_it.c
    src/system_stm32f4xx.c
    src/startup_stm32f446xx.s
    src/board/board.c
    src/board/trace.c
    src/board/console.c
    src/ssi/ssi_slave.c
    src/ssi/ssi_master.c
    src/ssi/ssi_variant.c
    src/encoder/incoder.c
    src/encoder/position_source.c
    ${HAL_SOURCES}
)

target_include_directories(simenc.elf PRIVATE
    src
    ${CMSIS_CORE}/CMSIS/Core/Include
    ${CMSIS_DEV}/Include
    ${HAL}/Inc
)

target_compile_definitions(simenc.elf PRIVATE
    STM32F446xx
    USE_HAL_DRIVER
    SIMENC_CLOCK_SOURCE_${SIMENC_CLOCK_SOURCE}=1
)

target_compile_options(simenc.elf PRIVATE
    -Wall -Wextra -Wshadow -Wundef
    -ffunction-sections -fdata-sections
    $<$<CONFIG:Debug>:-Og -g3>
    $<$<CONFIG:Release>:-O2 -g>
)

set(LDSCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F446RETX_FLASH.ld)
target_link_options(simenc.elf PRIVATE
    -T${LDSCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/simenc.map
    -Wl,--print-memory-usage
)
set_target_properties(simenc.elf PROPERTIES LINK_DEPENDS ${LDSCRIPT})

add_custom_command(TARGET simenc.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:simenc.elf> ${CMAKE_BINARY_DIR}/simenc.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:simenc.elf> ${CMAKE_BINARY_DIR}/simenc.hex
    COMMENT "Generating simenc.bin / simenc.hex")

add_custom_target(flash
    COMMAND openocd -f board/st_nucleo_f4.cfg
            -c "program $<TARGET_FILE:simenc.elf> verify reset exit"
    DEPENDS simenc.elf
    USES_TERMINAL
    COMMENT "Flashing via OpenOCD")
