cmake_minimum_required(VERSION 3.20)

project(simenc C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CUBE ${CMAKE_CURRENT_SOURCE_DIR}/vendor/STM32CubeF4)
set(CMSIS ${CUBE}/Drivers/CMSIS)
set(HAL ${CUBE}/Drivers/STM32F4xx_HAL_Driver)

if(NOT EXISTS ${HAL}/Src/stm32f4xx_hal.c)
  message(FATAL_ERROR
    "STM32CubeF4 submodules are missing. Run:\n"
    "  git submodule update --init --recursive vendor/STM32CubeF4\n"
    "  git -C vendor/STM32CubeF4 submodule update --init --depth 1 \\\n"
    "      Drivers/CMSIS/Device/ST/STM32F4xx Drivers/STM32F4xx_HAL_Driver")
endif()

# --- Clock source -------------------------------------------------------------
# HSI (default): 16 MHz internal RC. Always boots regardless of board solder
# bridges. HSE: 8 MHz from the ST-LINK MCO (UM1724 7.9.1) -- better timestamp
# accuracy, but requires SB54/SB16/SB50 ON and SB55 OFF (factory default).
set(SIMENC_CLOCK_SOURCE "HSI" CACHE STRING "System clock source: HSI or HSE")
set_property(CACHE SIMENC_CLOCK_SOURCE PROPERTY STRINGS HSI HSE)

set(HAL_MODULES
    hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_dma hal_dma_ex hal_pwr
    hal_pwr_ex hal_flash hal_flash_ex hal_flash_ramfunc hal_adc hal_adc_ex
    hal_tim hal_tim_ex hal_uart hal_spi hal_exti)

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
    src/ssi/ssi4.c
    src/encoder/incoder.c
    src/encoder/position_source.c
    ${HAL_SOURCES}
)

target_include_directories(simenc.elf PRIVATE
    src
    ${CMSIS}/Include
    ${CMSIS}/Device/ST/STM32F4xx/Include
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
