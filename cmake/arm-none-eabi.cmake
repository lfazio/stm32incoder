set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_CC arm-none-eabi-gcc REQUIRED)
find_program(ARM_CXX arm-none-eabi-g++)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(ARM_SIZE arm-none-eabi-size REQUIRED)

set(CMAKE_C_COMPILER   ${ARM_CC})
set(CMAKE_ASM_COMPILER ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})
set(CMAKE_OBJCOPY      ${ARM_OBJCOPY})
set(CMAKE_SIZE         ${ARM_SIZE})

# Cortex-M4F: hardware FPU, single precision.
set(SIMENC_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# C library selection. Debian's gcc-arm-none-eabi ships picolibc; the ARM/xPack
# toolchains ship newlib with nano.specs. Pick whichever this machine has
# rather than assuming, so the tree builds on both.
execute_process(COMMAND ${ARM_CC} --print-file-name=picolibc.specs
                OUTPUT_VARIABLE _picolibc_specs OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${ARM_CC} --print-file-name=nano.specs
                OUTPUT_VARIABLE _nano_specs OUTPUT_STRIP_TRAILING_WHITESPACE)

if(IS_ABSOLUTE "${_picolibc_specs}" AND EXISTS "${_picolibc_specs}")
  # --crt0=none: the ST startup file already provides Reset_Handler and the
  # data/bss init, so picolibc's own crt0 must not be linked in.
  set(SIMENC_LIBC_FLAGS "--specs=picolibc.specs --crt0=none")
  set(SIMENC_LIBC_NAME "picolibc")
elseif(IS_ABSOLUTE "${_nano_specs}" AND EXISTS "${_nano_specs}")
  set(SIMENC_LIBC_FLAGS "--specs=nano.specs --specs=nosys.specs")
  set(SIMENC_LIBC_NAME "newlib-nano")
else()
  message(FATAL_ERROR "No usable C library found for ${ARM_CC} "
                      "(looked for picolibc.specs and nano.specs)")
endif()
message(STATUS "ARM C library: ${SIMENC_LIBC_NAME}")

set(CMAKE_C_FLAGS_INIT   "${SIMENC_CPU_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${SIMENC_CPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${SIMENC_CPU_FLAGS} ${SIMENC_LIBC_FLAGS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
