cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE "arm-none-eabi.cmake")

project(ekf-test)

enable_language(ASM C)

set(TARGET ${CMAKE_PROJECT_NAME}.elf)

add_executable(${TARGET}
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_ll_usb.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_i2c.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_uart.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_can.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pcd.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pcd_ex.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_adc.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_adc_ex.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_exti.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_gpio.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_rcc.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pwr.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_dma.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_tim.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_tim_ex.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash_ex.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal_cortex.c"
	"../../third-party/stm32f4xx_hal_driver/Src/stm32f4xx_hal.c"

	"../../shared/protocol/protocol.c"

	"device/system_stm32f4xx.c"
	"device/startup_stm32f411retx.s"
	"device/syscalls.c"
	"device/sysmem.c"

	"HAL/stm32f4xx_it.c"
	"HAL/stm32f4xx_hal_msp.c"

	"src/main.c"
	"src/nmea.c"
	"src/bmp280_compensate.c"
	"src/estimator.c"
	"src/nvm.c"
	"src/comm.c"
)

target_compile_definitions(${TARGET} PRIVATE
	STM32F411xE
	USE_HAL
)

target_include_directories(${TARGET} PRIVATE
	"../../third-party/cmsis_core/Include"
	"../../third-party/cmsis_device_f4/Include"
	"../../third-party/stm32f4xx_hal_driver/Inc"
	"../../third-party/cmsis_core/DSP/Include"
	"../../shared/protocol"
	"HAL"
	"include"
)

target_compile_options(${TARGET} PRIVATE
	$<$<COMPILE_LANGUAGE:ASM>:
		-Os -g

		-mthumb
		-mhard-float
		-mcpu=cortex-m4
		-mfpu=fpv4-sp-d16
		-specs=nano.specs

		-x assembler-with-cpp
	>

	$<$<COMPILE_LANGUAGE:C>:
		-Os -g
		-std=c17

		-Wall
		-Wpedantic
		-Wdouble-promotion

		-ffunction-sections
		-fdata-sections
		-fstack-usage

		-mthumb
		-mhard-float
		-mcpu=cortex-m4
		-mfpu=fpv4-sp-d16
		-specs=nano.specs
	>
)

target_link_options(${TARGET} PRIVATE
	-T ${CMAKE_SOURCE_DIR}/linker.ld
	-Wl,-Map=${CMAKE_PROJECT_NAME}.map
	-Wl,--gc-sections
	-Wl,-cref
	-Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group

	-mthumb
	-mhard-float
	-mfloat-abi=hard
	-mcpu=cortex-m4
	-mfpu=fpv4-sp-d16
	-specs=nosys.specs
	-specs=nano.specs

	-static
	-u _printf_float
)

target_link_libraries(${TARGET}
	"${CMAKE_SOURCE_DIR}/../../third-party/cmsis_core/DSP/Lib/GCC/libarm_cortexM4lf_math.a"
)

add_custom_command(
	TARGET ${TARGET} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${CMAKE_PROJECT_NAME}.bin
	COMMAND ${CMAKE_OBJDUMP} -j .text -D ${TARGET} > ${CMAKE_PROJECT_NAME}.dump
	COMMAND ${CMAKE_SIZE} ${TARGET}
)
