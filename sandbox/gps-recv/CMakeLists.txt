cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE "arm-none-eabi.cmake")

project(gps-recv)

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

	"startup/system_stm32f4xx.c"
	"startup/startup_stm32f411xe.s"

	"HAL/stm32f4xx_hal_it.c"
	"HAL/stm32f4xx_hal_msp.c"
	"HAL/hal_exceptions.c"

	"src/main.c"
	"src/nmea.c"
)

target_compile_definitions(${TARGET} PRIVATE
	STM32F411xE
	USE_HAL
)

target_include_directories(${TARGET} PRIVATE
	"../../third-party/cmsis_core/Include"
	"../../third-party/cmsis_device_f4/Include"
	"../../third-party/stm32f4xx_hal_driver/Inc"
	"HAL"
	"src"
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
	-T ${CMAKE_SOURCE_DIR}/STM32F411RETX_FLASH.ld
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

add_custom_command(
	TARGET ${TARGET} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET} ${CMAKE_PROJECT_NAME}.bin
	COMMAND ${CMAKE_SIZE} ${TARGET}
)
