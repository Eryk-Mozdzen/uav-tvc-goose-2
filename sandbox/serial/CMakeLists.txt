cmake_minimum_required(VERSION 3.16)

project(serial)

find_package(Qt6 REQUIRED COMPONENTS
	Core
	SerialPort
)

add_executable(${PROJECT_NAME}
	"src/main.cpp"
	"../../shared/protocol/protocol.c"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"../../shared/protocol"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME} PRIVATE
	Qt6::Core
	Qt6::SerialPort
)
