cmake_minimum_required(VERSION 3.16)

set(CMAKE_AUTOMOC ON)

project(serial)

find_package(Qt6 REQUIRED COMPONENTS
	Core
	Network
	SerialPort
)

add_executable(${PROJECT_NAME}
	"src/main.cpp"
	"../../shared/protocol/protocol.c"
	"../../shared/communication/Serial.cpp"
	"../../shared/communication/Telnet.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"../../shared"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME} PRIVATE
	Qt6::Core
	Qt6::Network
	Qt6::SerialPort
)
