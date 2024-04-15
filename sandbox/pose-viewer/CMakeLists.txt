cmake_minimum_required(VERSION 3.16)

set(CMAKE_AUTOMOC ON)

project(pose-viewer)

find_package(Qt6 REQUIRED COMPONENTS
	Core
	Network
	SerialPort
)

add_executable(${PROJECT_NAME}
	"../../shared/protocol/protocol.c"
	"../../shared/communication/Serial.cpp"
	"../../shared/communication/Telnet.cpp"
	"../../shared/communication/Visualization3d.cpp"

	"src/main.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"../../shared"
	"src"
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
