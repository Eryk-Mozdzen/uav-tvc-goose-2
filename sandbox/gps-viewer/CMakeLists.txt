cmake_minimum_required(VERSION 3.16)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

project(gps-viewer)

find_package(Qt6 COMPONENTS
	Widgets
	WebEngineWidgets
	SerialPort
REQUIRED)

add_executable(${PROJECT_NAME}
	"../../shared/protocol/protocol.c"
	"../../shared/communication/Serial.cpp"

	"src/main.cpp"
	"src/Map.cpp"

	"resources.qrc"
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

target_link_libraries(${PROJECT_NAME}
	Qt6::Widgets
	Qt6::WebEngineWidgets
	Qt6::SerialPort
)
