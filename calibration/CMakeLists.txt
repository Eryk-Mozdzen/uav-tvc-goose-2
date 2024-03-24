cmake_minimum_required(VERSION 3.16)

project(calibration)

set(CMAKE_AUTOMOC ON)

find_package(Eigen3 REQUIRED)
find_package(Qt6 COMPONENTS
	Widgets
	SerialPort
REQUIRED)

add_executable(${PROJECT_NAME}
	"../shared/protocol/protocol.c"
	"../shared/communication/Serial.cpp"

	"src/main.cpp"
	"src/Window.cpp"
	"src/magnetometer/Magnetometer.cpp"
	"src/magnetometer/Viewer.cpp"
	"src/accelerometer/Accelerometer.cpp"
	"src/accelerometer/Axis.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"../shared"
	"src"
	"src/magnetometer"
	"src/accelerometer"
	"src/servos"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O3
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen
	Qt6::Widgets
	Qt6::SerialPort
)
