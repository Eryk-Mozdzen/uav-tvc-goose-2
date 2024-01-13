cmake_minimum_required(VERSION 3.16)

project(simulation-cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS
	Widgets
	Network
	Gamepad
	Charts
REQUIRED)

add_executable(${PROJECT_NAME}
	"src/main.cpp"
	"src/trajectory/Manual.cpp"
	"src/trajectory/Lemniscate.cpp"
	"src/trajectory/Circle.cpp"
	"src/simulation/Object.cpp"
	"src/simulation/Controller.cpp"
	"src/simulation/HoverController.cpp"
	"src/simulation/PositionController.cpp"
	"src/simulation/Simulation.cpp"
	"src/interface/Gamepad.cpp"
	"src/interface/Client.cpp"
	"src/graphs/Chart.cpp"
	"src/graphs/GraphXY.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"src"
	"src/trajectory"
	"src/simulation"
	"src/interface"
	"src/graphs"
	"src/abstract"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen
	Qt5::Widgets
	Qt5::Network
	Qt5::Gamepad
	Qt5::Charts
)
