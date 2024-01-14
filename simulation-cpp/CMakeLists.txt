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
	"src/graphs/Chart.cpp"
	"src/graphs/GraphXY.cpp"
	"src/interface/Client.cpp"
	"src/interface/Gamepad.cpp"
	"src/simulation/Object.cpp"
	"src/simulation/Simulation.cpp"
	"src/controllers/PFL.cpp"
	"src/generators/Manual.cpp"
	"src/generators/Lemniscate.cpp"
	"src/generators/Circle.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"src"
	"src/graphs"
	"src/interface"
	"src/abstract"
	"src/simulation"
	"src/controllers"
	"src/generators"
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
