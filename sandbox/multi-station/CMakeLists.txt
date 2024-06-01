cmake_minimum_required(VERSION 3.16)

project(multi-station)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
	"src/main.cpp"
	"src/Station.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"include"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O0
	-Wall
	-Wextra
	-Wpedantic
	-std=c++20
)

target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen
)
