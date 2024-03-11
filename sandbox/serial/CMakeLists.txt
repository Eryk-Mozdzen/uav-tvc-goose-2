cmake_minimum_required(VERSION 3.16)

project(serial)

add_executable(${PROJECT_NAME}
	"src/main.cpp"
	"../../shared/protocol/protocol.c"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"include"
	"../../shared/protocol"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-Wall
	-Wextra
	-Wpedantic
)
