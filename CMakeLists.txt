cmake_minimum_required(VERSION 3.0)

project(dalybms)

add_executable(dalybms main.cpp daly-bms-uart.cpp daly-bms-uart.h)

install(TARGETS dalybms RUNTIME DESTINATION bin)
