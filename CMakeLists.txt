cmake_minimum_required(VERSION 3.18)

project(Modbuspp CXX)

add_library(modbuspp INTERFACE)
target_include_directories(modbuspp INTERFACE include)
