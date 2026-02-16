cmake_minimum_required(VERSION 3.20)

project(ModbusppTest CXX)

include(../cmake/CPM.cmake)

CPMAddPackage(
    NAME modbuspp
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..
    OPTIONS "MODBUSPP_BUILD_LIBMODBUS ON"
)

function(add name src std)
    add_executable(${name} ${src})
    target_compile_features(${name} PRIVATE cxx_std_${std})
    target_compile_options(${name} PRIVATE -Os -pedantic -Wall -Wextra -U_LIBCPP_ENABLE_CXX17_REMOVED_UNEXPECTED_FUNCTIONS)
endfunction()

add(slave_static slave.cpp 23)
add(master_static master.cpp 23)
target_link_libraries(slave_static PRIVATE modbuspp::static)
target_link_libraries(master_static PRIVATE modbuspp::static)
target_link_options(slave_static PRIVATE -static)
target_link_options(master_static PRIVATE -static)

add(slave_shared slave.cpp 23)
add(master_shared master.cpp 23)
target_link_libraries(slave_shared PRIVATE modbuspp::shared)
target_link_libraries(master_shared PRIVATE modbuspp::shared)
