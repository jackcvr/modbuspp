cmake_minimum_required(VERSION 3.18)

project(ModbusppTest CXX)

set(LIBMODBUS_VERSION "3.1.6")

function(add_exe name src std)
    add_executable(${name} ${src})
    target_compile_features(${name} PRIVATE cxx_std_${std})
    target_compile_options(${name} PRIVATE -Os -pedantic -Wall -Wextra -U_LIBCPP_ENABLE_CXX17_REMOVED_UNEXPECTED_FUNCTIONS)
endfunction()

include(cmake/CPM.cmake)

# C++ wrapper
CPMAddPackage(
    NAME modbuspp
    SOURCE_DIR ..
)

# libmodbus (static)
CPMAddPackage(
    NAME libmodbus
    GITHUB_REPOSITORY stephane/libmodbus
    VERSION 3.1.12
    DOWNLOAD_ONLY YES
)
if(libmodbus_ADDED)
    set(LIBMODBUS_LIB_PATH "${libmodbus_SOURCE_DIR}/src/.libs/libmodbus.a")
    if(NOT EXISTS "${LIBMODBUS_LIB_PATH}")
        execute_process(
            COMMAND sh -c "./autogen.sh \
                && ./configure CC='${CMAKE_C_COMPILER}' CXX='${CMAKE_CXX_COMPILER}' CFLAGS='-Os' CXXFLAGS='-Os' \
                    --enable-static --disable-shared --with-pic \
                && make LDFLAGS='${LDFLAGS} -all-static' -j$(nproc)"
            WORKING_DIRECTORY ${libmodbus_SOURCE_DIR}
            COMMAND_ERROR_IS_FATAL ANY
        )
    endif()
    add_library(libmodbus STATIC IMPORTED GLOBAL)
    set_target_properties(libmodbus PROPERTIES
        IMPORTED_LOCATION "${LIBMODBUS_LIB_PATH}"
        INTERFACE_INCLUDE_DIRECTORIES "${libmodbus_SOURCE_DIR}/src"
    )
endif()
add_exe(slave_static slave.cpp 23)
add_exe(master_static master.cpp 23)
target_link_libraries(slave_static PRIVATE modbuspp libmodbus)
target_link_libraries(master_static PRIVATE modbuspp libmodbus)
target_link_options(slave_static PRIVATE -static)
target_link_options(master_static PRIVATE -static)


# libmodbus (dynamic pre-installed)
find_package(PkgConfig REQUIRED)
pkg_check_modules(libmodbus REQUIRED IMPORTED_TARGET libmodbus>=${LIBMODBUS_VERSION})
add_exe(slave_dynamic slave.cpp 23)
add_exe(master_dynamic master.cpp 23)
target_link_libraries(slave_dynamic PRIVATE modbuspp PkgConfig::libmodbus)
target_link_libraries(master_dynamic PRIVATE modbuspp PkgConfig::libmodbus)
