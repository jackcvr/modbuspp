cmake_minimum_required(VERSION 3.20)

project(Modbuspp CXX)

set(MODBUSPP_LIBMODBUS_VERSION "3.1.12")
set(MODBUSPP_LIBMODBUS_VERSION_MIN "3.1.6")

option(MODBUSPP_BUILD_LIBMODBUS "Build bundled libmodbus" OFF)

add_library(modbuspp_shared INTERFACE)
add_library(modbuspp_static INTERFACE)
add_library(modbuspp::shared ALIAS modbuspp_shared)
add_library(modbuspp::static ALIAS modbuspp_static)
target_include_directories(modbuspp_shared INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(modbuspp_static INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MODBUSPP_BUILD_LIBMODBUS)
    include(cmake/CPM.cmake)
    CPMAddPackage(
        NAME libmodbus
        GITHUB_REPOSITORY stephane/libmodbus
        VERSION ${MODBUSPP_LIBMODBUS_VERSION}
        DOWNLOAD_ONLY YES
    )
    if(libmodbus_ADDED)
        set(LIBMODBUS_LIB_PATH "${libmodbus_SOURCE_DIR}/src/.libs/libmodbus.a")
        if(NOT EXISTS "${LIBMODBUS_LIB_PATH}")
            execute_process(
                COMMAND sh -c "./autogen.sh \
                    && ./configure CC='${CMAKE_C_COMPILER}' CXX='${CMAKE_CXX_COMPILER}' CFLAGS='-Os' CXXFLAGS='-Os' \
                        --enable-static --disable-shared --with-pic --without-documentation \
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
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(libmodbus_system QUIET IMPORTED_TARGET libmodbus>=${MODBUSPP_LIBMODBUS_VERSION_MIN})
    if(libmodbus_system_FOUND)
        message(STATUS "libmodbus ${libmodbus_system_VERSION} found via pkg-config")
        target_include_directories(modbuspp_shared INTERFACE ${libmodbus_system_INCLUDE_DIRS})
        target_link_libraries(modbuspp_shared INTERFACE PkgConfig::libmodbus_system)
    else()
        message(STATUS "libmodbus is not installed.")
    endif()
else()
    message(STATUS "PkgConfig not found; skipping system libmodbus detection.")
endif()

if(TARGET libmodbus)
    target_link_libraries(modbuspp_static INTERFACE libmodbus)
    add_library(modbuspp::modbuspp ALIAS modbuspp_static)
elseif(libmodbus_system_FOUND)
    add_library(modbuspp::modbuspp ALIAS modbuspp_shared)
else()
    message(WARNING "Neither libmodbus target nor system libmodbus found. 'modbuspp::modbuspp' will not be available.")
endif()
