cmake_minimum_required(VERSION 3.1)
project(rsp_tcp VERSION 0.1.0)

#find_library(USB_LIBRARY NAMES usb usb-1.0)
find_package(Threads REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(LibSDRplay)

if (NOT LIBSDRPLAY_FOUND)
    message(FATAL_ERROR "SDRPlay development files not found...")
endif ()
message(STATUS "LIBSDRPLAY_INCLUDE_DIRS - ${LIBSDRPLAY_INCLUDE_DIRS}")
message(STATUS "LIBSDRPLAY_LIBRARIES - ${LIBSDRPLAY_LIBRARIES}")


set(CMAKE_BUILD_TYPE Release)

set(SOURCE_FILES rsp_tcp.c)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ggdb")
add_definitions(-D_GNU_SOURCE)
add_definitions(-DSERVER_VERSION="${PROJECT_VERSION}")
include_directories(${LIBSDRPLAY_INCLUDE_DIRS})

add_executable(rsp_tcp ${SOURCE_FILES} )
target_link_libraries(rsp_tcp ${LIBSDRPLAY_LIBRARIES} Threads::Threads)
install(TARGETS rsp_tcp DESTINATION bin)

set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_VENDOR "F4FHH Nicolas")
set(CPACK_PACKAGE_CONTACT "F4FHH Nicolas (f4fhh@ducor.fr)")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "a rtl_tcp compatible IQ server for the RSP range of SDRPlay SDR")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_STRIP_FILES true)
set(CPACK_PACKAGE_EXECUTABLES rsp_tcp)

include(CPack)
