project(ec_dds)
cmake_minimum_required(VERSION 3.0)

set(ENABLE_XENO OFF CACHE BOOL "Enable xenomai real-time features")

if ( ${ENABLE_XENO} )
    option(CMAKE_USE_XENOMAI "Use Xenomai libraries" ON)
    find_package(Xenomai REQUIRED)
    if(TARGET Xenomai::cobalt)
        set(USE_COBALT True)
    endif()
else ()
    option(CMAKE_USE_XENOMAI "Use Xenomai libraries" OFF)
endif()

find_package(ecat_client_advr REQUIRED)

# Find requirements
# if(NOT fastcdr_FOUND)
#     find_package(fastcdr REQUIRED)
# endif()

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(Eigen3 REQUIRED)
# if(NOT fastrtps_FOUND)
#     find_package(fastrtps REQUIRED)
# endif()

#Check C++17
include(CheckCXXCompilerFlag)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    check_cxx_compiler_flag(-std=c++17 SUPPORTS_CXX17)
    if(NOT SUPPORTS_CXX17)
        message(FATAL_ERROR "Compiler doesn't support C++17")
    endif()
endif()

file(GLOB DDS_MESSAGE_SOURCES_CXX "src/messages/ecat*.cxx")
file(GLOB DDS_MESSAGE_SOURCES_CPP "src/messages/ecat*.cpp")

add_executable(${PROJECT_NAME} 
     	       src/ec_hydraulic_control.cpp
               src/utils.cpp
               src/custom_pid.cpp
               ${DDS_MESSAGE_SOURCES_CXX}
               ${DDS_MESSAGE_SOURCES_CPP})

target_include_directories(${PROJECT_NAME}
PUBLIC
    include
    src/messages
    ${EIGEN3_INCLUDE_DIR}
)

if ( Xenomai_FOUND )
    xenomai_target_bootstrap(${PROJECT_NAME} SKINS posix)
    xenomai_target_bootstrap(${PROJECT_NAME} SKINS posix)
endif()
      
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ecat_client_advr::ec_library
    ecat_master::tools_utils
    fastrtps
    fastcdr
    fastdds::optionparser
    fmt
    )

install(TARGETS ${PROJECT_NAME}
        DESTINATION bin
        COMPONENT applications)