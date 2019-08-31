cmake_minimum_required(VERSION 2.8.12)
project(cityflow)

set(CMAKE_CXX_STANDARD "11" CACHE STRING "")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(POLICY CMP0063)
  cmake_policy(SET CMP0063 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

include_directories(extern/milo)

set(REQUIRED_SUBMODULES
    "extern/pybind11/CMakeLists.txt"
    "extern/rapidjson/include"
)

foreach(REQUIRED_SUBMODULE ${REQUIRED_SUBMODULES})
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${REQUIRED_SUBMODULE}")
        # update submodule 
        # https://cliutils.gitlab.io/modern-cmake/chapters/projects/submodule.html
        find_package(Git QUIET)
        if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
        # Update submodules as needed
            option(GIT_SUBMODULE "Check submodules during build" ON)
            if(GIT_SUBMODULE)
                message(STATUS "Submodule update, this may take some time...")
                execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                                RESULT_VARIABLE GIT_SUBMOD_RESULT)
                if(NOT GIT_SUBMOD_RESULT EQUAL "0")
                    message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
                endif()
            endif()
        endif()
        break()
    else()
        message(STATUS "Found Submodule: ${REQUIRED_SUBMODULE}")
    endif()
endforeach()

foreach(REQUIRED_SUBMODULE ${REQUIRED_SUBMODULES})
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${REQUIRED_SUBMODULE}")
        message(FATAL_ERROR "The submodule ${REQUIRED_SUBMODULE} was not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
    endif()
endforeach()

add_subdirectory(extern/pybind11)
include_directories(extern/rapidjson/include)

add_subdirectory(src)

# Tests
file(GLOB TEST_SRCS tests/cpp/*.cpp)
find_library(GTEST_LIB gtest "$ENV{GTEST_LIB}")

if(GTEST_LIB)
    enable_testing()
    set(FLAGS "-fsanitize=address -fno-omit-frame-pointer")
    foreach(__srcpath ${TEST_SRCS})
        get_filename_component(__srcname ${__srcpath} NAME)
        string(REPLACE ".cpp" "" __execname ${__srcname})
        add_executable(${__execname} ${__srcpath})
        set_target_properties(${__execname} PROPERTIES APPEND_STRING PROPERTY LINK_FLAGS "${FLAGS}")
        set_target_properties(${__execname} PROPERTIES APPEND_STRING PROPERTY COMPILE_FLAGS "${FLAGS}")
        target_link_libraries(${__execname} PUBLIC ${PROJECT_LIB_NAME} ${GTEST_LIB} ${Boost_LIBRARIES})
        add_test(NAME ${__execname}
                 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                 COMMAND ${__execname})
    endforeach()
endif()

pybind11_add_module(${PROJECT_NAME} MODULE src/cityflow.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_LIB_NAME})
if(VERSION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DVERSION=${VERSION})
endif()

