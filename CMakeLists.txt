cmake_minimum_required(VERSION 3.0)
project(cityflow)

set(CMAKE_CXX_STANDARD "11" CACHE STRING "")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DRAPIDJSON_HAS_STDSTRING=1")
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
find_package(GTest)
if(GTEST_FOUND)
    enable_testing()
    add_subdirectory(tests)
endif()

if (${CMAKE_BUILD_TYPE} STREQUAL Debug)
    add_subdirectory(tools/debug)
endif()

pybind11_add_module(${PROJECT_NAME} MODULE src/cityflow.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_LIB_NAME})
if(VERSION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DVERSION=${VERSION})
endif()

