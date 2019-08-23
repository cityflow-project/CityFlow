cmake_minimum_required(VERSION 2.8.12)
project(cityflow)

set(CMAKE_CXX_STANDARD "11" CACHE STRING "")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

find_package(Boost 1.50 REQUIRED COMPONENTS
	program_options
)

add_executable(${PROJECT_NAME}_test src/test.cpp)
target_link_libraries(${PROJECT_NAME}_test PUBLIC ${PROJECT_LIB_NAME} ${Boost_LIBRARIES})

pybind11_add_module(${PROJECT_NAME} MODULE src/simulator.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_LIB_NAME})
if(VERSION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DVERSION=${VERSION})
endif()

