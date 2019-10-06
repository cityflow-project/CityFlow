set(PROJECT_HEADER_FILES
    utility/config.h
    utility/utility.h
    utility/barrier.h
    utility/optionparser.h
    engine/archive.h
    engine/engine.h
    flow/flow.h
    flow/route.h
    roadnet/roadnet.h
    roadnet/trafficlight.h
    vehicle/router.h
    vehicle/vehicle.h
    vehicle/lanechange.h
)

set(PROJECT_SOURCE_FILES
    utility/utility.cpp
    utility/barrier.cpp
    engine/archive.cpp
    engine/engine.cpp
    flow/flow.cpp
    roadnet/roadnet.cpp
    roadnet/trafficlight.cpp
    vehicle/router.cpp
    vehicle/vehicle.cpp
    vehicle/lanechange.cpp)

set(PROJECT_LIB_NAME ${PROJECT_NAME}_lib CACHE INTERNAL "")

find_package(Threads REQUIRED)

add_library(${PROJECT_LIB_NAME} ${PROJECT_HEADER_FILES} ${PROJECT_SOURCE_FILES})
set_target_properties(${PROJECT_LIB_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden")
target_link_libraries(${PROJECT_LIB_NAME} PRIVATE Threads::Threads)
target_include_directories(${PROJECT_LIB_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})