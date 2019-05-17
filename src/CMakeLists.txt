set(PROJECT_HEADER_FILES
	utility/config.h
	utility/utility.h
	engine/engine.h
	flow/flow.h
	flow/route.h
	roadnet/roadnet.h
	roadnet/trafficlight.h
	vehicle/router.h
	vehicle/vehicle.h
)

set(PROJECT_SOURCE_FILES
	utility/utility.cpp
	engine/engine.cpp
	flow/flow.cpp
	flow/route.cpp
	roadnet/roadnet.cpp
	roadnet/trafficlight.cpp
	vehicle/router.cpp
	vehicle/vehicle.cpp
)

set(PROJECT_LIB_NAME ${PROJECT_NAME}_lib CACHE INTERNAL "")

find_package(Boost 1.50 REQUIRED COMPONENTS
	thread
)

find_package(Threads REQUIRED)

add_library(${PROJECT_LIB_NAME} ${PROJECT_HEADER_FILES} ${PROJECT_SOURCE_FILES})
target_link_libraries(${PROJECT_LIB_NAME} PRIVATE jsoncpp Threads::Threads ${Boost_LIBRARIES})
target_include_directories(${PROJECT_LIB_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${Boost_INCLUDE_DIRS})