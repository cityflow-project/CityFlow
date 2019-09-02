set(SOURCE_FILES simple_run.cpp)

find_package(Boost 1.50 REQUIRED COMPONENTS
        program_options
        )
foreach(SRC_PATH ${SOURCE_FILES})
    get_filename_component(SRC_NAME ${SRC_PATH} NAME)
    string(REPLACE ".cpp" "" EXE_NAME ${SRC_NAME})
    add_executable(${EXE_NAME} ${SRC_PATH})
    set_target_properties(${EXE_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden")
    target_link_libraries(${EXE_NAME} PRIVATE ${PROJECT_LIB_NAME} ${Boost_LIBRARIES})
endforeach()