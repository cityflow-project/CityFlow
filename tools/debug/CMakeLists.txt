set(SOURCE_FILES simple_run.cpp)

foreach(SRC_PATH ${SOURCE_FILES})
    get_filename_component(SRC_NAME ${SRC_PATH} NAME)
    string(REPLACE ".cpp" "" EXE_NAME ${SRC_NAME})
    add_executable(${EXE_NAME} ${SRC_PATH})
    set_target_properties(${EXE_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden")
    target_link_libraries(${EXE_NAME} PRIVATE ${PROJECT_LIB_NAME})
endforeach()