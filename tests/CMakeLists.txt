file(GLOB TEST_SRCS cpp/*.cpp)
set(FLAGS "-fsanitize=address -fno-omit-frame-pointer")
include_directories(${GTEST_INCLUDE_DIRS})
foreach(SRC_PATH ${TEST_SRCS})
    get_filename_component(SRC_NAME ${SRC_PATH} NAME)
    string(REPLACE ".cpp" "" EXE_NAME ${SRC_NAME})
    add_executable(${EXE_NAME} ${SRC_PATH})
    set_target_properties(${EXE_NAME} PROPERTIES APPEND_STRING PROPERTY LINK_FLAGS "${FLAGS}")
    set_target_properties(${EXE_NAME} PROPERTIES APPEND_STRING PROPERTY COMPILE_FLAGS "${FLAGS}")
    target_link_libraries(${EXE_NAME} PUBLIC ${PROJECT_LIB_NAME} ${GTEST_BOTH_LIBRARIES})
    add_test(NAME ${EXE_NAME}
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMAND ${EXE_NAME})
endforeach()