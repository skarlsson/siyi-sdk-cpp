set(TEST_LIBS
        siyi_sdk_cpp_s
        gtest
        glog
        z
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

set(target a8_test)
add_executable(${target} ${target}.cpp)
target_link_libraries(${target} ${TEST_LIBS})
add_test(NAME ${target} COMMAND ${target})
set_tests_properties(${target} PROPERTIES LABELS ${target})
