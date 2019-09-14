add_library(graph_fixture graph_fixture.h)

add_executable(graph_test graph_test.cpp)
target_link_libraries(graph_test ${GTEST_LIBRARIES} pthread)
