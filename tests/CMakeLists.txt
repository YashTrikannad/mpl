add_library(graph_fixture graph_fixture.h)
set_target_properties(graph_fixture PROPERTIES LINKER_LANGUAGE CXX)

add_library(planner_fixture planner_fixture.h)
set_target_properties(planner_fixture PROPERTIES LINKER_LANGUAGE CXX)

add_executable(planner_test planner_test.cpp jps_test.cpp)
target_link_libraries(planner_test ${GTEST_LIBRARIES} pthread)

add_executable(graph_test graph_test.cpp)
target_link_libraries(graph_test ${GTEST_LIBRARIES} pthread)



