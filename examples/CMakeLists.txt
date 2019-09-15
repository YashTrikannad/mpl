add_executable(examples grid_map_generation.cpp)
target_link_libraries(examples utility)

add_executable(dfs dfs.cpp)
target_link_libraries(dfs utility)

add_executable(astar astar.cpp)
target_link_libraries(astar utility)

add_executable(bfs bfs.cpp)
target_link_libraries(bfs utility)