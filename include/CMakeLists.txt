add_library(planner mpl/planner.h mpl/dfs.h mpl/bfs.h mpl/astar.h mpl/jps.h)

set_target_properties(planner PROPERTIES LINKER_LANGUAGE CXX)