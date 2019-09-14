add_library(utility STATIC mpl/eigen_utility.h mpl/common_utility.h mpl/grid_map_generator/random_grid_map_generator.h
        mpl/visualization_utility.h)

set_target_properties(utility PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(utility ${OpenCV_LIBS})