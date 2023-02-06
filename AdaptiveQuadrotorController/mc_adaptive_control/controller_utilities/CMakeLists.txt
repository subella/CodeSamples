px4_add_library(mc_adaptive_control_utils
    src/geometric_controller.cpp
    src/px4_helpers.cpp
    src/transform_utils.cpp)

target_include_directories(mc_adaptive_control_utils PUBLIC include)

px4_add_unit_gtest(SRC test/utest_geometric.cpp LINKLIBS mc_adaptive_control_utils)
