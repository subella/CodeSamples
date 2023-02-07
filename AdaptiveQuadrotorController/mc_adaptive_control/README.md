# Adaptive Geometric Controller for Multicopters

This contains a library and two modules for PX4 that together form an implementation of "Farhad A. Goodarzi, Daewon Lee, & Taeyoung Lee (2015). Geometric Adaptive Tracking Control of a Quadrotor Unmanned Aerial Vehicle on SE(3) for Agile Maneuvers. Journal of Dynamic Systems, Measurement, and Control, 137(9)."

This should be built as a submodule to PX4.

## Details for reviewer
This work was done in collaboration with my colleague, Nathan Hughes. The majority of my contribution is in the actual implementation of the control law (the math in [geometric.cpp](https://github.com/subella/CodeSamples/blob/main/AdaptiveQuadrotorController/mc_adaptive_control/controller_utilities/src/geometric_controller.cpp), [geometric.hpp](https://github.com/subella/CodeSamples/blob/main/AdaptiveQuadrotorController/mc_adaptive_control/controller_utilities/include/mc_adaptive_control_utils/geometric_controller.hpp), and [utest_geometric.cpp](https://github.com/subella/CodeSamples/blob/main/AdaptiveQuadrotorController/mc_adaptive_control/controller_utilities/test/utest_geometric.cpp)), with minor additions elsewhere in the code base to connect the pipeline.
