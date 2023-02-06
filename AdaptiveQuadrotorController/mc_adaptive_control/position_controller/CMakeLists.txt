# TODO(nathan) license

px4_add_module(
	MODULE modules__mc_adaptive_pos_control
	MAIN mc_adaptive_pos_control
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS
    INCLUDES
        include
	SRCS
        src/manual_input_handler.cpp
		src/mc_adaptive_pos_control_main.cpp
	DEPENDS
        mc_adaptive_control_utils
		mathlib
	)
