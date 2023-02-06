px4_add_module(
	MODULE modules__mc_adaptive_att_control
	MAIN mc_adaptive_att_control
	STACK_MAIN 1200
	STACK_MAX 3500
	COMPILE_FLAGS
    INCLUDES
        include
	SRCS
		src/mc_adaptive_att_control_main.cpp
	DEPENDS
        mc_adaptive_control_utils
		mathlib
	)
