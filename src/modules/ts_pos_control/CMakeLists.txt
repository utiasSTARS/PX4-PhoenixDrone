px4_add_module(
	MODULE modules__ts_pos_control
	MAIN ts_pos_control
	COMPILE_FLAGS
	STACK_MAIN 1200
	SRCS
		ts_pos_control_main.cpp
	DEPENDS
		platforms__common
	)