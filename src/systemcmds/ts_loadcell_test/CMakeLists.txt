px4_add_module(
	MODULE systemcmds__ts_loadcell_test
	MAIN loadcell_test
	STACK_MAIN 4096
	COMPILE_FLAGS
	SRCS
		ts_loadcell_test.cpp
		step_test.cpp
		motor_test.cpp
		motor_step_test.cpp
		servo_step_test.cpp
	DEPENDS
		platforms__common
	)