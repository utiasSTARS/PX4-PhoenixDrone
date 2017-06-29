px4_add_module(
	MODULE systemcmds__actuator_test
	MAIN actuator_test
	STACK_MAIN 4096
	COMPILE_FLAGS
	SRCS
		actuator_test.c
	DEPENDS
		platforms__common
	)