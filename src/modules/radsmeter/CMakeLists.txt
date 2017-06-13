px4_add_module(
	MODULE modules__radsmeter
	MAIN radsmeter
	COMPILE_FLAGS
	STACK_MAIN 2000
	SRCS
		radsmeter_main.cpp
	DEPENDS
		platforms__common
	)