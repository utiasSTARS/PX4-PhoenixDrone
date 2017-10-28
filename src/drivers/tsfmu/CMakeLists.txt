px4_add_module(
	MODULE drivers__tsfmu
	MAIN tsfmu
	STACK_MAIN 3600
	COMPILE_FLAGS
		-Weffc++
	SRCS
		tsfmu.cpp
	DEPENDS
		platforms__common
	)