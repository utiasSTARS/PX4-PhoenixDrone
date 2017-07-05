px4_add_module(
	MODULE drivers__tsfmu
	MAIN tsfmu
	STACK_MAIN 1200
	COMPILE_FLAGS
		-Weffc++
	SRCS
		tsfmu.cpp
	DEPENDS
		platforms__common
	)