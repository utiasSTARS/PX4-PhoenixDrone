px4_add_module(
	MODULE lib__LoopTimer
	COMPILE_FLAGS
	SRCS
		LoopTimer.cpp
	DEPENDS
		platforms__common
	)