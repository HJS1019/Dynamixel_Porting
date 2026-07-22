px4_add_module(
	MODULE drivers__dynamixel
	MAIN dynamixel
	COMPILE_FLAGS
	SRCS
		dynamixel.cpp
		dynamixel.hpp
	DEPENDS
		uORB
)
