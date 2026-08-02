px4_add_module(
	MODULE drivers__dynamixel
	MAIN dynamixel
	COMPILE_FLAGS
	SRCS
		dynamixel.cpp
		dynamixel.hpp
		DynamixelPositionMapping.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		uORB
)

px4_add_unit_gtest(SRC DynamixelPositionMappingTest.cpp)
