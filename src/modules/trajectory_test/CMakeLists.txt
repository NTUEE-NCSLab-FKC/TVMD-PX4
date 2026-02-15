px4_add_module(
	MODULE modules__trajectory_test
	MAIN trajectory_test
	SRCS
		TrajectoryTest.cpp
	DEPENDS
		mathlib
		px4_work_queue
	)
