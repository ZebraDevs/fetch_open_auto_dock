if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

catkin_add_gtest(test_icp
  test_icp.cpp
  ../src/icp_2d.cpp
)
target_link_libraries(test_icp
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_low_pass_filter
  test_low_pass_filter.cpp
  ../src/linear_pose_filter_2d.cpp
)
target_link_libraries(test_low_pass_filter
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
