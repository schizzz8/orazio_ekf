add_library(ekf_library
  ekf.cpp ekf.h
)

target_link_libraries(ekf_library
  ${catkin_LIBRARIES}
)
