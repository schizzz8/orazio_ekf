add_executable(ekf_app
  ekf_app.cpp
)

target_link_libraries(ekf_app
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ekf_library
)

