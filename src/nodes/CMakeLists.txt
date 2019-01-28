add_executable(message_dumper_node
  message_dumper_node.cpp
)

target_link_libraries(message_dumper_node
  ${catkin_LIBRARIES}
)

