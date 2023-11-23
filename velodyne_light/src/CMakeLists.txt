# build the driver node
add_executable(velodyne_light_node 
  velodyne_light.cc 
  input.cc 
  driver.cc
)

target_link_libraries(velodyne_light_node
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS velodyne_light_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
