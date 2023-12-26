
add_executable (grasp src/main.cpp src/canAPI.cpp src/RockScissorsPaper.cpp)
target_link_libraries (grasp BHand pcanbasic pthread)

add_executable (test_can_comm src/test_can_comm.cpp src/canAPI.cpp src/RockScissorsPaper.cpp)
target_link_libraries (test_can_comm BHand pcanbasic pthread)

install (TARGETS grasp DESTINATION ${PROJECT_BINARY_DIR}/bin)
install (TARGETS test_can_comm DESTINATION ${PROJECT_BINARY_DIR}/bin)