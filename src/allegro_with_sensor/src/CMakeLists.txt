
add_executable (grasp src/main.cpp src/canAPI.cpp src/RockScissorsPaper.cpp)
target_link_libraries (grasp BHand pcanbasic pthread)

add_executable (ft_sensing src/main_wristFT.cpp src/canAPI.cpp src/RockScissorsPaper.cpp)
target_link_libraries (ft_sensing BHand pcanbasic pthread)

install (TARGETS grasp DESTINATION ${PROJECT_BINARY_DIR}/bin)
install (TARGETS ft_sensing DESTINATION ${PROJECT_BINARY_DIR}/bin)