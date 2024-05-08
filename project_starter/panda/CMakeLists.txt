set(PANDA_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPANDA_FOLDER="${PANDA_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/panda_gripper_example)
ADD_EXECUTABLE (simviz_panda_gripper simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_panda_gripper controller.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_panda_gripper ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_panda_gripper ${CS225A_COMMON_LIBRARIES})
