# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/hw1)
ADD_EXECUTABLE (hw1 hw1.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw1 simviz_hw1.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw1_slow simviz_hw1_slow.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw1 ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw1 ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw1_slow ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_controller.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm_simulation.urdf DESTINATION ${APP_RESOURCE_DIR})
