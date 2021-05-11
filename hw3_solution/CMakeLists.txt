# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/hw3_solution)
ADD_EXECUTABLE (hw3_sol hw3.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw3_sol simviz_hw3.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_hw3_sol_slow simviz_hw3_slow.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (hw3_sol ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw3_sol ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_hw3_sol_slow ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
