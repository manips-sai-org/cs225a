# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/collision_demo)
ADD_EXECUTABLE (simviz_demo simviz_demo.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_demo controller_demo.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_demo ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_demo ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_collision.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY cup.urdf DESTINATION ${APP_RESOURCE_DIR})
