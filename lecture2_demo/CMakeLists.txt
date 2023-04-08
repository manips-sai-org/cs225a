# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/lecture2_demo)
ADD_EXECUTABLE (simviz simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (visualization visualization.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simulation simulation.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller controller.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (visualization ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simulation ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY rbot.urdf DESTINATION ${APP_RESOURCE_DIR})
