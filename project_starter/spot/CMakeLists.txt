# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/spot_example)
ADD_EXECUTABLE (controller_spot controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_spot simviz.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_spot ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_spot ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_spot.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY spot.urdf DESTINATION ${APP_RESOURCE_DIR})
