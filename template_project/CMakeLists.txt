# Add sai2-primitives
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/template_project)
ADD_EXECUTABLE (simviz_template simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_template controller.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_template ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_template ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})


