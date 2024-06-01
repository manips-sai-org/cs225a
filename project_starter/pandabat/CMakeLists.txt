set(PANDABAT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPANDABAT_FOLDER="${PANDABAT_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/panda_bat)
ADD_EXECUTABLE (simviz_panda_bat simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_panda_bat controller.cpp ${CS225A_COMMON_SOURCE})
# ADD_EXECUTABLE (controller_panda_bat2 controller2.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_panda_bat ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_panda_bat ${CS225A_COMMON_LIBRARIES})
# TARGET_LINK_LIBRARIES (controller_panda_bat2 ${CS225A_COMMON_LIBRARIES})
