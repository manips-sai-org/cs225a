set(MMP_PANDA_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DMMP_PANDA_FOLDER="${MMP_PANDA_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/mmp_panda_example)
ADD_EXECUTABLE (controller_mmp_panda controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_mmp_panda simviz.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_mmp_panda ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_mmp_panda ${CS225A_COMMON_LIBRARIES})
