set(OCEAN1_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DOCEAN1_FOLDER="${OCEAN1_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/ocean1_example)
ADD_EXECUTABLE (controller_ocean1 controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_ocean1 simviz.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_ocean1 ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_ocean1 ${CS225A_COMMON_LIBRARIES})
