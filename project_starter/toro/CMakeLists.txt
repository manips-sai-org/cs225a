set(TORO_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DTORO_FOLDER="${TORO_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/toro_example)
ADD_EXECUTABLE (controller_toro controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_toro simviz.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_toro ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_toro ${CS225A_COMMON_LIBRARIES})
