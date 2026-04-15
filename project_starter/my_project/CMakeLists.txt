set(MY_PROJECT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DMY_PROJECT_FOLDER="${MY_PROJECT_FOLDER}")

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/my_project)
ADD_EXECUTABLE (simviz_my_project simviz.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_my_project controller.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_my_project ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_my_project ${CS225A_COMMON_LIBRARIES})
