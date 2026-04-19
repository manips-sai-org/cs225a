set(APP_NAME hw2)
# create an executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/${APP_NAME})
add_executable(${APP_NAME} ${APP_NAME}.cpp)
add_executable(${APP_NAME}-simviz ${APP_NAME}-simviz.cpp)

# and link the library against the executable
target_link_libraries(${APP_NAME} ${CS225A_COMMON_LIBRARIES})
target_link_libraries(${APP_NAME}-simviz ${CS225A_COMMON_LIBRARIES})
