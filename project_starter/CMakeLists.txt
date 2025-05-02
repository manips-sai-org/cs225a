# project starter folder
set(PROJECT_STARTER_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPROJECT_STARTER_FOLDER="${PROJECT_STARTER_FOLDER}")

add_subdirectory(panda)
add_subdirectory(mmp_panda)
add_subdirectory(toro)
add_subdirectory(ocean1)
add_subdirectory(spot)
