cmake_minimum_required(VERSION 3.26)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)

project(test_gjk)

############ PHYSX ############
set(PHYSX_DIR "/opt/PhysX")
include_directories(
    "${PHYSX_DIR}/physx/include"
    "${PHYSX_DIR}/physx"
    "${PHYSX_DIR}/physx/source/common/src"
    "${PHYSX_DIR}/physx/source/geomutils/include"
    "${PHYSX_DIR}/physx/source/geomutils/src")

set(PHYSX_LIB_DIR "${PHYSX_DIR}/physx/bin/linux.x86_64/release")
message(STATUS "PHYSX_LIB_DIR: ${PHYSX_LIB_DIR}")
set(PHYSX_LIB
    PhysX_static_64
    PhysXExtensions_static_64
    PhysXCharacterKinematic_static_64
    PhysXCooking_static_64
    PhysXCommon_static_64
    PhysXFoundation_static_64
    SnippetUtils_static_64
)
############ END PHYSX ###########


set(SRCS phylib.cpp main.cpp)

add_executable(test_gjk ${SRCS})
target_link_libraries(test_gjk ${PHYSX_LIB})
target_link_directories(test_gjk PRIVATE ${PHYSX_LIB_DIR})

add_custom_command(
    TARGET test_gjk POST_BUILD
    COMMENT "Copying PhysX shared libraries to output directory"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/obj/p3l.obj ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/obj/p3r.obj ${CMAKE_BINARY_DIR}
)
