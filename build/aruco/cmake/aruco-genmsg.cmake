# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "aruco: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iaruco:/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(aruco_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg" NAME_WE)
add_custom_target(_aruco_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "aruco" "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg" ""
)

get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg" NAME_WE)
add_custom_target(_aruco_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "aruco" "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco
)
_generate_msg_cpp(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco
)

### Generating Services

### Generating Module File
_generate_module_cpp(aruco
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(aruco_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(aruco_generate_messages aruco_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg" NAME_WE)
add_dependencies(aruco_generate_messages_cpp _aruco_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg" NAME_WE)
add_dependencies(aruco_generate_messages_cpp _aruco_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_gencpp)
add_dependencies(aruco_gencpp aruco_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco
)
_generate_msg_eus(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco
)

### Generating Services

### Generating Module File
_generate_module_eus(aruco
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(aruco_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(aruco_generate_messages aruco_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg" NAME_WE)
add_dependencies(aruco_generate_messages_eus _aruco_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg" NAME_WE)
add_dependencies(aruco_generate_messages_eus _aruco_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_geneus)
add_dependencies(aruco_geneus aruco_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco
)
_generate_msg_lisp(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco
)

### Generating Services

### Generating Module File
_generate_module_lisp(aruco
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(aruco_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(aruco_generate_messages aruco_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg" NAME_WE)
add_dependencies(aruco_generate_messages_lisp _aruco_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg" NAME_WE)
add_dependencies(aruco_generate_messages_lisp _aruco_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_genlisp)
add_dependencies(aruco_genlisp aruco_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco
)
_generate_msg_nodejs(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco
)

### Generating Services

### Generating Module File
_generate_module_nodejs(aruco
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(aruco_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(aruco_generate_messages aruco_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg" NAME_WE)
add_dependencies(aruco_generate_messages_nodejs _aruco_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg" NAME_WE)
add_dependencies(aruco_generate_messages_nodejs _aruco_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_gennodejs)
add_dependencies(aruco_gennodejs aruco_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco
)
_generate_msg_py(aruco
  "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco
)

### Generating Services

### Generating Module File
_generate_module_py(aruco
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(aruco_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(aruco_generate_messages aruco_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/vel.msg" NAME_WE)
add_dependencies(aruco_generate_messages_py _aruco_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cuongnq23/PycharmProjects/Robotics/catkin_ws/src/aruco/msg/Angle.msg" NAME_WE)
add_dependencies(aruco_generate_messages_py _aruco_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(aruco_genpy)
add_dependencies(aruco_genpy aruco_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aruco_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aruco
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aruco
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aruco
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aruco
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aruco
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
