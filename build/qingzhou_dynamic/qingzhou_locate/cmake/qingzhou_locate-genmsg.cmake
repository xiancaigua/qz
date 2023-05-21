# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qingzhou_locate: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qingzhou_locate_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv" NAME_WE)
add_custom_target(_qingzhou_locate_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qingzhou_locate" "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(qingzhou_locate
  "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qingzhou_locate
)

### Generating Module File
_generate_module_cpp(qingzhou_locate
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qingzhou_locate
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qingzhou_locate_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qingzhou_locate_generate_messages qingzhou_locate_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv" NAME_WE)
add_dependencies(qingzhou_locate_generate_messages_cpp _qingzhou_locate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qingzhou_locate_gencpp)
add_dependencies(qingzhou_locate_gencpp qingzhou_locate_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qingzhou_locate_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(qingzhou_locate
  "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qingzhou_locate
)

### Generating Module File
_generate_module_eus(qingzhou_locate
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qingzhou_locate
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qingzhou_locate_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qingzhou_locate_generate_messages qingzhou_locate_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv" NAME_WE)
add_dependencies(qingzhou_locate_generate_messages_eus _qingzhou_locate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qingzhou_locate_geneus)
add_dependencies(qingzhou_locate_geneus qingzhou_locate_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qingzhou_locate_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(qingzhou_locate
  "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qingzhou_locate
)

### Generating Module File
_generate_module_lisp(qingzhou_locate
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qingzhou_locate
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qingzhou_locate_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qingzhou_locate_generate_messages qingzhou_locate_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv" NAME_WE)
add_dependencies(qingzhou_locate_generate_messages_lisp _qingzhou_locate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qingzhou_locate_genlisp)
add_dependencies(qingzhou_locate_genlisp qingzhou_locate_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qingzhou_locate_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(qingzhou_locate
  "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qingzhou_locate
)

### Generating Module File
_generate_module_nodejs(qingzhou_locate
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qingzhou_locate
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qingzhou_locate_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qingzhou_locate_generate_messages qingzhou_locate_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv" NAME_WE)
add_dependencies(qingzhou_locate_generate_messages_nodejs _qingzhou_locate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qingzhou_locate_gennodejs)
add_dependencies(qingzhou_locate_gennodejs qingzhou_locate_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qingzhou_locate_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(qingzhou_locate
  "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qingzhou_locate
)

### Generating Module File
_generate_module_py(qingzhou_locate
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qingzhou_locate
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qingzhou_locate_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qingzhou_locate_generate_messages qingzhou_locate_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cquer/2023_qingzhou/src/qingzhou_dynamic/qingzhou_locate/srv/RobotLocation.srv" NAME_WE)
add_dependencies(qingzhou_locate_generate_messages_py _qingzhou_locate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qingzhou_locate_genpy)
add_dependencies(qingzhou_locate_genpy qingzhou_locate_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qingzhou_locate_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qingzhou_locate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qingzhou_locate
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(qingzhou_locate_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qingzhou_locate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qingzhou_locate
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(qingzhou_locate_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qingzhou_locate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qingzhou_locate
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(qingzhou_locate_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qingzhou_locate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qingzhou_locate
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(qingzhou_locate_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qingzhou_locate)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qingzhou_locate\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qingzhou_locate
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(qingzhou_locate_generate_messages_py std_msgs_generate_messages_py)
endif()
